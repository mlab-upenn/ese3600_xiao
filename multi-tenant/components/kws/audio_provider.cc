/* Improved Audio Provider
 * Fixes for XIAO ESP32S3 Sense:
 * - Larger buffers (80KB instead of 40KB)
 * - Better DMA configuration
 * - Proper I2S timeout settings
 * - Buffer health monitoring
 * - Accurate timestamps
 */

#include "audio_provider.h"

#include <cstdlib>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "driver/i2s_pdm.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "ringbuf.h"
#include "micro_model_settings.h"

using namespace std;

static const char* TAG = "TF_LITE_AUDIO_PROVIDER";

ringbuf_t* g_audio_capture_buffer;
volatile int32_t g_latest_audio_timestamp = 0;

// Statistics tracking (optional, for debugging)
volatile uint32_t g_buffer_overflow_count = 0;
volatile uint32_t g_total_samples_captured = 0;

// History buffer
constexpr int32_t history_samples_to_keep =
    ((kFeatureDurationMs - kFeatureStrideMs) *
     (kAudioSampleFrequency / 1000));

constexpr int32_t new_samples_to_get =
    (kFeatureStrideMs * (kAudioSampleFrequency / 1000));

// IMPROVED: Larger buffer for better reliability
const int32_t kAudioCaptureBufferSize = 80000;  // 80KB = ~5 seconds
const int32_t pdm_bytes_to_read = 3200;

namespace {
int16_t g_audio_output_buffer[kMaxAudioSampleSize * 32];
bool g_is_audio_initialized = false;
int16_t g_history_buffer[history_samples_to_keep];

uint8_t g_pdm_read_buffer[pdm_bytes_to_read] = {};
i2s_chan_handle_t g_rx_chan = nullptr;
gpio_num_t pdm_gpio_clk = GPIO_NUM_42;
gpio_num_t pdm_gpio_din = GPIO_NUM_41;
}

static void pdm_init(void) {
  if (g_rx_chan != nullptr) {
    return;
  }
  
  // IMPROVED: Better DMA configuration
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num  = 6;    // More descriptors (was 3)
  chan_cfg.dma_frame_num = 512;  // Larger frames (was 256)
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &g_rx_chan));

  i2s_pdm_rx_config_t pdm_cfg = {
      .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(16000),
      .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                  I2S_SLOT_MODE_MONO),
      .gpio_cfg = {
          .clk = pdm_gpio_clk,
          .din = pdm_gpio_din,
          .invert_flags = { .clk_inv = false },
      },
  };
  
  ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(g_rx_chan, &pdm_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(g_rx_chan));
  
  ESP_LOGI(TAG, "PDM initialized: GPIO CLK=%d DIN=%d, DMA %d×%d", 
           pdm_gpio_clk, pdm_gpio_din, chan_cfg.dma_desc_num, chan_cfg.dma_frame_num);
}

static void CaptureSamples(void* arg) {
  pdm_init();
  
  ESP_LOGI(TAG, "Audio capture task started");
  
  while (1) {
    size_t bytes_read = 0;

    // FIXED: Use portMAX_DELAY for blocking wait (not timeout)
    // This is correct for continuous audio streaming
    esp_err_t err = i2s_channel_read(g_rx_chan, g_pdm_read_buffer,
                                     pdm_bytes_to_read, &bytes_read,
                                     portMAX_DELAY);
    
    if (err == ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "I2S read timed out");
      continue;
    } else if (err != ESP_OK) {
      ESP_LOGE(TAG, "I2S read failed: %s", esp_err_to_name(err));
      vTaskDelay(pdMS_TO_TICKS(10));  // Brief delay on error
      continue;
    }

    if (bytes_read == 0) {
      ESP_LOGD(TAG, "No I2S data");
      continue;
    }
    
    if (bytes_read < pdm_bytes_to_read) {
      ESP_LOGD(TAG, "Partial I2S read: %d/%d", bytes_read, pdm_bytes_to_read);
    }

    // Check available space
    int available = rb_available(g_audio_capture_buffer);
    if (available < bytes_read) {
      ESP_LOGW(TAG, "Buffer full, dropping %d bytes", bytes_read - available);
      g_buffer_overflow_count++;
      bytes_read = available;
      if (bytes_read == 0) {
        continue;
      }
    }

    // Write to ring buffer
    int bytes_written = rb_write(g_audio_capture_buffer,
                                 (uint8_t*)g_pdm_read_buffer, 
                                 bytes_read, 
                                 pdMS_TO_TICKS(100));
    
    if (bytes_written <= 0) {
      ESP_LOGW(TAG, "Could not write to ring buffer");
      g_buffer_overflow_count++;
      continue;
    } else if (bytes_written < bytes_read) {
      ESP_LOGD(TAG, "Partial write: %d/%d", bytes_written, bytes_read);
      g_buffer_overflow_count++;
    }

    // Update timestamp based on actual samples written
    int samples_written = bytes_written / 2;
    g_total_samples_captured += samples_written;
    g_latest_audio_timestamp = (g_total_samples_captured * 1000) / kAudioSampleFrequency;
  }
  
  vTaskDelete(NULL);
}

TfLiteStatus InitAudioRecording() {
  ESP_LOGI(TAG, "Initializing audio (buffer: %d bytes = %.1f sec)",
           kAudioCaptureBufferSize,
           (float)kAudioCaptureBufferSize / (kAudioSampleFrequency * 2));
  
  g_audio_capture_buffer = rb_init("tf_ringbuffer", kAudioCaptureBufferSize);
  if (!g_audio_capture_buffer) {
    ESP_LOGE(TAG, "Error creating ring buffer");
    return kTfLiteError;
  }

  // Reset statistics
  g_buffer_overflow_count = 0;
  g_total_samples_captured = 0;
  g_latest_audio_timestamp = 0;

  // Create capture task
  xTaskCreate(CaptureSamples, "CaptureSamples", 1024 * 4, NULL, 10, NULL);
  
  // Wait for first audio data
  int wait_count = 0;
  while (!g_latest_audio_timestamp && wait_count < 100) {
    vTaskDelay(pdMS_TO_TICKS(10));
    wait_count++;
  }
  
  if (!g_latest_audio_timestamp) {
    ESP_LOGE(TAG, "Timeout waiting for audio data");
    return kTfLiteError;
  }
  
  ESP_LOGI(TAG, "Audio recording started successfully");
  return kTfLiteOk;
}

TfLiteStatus GetAudioSamples(int start_ms, int duration_ms,
                             int* audio_samples_size, int16_t** audio_samples) {
  if (!g_is_audio_initialized) {
    TfLiteStatus init_status = InitAudioRecording();
    if (init_status != kTfLiteOk) {
      return init_status;
    }
    g_is_audio_initialized = true;
  }

  // Copy history samples
  memcpy((void*)(g_audio_output_buffer), 
         (void*)(g_history_buffer),
         history_samples_to_keep * sizeof(int16_t));

  // Read new samples from ring buffer
  int bytes_to_read = new_samples_to_get * sizeof(int16_t);
  int bytes_read = rb_read(g_audio_capture_buffer,
                          ((uint8_t*)(g_audio_output_buffer + history_samples_to_keep)),
                          bytes_to_read,
                          pdMS_TO_TICKS(200));

  if (bytes_read < 0) {
    ESP_LOGE(TAG, "Could not read from ring buffer");
    return kTfLiteError;
  } else if (bytes_read < bytes_to_read) {
    ESP_LOGD(TAG, "Partial read: %d/%d bytes", bytes_read, bytes_to_read);
    
    // Fill remaining with zeros instead of garbage
    memset((uint8_t*)(g_audio_output_buffer + history_samples_to_keep) + bytes_read,
           0,
           bytes_to_read - bytes_read);
  }

  // Update history buffer
  memcpy((void*)(g_history_buffer),
         (void*)(g_audio_output_buffer + new_samples_to_get),
         history_samples_to_keep * sizeof(int16_t));

  *audio_samples_size = kMaxAudioSampleSize;
  *audio_samples = g_audio_output_buffer;
  
  return kTfLiteOk;
}

int32_t LatestAudioTimestamp() { 
  return g_latest_audio_timestamp; 
}