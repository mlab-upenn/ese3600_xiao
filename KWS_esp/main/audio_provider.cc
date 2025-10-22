/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "audio_provider.h"

#include <cstdlib>
#include <cstring>

// FreeRTOS.h must be included before some of the following dependencies.
// Solves b/150260343.
// clang-format off
#include "freertos/FreeRTOS.h"
// clang-format on

#include "driver/i2s.h"
#include "driver/i2s_pdm.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "ringbuf.h"
#include "micro_model_settings.h"

using namespace std;

// for c2 and c3, I2S support was added from IDF v4.4 onwards
#define NO_I2S_SUPPORT CONFIG_IDF_TARGET_ESP32C2 || \
                          (CONFIG_IDF_TARGET_ESP32C3 \
                          && (ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 4, 0)))

static const char* TAG = "TF_LITE_AUDIO_PROVIDER";
/* ringbuffer to hold the incoming audio data */
ringbuf_t* g_audio_capture_buffer;
volatile int32_t g_latest_audio_timestamp = 0;
/* model requires 20ms new data from g_audio_capture_buffer and 10ms old data
 * each time , storing old data in the histrory buffer , {
 * history_samples_to_keep = 10 * 16 } */
constexpr int32_t history_samples_to_keep =
    ((kFeatureDurationMs - kFeatureStrideMs) *
     (kAudioSampleFrequency / 1000));
/* new samples to get each time from ringbuffer, { new_samples_to_get =  20 * 16
 * } */
constexpr int32_t new_samples_to_get =
    (kFeatureStrideMs * (kAudioSampleFrequency / 1000));

const int32_t kAudioCaptureBufferSize = 40000;
const int32_t pdm_bytes_to_read = 3200;

namespace {
int16_t g_audio_output_buffer[kMaxAudioSampleSize * 32];
bool g_is_audio_initialized = false;
int16_t g_history_buffer[history_samples_to_keep];

uint8_t g_pdm_read_buffer[pdm_bytes_to_read] = {};
// i2s_port_t i2s_port = I2S_NUM_0;
i2s_chan_handle_t g_rx_chan = nullptr;
gpio_num_t pdm_gpio_clk = GPIO_NUM_42;
gpio_num_t pdm_gpio_din = GPIO_NUM_41;
}  // namespace

static void pdm_init(void) {
  // 1) Create an RX channel
  if (g_rx_chan != nullptr) {
    return;
  }
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num  = 3;    // small, like Arduino defaults
  chan_cfg.dma_frame_num = 256;  // frames per DMA node
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &g_rx_chan));

  // 2) Configure PDM RX: 16 kHz, 16-bit, mono; pins: CLK=42, DIN=41
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
}


static void CaptureSamples(void* arg) {
  size_t bytes_read = pdm_bytes_to_read;
  pdm_init();
  while (1) {
    size_t bytes_read = 0;

    esp_err_t err = i2s_channel_read(g_rx_chan, g_pdm_read_buffer,
                                     pdm_bytes_to_read, &bytes_read,
                                     pdMS_TO_TICKS(50));
    if (err == ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "I2S read timed out");
      continue;
    } else if (err != ESP_OK) {
      ESP_LOGE(TAG, "I2S read failed: %s", esp_err_to_name(err));
      continue;
    }

    if (bytes_read == 0) {
      ESP_LOGW(TAG, "No I2S data available");
      continue;
    } else {
      if (bytes_read < pdm_bytes_to_read) {
        ESP_LOGW(TAG, "Partial I2S read");
      }
      /* write bytes read by i2s into ring buffer */
      int bytes_written = rb_write(g_audio_capture_buffer,
                                   (uint8_t*)g_pdm_read_buffer, bytes_read, pdMS_TO_TICKS(100));
      if (bytes_written != bytes_read) {
        ESP_LOGI(TAG, "Could only write %d bytes out of %d", bytes_written, bytes_read);
      }
      /* update the timestamp (in ms) to let the model know that new data has
       * arrived */
      g_latest_audio_timestamp = g_latest_audio_timestamp +
          ((1000 * (bytes_written / 2)) / kAudioSampleFrequency);
      if (bytes_written <= 0) {
        ESP_LOGE(TAG, "Could Not Write in Ring Buffer: %d ", bytes_written);
      } else if (bytes_written < bytes_read) {
        ESP_LOGW(TAG, "Partial Write");
      }
    }
  }
  vTaskDelete(NULL);
}

TfLiteStatus InitAudioRecording() {
  g_audio_capture_buffer = rb_init("tf_ringbuffer", kAudioCaptureBufferSize);
  if (!g_audio_capture_buffer) {
    ESP_LOGE(TAG, "Error creating ring buffer");
    return kTfLiteError;
  }
  /* create CaptureSamples Task which will get the i2s_data from mic and fill it
   * in the ring buffer */
  xTaskCreate(CaptureSamples, "CaptureSamples", 1024 * 4, NULL, 10, NULL);
  while (!g_latest_audio_timestamp) {
    vTaskDelay(1); // one tick delay to avoid watchdog
  }
  ESP_LOGI(TAG, "Audio Recording started");
  return kTfLiteOk;
}

TfLiteStatus GetAudioSamples1(int* audio_samples_size, int16_t** audio_samples)
{
  if (!g_is_audio_initialized) {
    TfLiteStatus init_status = InitAudioRecording();
    if (init_status != kTfLiteOk) {
      return init_status;
    }
    g_is_audio_initialized = true;
  }
  int bytes_read =
    rb_read(g_audio_capture_buffer, (uint8_t*)(g_audio_output_buffer), 16000, 1000);
  if (bytes_read < 0) {
    ESP_LOGI(TAG, "Couldn't read data in time");
    bytes_read = 0;
  }
  *audio_samples_size = bytes_read;
  *audio_samples = g_audio_output_buffer;
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
  /* copy 160 samples (320 bytes) into output_buff from history */
  memcpy((void*)(g_audio_output_buffer), (void*)(g_history_buffer),
         history_samples_to_keep * sizeof(int16_t));

  /* copy 320 samples (640 bytes) from rb at ( int16_t*(g_audio_output_buffer) +
   * 160 ), first 160 samples (320 bytes) will be from history */
  int bytes_read =
      rb_read(g_audio_capture_buffer,
              ((uint8_t*)(g_audio_output_buffer + history_samples_to_keep)),
              new_samples_to_get * sizeof(int16_t), pdMS_TO_TICKS(200));
  if (bytes_read < 0) {
    ESP_LOGE(TAG, " Model Could not read data from Ring Buffer");
  } else if (bytes_read < new_samples_to_get * sizeof(int16_t)) {
    ESP_LOGD(TAG, "RB FILLED RIGHT NOW IS %d",
             rb_filled(g_audio_capture_buffer));
    ESP_LOGD(TAG, " Partial Read of Data by Model ");
    ESP_LOGV(TAG, " Could only read %d bytes when required %d bytes ",
             bytes_read, (int) (new_samples_to_get * sizeof(int16_t)));
  }

  /* copy 320 bytes from output_buff into history */
  memcpy((void*)(g_history_buffer),
         (void*)(g_audio_output_buffer + new_samples_to_get),
         history_samples_to_keep * sizeof(int16_t));

  *audio_samples_size = kMaxAudioSampleSize;
  *audio_samples = g_audio_output_buffer;
  return kTfLiteOk;
}

int32_t LatestAudioTimestamp() { return g_latest_audio_timestamp; }
