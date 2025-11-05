#include "vww/vww.h"

#include <cstdint>
#include <cstring>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"

#include "vww/NeuralNetwork.h"
#define CAMERA_MODEL_XIAO_ESP32S3
#include "vww/camera_pins.h"
#include "tensorflow/lite/c/common.h"

namespace {
constexpr int kInputWidth = 96;
constexpr int kInputHeight = 96;
constexpr gpio_num_t kLedPin = GPIO_NUM_21;

NeuralNetwork *s_network = nullptr;
const char *TAG = "vww_runner";

uint32_t Rgb565ToRgb888(uint16_t color) {
  const uint8_t lb = (color >> 8) & 0xFF;
  const uint8_t hb = color & 0xFF;

  const uint32_t r = (lb & 0x1F) << 3;
  const uint32_t g = ((hb & 0x07) << 5) | ((lb & 0xE0) >> 3);
  const uint32_t b = hb & 0xF8;

  return (r << 16) | (g << 8) | b;
}

esp_err_t FillInputTensor(camera_fb_t *fb, TfLiteTensor *input) {
  if (fb->format != PIXFORMAT_RGB565) {
    ESP_LOGE(TAG, "Unexpected frame format %d", fb->format);
    return ESP_ERR_INVALID_STATE;
  }

  int post = 0;
  const int startx = (fb->width - kInputWidth) / 2;
  const int starty = (fb->height - kInputHeight);
  float *image_data = input->data.f;

  for (int y = 0; y < kInputHeight; ++y) {
    for (int x = 0; x < kInputWidth; ++x) {
      const int getPos = (starty + y) * fb->width + startx + x;
      const uint16_t color = reinterpret_cast<uint16_t *>(fb->buf)[getPos];
      const uint32_t rgb = Rgb565ToRgb888(color);

      image_data[post * 3 + 0] = (rgb >> 16) & 0xFF;
      image_data[post * 3 + 1] = (rgb >> 8) & 0xFF;
      image_data[post * 3 + 2] = rgb & 0xFF;
      ++post;
    }
  }
  return ESP_OK;
}

camera_config_t CreateCameraConfig() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_96X96;
  config.pixel_format = PIXFORMAT_RGB565;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  return config;
}

void ConfigureLed() {
  gpio_config_t cfg = {};
  cfg.mode = GPIO_MODE_OUTPUT;
  cfg.pin_bit_mask = 1ULL << kLedPin;
  gpio_config(&cfg);
  gpio_set_level(kLedPin, 1);
}

void ProcessFrame() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    ESP_LOGW(TAG, "Camera capture failed");
    return;
  }

  const uint64_t start_prep = esp_timer_get_time();
  if (FillInputTensor(fb, s_network->getInput()) != ESP_OK) {
    esp_camera_fb_return(fb);
    return;
  }
  const uint64_t dur_prep = esp_timer_get_time() - start_prep;

  const uint64_t start_infer = esp_timer_get_time();
  if (s_network->predict() != kTfLiteOk) {
    ESP_LOGE(TAG, "Inference failed");
    esp_camera_fb_return(fb);
    return;
  }
  const uint64_t dur_infer = esp_timer_get_time() - start_infer;

  esp_camera_fb_return(fb);

  const float prob = s_network->getOutput()->data.f[0];
  ESP_LOGI(TAG, "Prep: %llums, Infer: %llums, prob=%.3f",
           dur_prep / 1000ULL, dur_infer / 1000ULL, prob);

  const bool person_detected = prob >= 0.5f;
  gpio_set_level(kLedPin, person_detected ? 0 : 1);
}
}  // namespace

extern "C" esp_err_t vww_init(void) {
  if (s_network != nullptr) {
    return ESP_OK;
  }

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  ConfigureLed();

  camera_config_t config = CreateCameraConfig();
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
    return err;
  }

  ESP_LOGI(TAG, "Camera initialized: frame_size=%d pixel_format=%d",
           config.frame_size, config.pixel_format);

  s_network = new NeuralNetwork();
  if (s_network == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate neural network");
    return ESP_ERR_NO_MEM;
  }

  if (s_network->getInput() == nullptr || s_network->getOutput() == nullptr) {
    ESP_LOGE(TAG, "Neural network tensors unavailable");
    return ESP_FAIL;
  }

  return ESP_OK;
}

extern "C" void vww_task(void *param) {
  (void)param;
  ESP_LOGI(TAG, "VWW task started");
  while (true) {
    if (s_network != nullptr) {
      ProcessFrame();
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
