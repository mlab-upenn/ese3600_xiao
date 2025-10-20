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

#include "output_handler.h"
#include "tensorflow/lite/micro/micro_log.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

namespace {
constexpr gpio_num_t kLedPin = GPIO_NUM_21;  // Update if your LED uses a different pin.
constexpr ledc_mode_t kLedcMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kLedcTimer = LEDC_TIMER_0;
constexpr ledc_channel_t kLedcChannel = LEDC_CHANNEL_0;
constexpr ledc_timer_bit_t kLedcResolution = LEDC_TIMER_13_BIT;
constexpr uint32_t kLedcFrequencyHz = 1000;
constexpr uint32_t kMaxDuty =
    (1u << static_cast<uint32_t>(kLedcResolution)) - 1u;

bool InitializeLedPwm() {
  ledc_timer_config_t timer_config = {
      .speed_mode = kLedcMode,
      .duty_resolution = kLedcResolution,
      .timer_num = kLedcTimer,
      .freq_hz = kLedcFrequencyHz,
      .clk_cfg = LEDC_AUTO_CLK,
  };
  if (ledc_timer_config(&timer_config) != ESP_OK) {
    MicroPrintf("LEDC timer config failed");
    return false;
  }

  ledc_channel_config_t channel_config = {};
  channel_config.gpio_num = kLedPin;
  channel_config.speed_mode = kLedcMode;
  channel_config.channel = kLedcChannel;
  channel_config.intr_type = LEDC_INTR_DISABLE;
  channel_config.timer_sel = kLedcTimer;
  channel_config.duty = 0;
  channel_config.hpoint = 0;
  if (ledc_channel_config(&channel_config) != ESP_OK) {
    MicroPrintf("LEDC channel config failed");
    return false;
  }

  return true;
}

bool EnsureLedInitialized() {
  static bool initialized = false;
  static bool attempted = false;
  if (!attempted) {
    attempted = true;
    initialized = InitializeLedPwm();
  }
  return initialized;
}
}  // namespace

void HandleOutput(float x_value, float y_value) {
  if (!EnsureLedInitialized()) {
    return;
  }

  float normalized = (y_value + 1.0f) * 0.5f;
  if (normalized < 0.0f) {
    normalized = 0.0f;
  } else if (normalized > 1.0f) {
    normalized = 1.0f;
  }
  uint32_t duty = static_cast<uint32_t>(normalized * static_cast<float>(kMaxDuty));

  if (ledc_set_duty(kLedcMode, kLedcChannel, duty) == ESP_OK) {
    ledc_update_duty(kLedcMode, kLedcChannel);
  }

  MicroPrintf("x_value: %f, y_value: %f, duty: %u", static_cast<double>(x_value),
              static_cast<double>(y_value), duty);
}
