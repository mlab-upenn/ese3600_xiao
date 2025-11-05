#include "kws/kws.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tensorflow/lite/micro/system_setup.h"

#include "kws/main_functions.h"

static const char *TAG = "kws_runner";

extern "C" void kws_init(void) {
  ESP_LOGI(TAG, "Initializing keyword spotting pipeline");
  tflite::InitializeTarget();
  kws_setup();
}

extern "C" void kws_task(void *param) {
  (void)param;
  ESP_LOGI(TAG, "Keyword spotting task started");
  while (true) {
    kws_run_once();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
