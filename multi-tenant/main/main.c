#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include "kws/kws.h"
#include "vww/vww.h"

static const char *TAG = "multi_tenant";

static void start_kws_task(void) {
  xTaskCreatePinnedToCore(kws_task, "kws_task", 8192, NULL, 4, NULL, 0);
}

static void start_vww_task(void) {
  xTaskCreatePinnedToCore(vww_task, "vww_task", 12288, NULL, 5, NULL, 1);
}

void app_main(void) {
  ESP_LOGI(TAG, "Initializing KWS module");
  kws_init();
  start_kws_task();

  ESP_LOGI(TAG, "Initializing VWW module");
  esp_err_t vision_status = vww_init();
  if (vision_status == ESP_OK) {
    start_vww_task();
  } else {
    ESP_LOGE(TAG, "VWW initialization failed: 0x%x", vision_status);
  }
}
