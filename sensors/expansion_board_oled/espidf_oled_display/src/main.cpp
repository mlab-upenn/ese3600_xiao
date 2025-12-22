#include "oled_display.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" void app_main() {
    const char* TAG = "MAIN";

    ESP_LOGI(TAG, "Initializing OLED...");
    esp_err_t ret = oled_display_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OLED init failed: %s", esp_err_to_name(ret));
        return;
    }

    oled_display_clear();
    oled_display_text(0, 0, "Hello,");
    oled_display_text(0, 8, "World!");
    oled_display_update();
    ESP_LOGI(TAG, "Displayed Hello World on OLED");

    // Keep the task alive so the display remains on
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
