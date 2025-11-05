#include "esp_attr.h"
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

void esp32_camera_component_placeholder(void) {}

// The prebuilt esp32-camera binary expects this FreeRTOS helper, which no longer
// exists in newer ESP-IDF releases. Provide a thin shim that yields from ISR.
void IRAM_ATTR vPortEvaluateYieldFromISR(void)
{
    portYIELD_FROM_ISR();
}
