#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the keyword spotting pipeline.
 *
 * Must be called before starting the processing task.
 */
void kws_init(void);

/**
 * @brief Task entry point that continuously runs keyword inference.
 */
void kws_task(void *param);

#ifdef __cplusplus
}
#endif

