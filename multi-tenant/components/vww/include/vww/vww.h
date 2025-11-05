#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t vww_init(void);
void vww_task(void *param);

#ifdef __cplusplus
}
#endif

