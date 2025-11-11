#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include <cstdint>
#include "driver/gpio.h"

// I2C pin assignments for the external ICM20600.
constexpr gpio_num_t kI2C_SDA = GPIO_NUM_5;
constexpr gpio_num_t kI2C_SCL = GPIO_NUM_6;
constexpr uint32_t kI2C_FREQ = 400000;  // 400 kHz fast mode

// ICM20600 device address when AD0 is pulled high.
constexpr uint8_t kIMU_Address = 0x69;

// Threshold for motion detection helper.
constexpr float kAccelThreshold = 0.15f;

#endif  // CONSTANTS_H_
