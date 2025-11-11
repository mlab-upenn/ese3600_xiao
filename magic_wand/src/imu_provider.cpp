#include "imu_provider.h"
#include "constants.h"
#include "esp_log.h"
#include <cmath>

static const char* TAG = "IMU_Provider";

static i2c_port_t i2c_port = I2C_NUM_0;

// I2C helper functions
static esp_err_t i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

bool SetupIMU() {
    ESP_LOGI(TAG, "Initializing I2C for IMU...");
    
    // Configure I2C
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = kI2C_SDA;
    conf.scl_io_num = kI2C_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = kI2C_FREQ;
    conf.clk_flags = 0;
    
    esp_err_t ret = i2c_param_config(i2c_port, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = i2c_driver_install(i2c_port, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for IMU to power up
    
    // Check WHO_AM_I register
    uint8_t who_am_i = 0;
    ret = i2c_read_bytes(kIMU_Address, ICM20600_Regs::WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I: %s", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI(TAG, "WHO_AM_I: 0x%02X (expected 0x11)", who_am_i);
    
    // Reset the device
    ret = i2c_write_byte(kIMU_Address, ICM20600_Regs::PWR_MGMT_1, 0x80);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset IMU");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Wake up the device (clear sleep bit)
    ret = i2c_write_byte(kIMU_Address, ICM20600_Regs::PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake IMU");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Configure accelerometer: ±4g range
    ret = i2c_write_byte(kIMU_Address, ICM20600_Regs::ACCEL_CONFIG, 0x08);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer");
        return false;
    }
    
    // Configure gyroscope: ±500 deg/s range
    ret = i2c_write_byte(kIMU_Address, ICM20600_Regs::GYRO_CONFIG, 0x08);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope");
        return false;
    }
    
    // Enable accelerometer and gyroscope
    ret = i2c_write_byte(kIMU_Address, ICM20600_Regs::PWR_MGMT_2, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable sensors");
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "IMU initialized successfully!");
    return true;
}

bool ReadIMU(float* accel_data, float* gyro_data) {
    uint8_t raw_data[14];
    
    // Read all sensor data at once (burst read)
    esp_err_t ret = i2c_read_bytes(kIMU_Address, ICM20600_Regs::ACCEL_XOUT_H, raw_data, 14);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read IMU data");
        return false;
    }
    
    // Parse accelerometer data (±4g range)
    int16_t accel_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    int16_t accel_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    int16_t accel_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
    
    // Skip temperature data (raw_data[6], raw_data[7])
    
    // Parse gyroscope data (±500 deg/s range)
    int16_t gyro_x = (int16_t)((raw_data[8] << 8) | raw_data[9]);
    int16_t gyro_y = (int16_t)((raw_data[10] << 8) | raw_data[11]);
    int16_t gyro_z = (int16_t)((raw_data[12] << 8) | raw_data[13]);
    
    // Convert to physical units
    // Accelerometer: ±4g range -> 8192 LSB/g
    constexpr float accel_scale = 4.0f / 32768.0f;
    accel_data[0] = accel_x * accel_scale;
    accel_data[1] = accel_y * accel_scale;
    accel_data[2] = accel_z * accel_scale;
    
    // Gyroscope: ±500 deg/s range -> 65.5 LSB/(deg/s)
    constexpr float gyro_scale = 500.0f / 32768.0f;
    gyro_data[0] = gyro_x * gyro_scale;
    gyro_data[1] = gyro_y * gyro_scale;
    gyro_data[2] = gyro_z * gyro_scale;
    
    return true;
}

bool IsMovementDetected(const float* accel_data) {
    // Calculate magnitude of acceleration vector
    float magnitude = sqrtf(accel_data[0] * accel_data[0] + 
                           accel_data[1] * accel_data[1] + 
                           accel_data[2] * accel_data[2]);
    
    // Detect if magnitude deviates significantly from 1g (gravity)
    float deviation = fabsf(magnitude - 1.0f);
    
    return deviation > kAccelThreshold;
}
