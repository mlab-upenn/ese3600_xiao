#ifndef IMU_PROVIDER_H_
#define IMU_PROVIDER_H_

#include <cstdint>
#include "driver/i2c.h"

// ICM-20600 Register Map
namespace ICM20600_Regs {
    constexpr uint8_t PWR_MGMT_1 = 0x6B;
    constexpr uint8_t PWR_MGMT_2 = 0x6C;
    constexpr uint8_t ACCEL_CONFIG = 0x1C;
    constexpr uint8_t ACCEL_CONFIG2 = 0x1D;
    constexpr uint8_t GYRO_CONFIG = 0x1B;
    constexpr uint8_t WHO_AM_I = 0x75;
    
    constexpr uint8_t ACCEL_XOUT_H = 0x3B;
    constexpr uint8_t ACCEL_XOUT_L = 0x3C;
    constexpr uint8_t ACCEL_YOUT_H = 0x3D;
    constexpr uint8_t ACCEL_YOUT_L = 0x3E;
    constexpr uint8_t ACCEL_ZOUT_H = 0x3F;
    constexpr uint8_t ACCEL_ZOUT_L = 0x40;
    
    constexpr uint8_t GYRO_XOUT_H = 0x43;
    constexpr uint8_t GYRO_XOUT_L = 0x44;
    constexpr uint8_t GYRO_YOUT_H = 0x45;
    constexpr uint8_t GYRO_YOUT_L = 0x46;
    constexpr uint8_t GYRO_ZOUT_H = 0x47;
    constexpr uint8_t GYRO_ZOUT_L = 0x48;
}

// Initialize the IMU
bool SetupIMU();

// Read accelerometer and gyroscope data
// accel_data: pointer to array of 3 floats for x, y, z acceleration (in g's)
// gyro_data: pointer to array of 3 floats for x, y, z gyroscope (in deg/s)
// Returns true if successful
bool ReadIMU(float* accel_data, float* gyro_data);

// Check if significant movement detected (for gesture start)
bool IsMovementDetected(const float* accel_data);

#endif  // IMU_PROVIDER_H_
