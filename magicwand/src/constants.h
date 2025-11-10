#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include <cstdint>

constexpr int kGestureCount = 15;  // UPDATE: Number of gestures in your dataset

extern const char* kGestureLabels[kGestureCount];

constexpr int kImageWidth = 32;
constexpr int kImageHeight = 32;
constexpr int kImageChannels = 3;  // RGB image
constexpr int kImageSize = kImageWidth * kImageHeight * kImageChannels;

// Model expects 32x32x3 input (RGB image)
constexpr int kInputTensorSize = kImageWidth * kImageHeight * kImageChannels;

// I2C pins for XIAO ESP32-S3 (using the Grove connector)
constexpr int kI2C_SDA = 5;  // GPIO5 - SDA
constexpr int kI2C_SCL = 6;  // GPIO6 - SCL
constexpr int kI2C_FREQ = 400000;  // 400kHz

// ICM-20600 I2C address
constexpr uint8_t kIMU_Address = 0x68; 

// ============================================================================
// GESTURE CAPTURE CONFIGURATION
// ============================================================================
// Accelerometer range for gesture detection (in g's)
constexpr float kAccelThreshold = 1.5f;  // Movement threshold to start recording

// Number of samples to capture per gesture
constexpr int kGestureSampleCount = 119;  // Should match training data length

// Sampling rate (Hz)
constexpr int kSamplingFreq = 25;  // 25 Hz = 40ms per sample

// Gesture timeout (ms) - time between samples
constexpr int kGestureSampleDelayMs = 1000 / kSamplingFreq;  // 40ms

// Normalization ranges (should match training data)
constexpr float kNormalizationRange = 0.6f;  // ±0.6 range for X and Y

// ============================================================================
// RASTERIZATION CONFIGURATION
// ============================================================================
// Stroke width for drawing gestures
constexpr int kStrokeWidth = 2;

// Background and foreground colors for rasterization
constexpr uint8_t kBackgroundColor[3] = {255, 255, 255};  // White background
constexpr uint8_t kStrokeColor[3] = {0, 0, 0};  // Black stroke

// ============================================================================
// OUTPUT CONFIGURATION
// ============================================================================
// Confidence score threshold (model outputs int8 values from -128 to 127)
constexpr int8_t kConfidenceThreshold = 60;  // Minimum confidence for valid prediction

// ASCII art size for gesture visualization
constexpr int kAsciiWidth = 32;
constexpr int kAsciiHeight = 32;

// Detection cooldown (ms) to prevent multiple detections of same gesture
constexpr int kDetectionCooldownMs = 1000;  // 1 second

#endif  // CONSTANTS_H_
