#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "constants.h"
#include "imu_provider.h"
#include "gesture_predictor.h"
#include "output_handler.h"
#include "rasterize.h"

static const char* TAG = "MagicWand";

// Buffers for gesture capture and processing
static float imu_buffer[kGestureSampleCount * 6];  // 6 values per sample (ax, ay, az, gx, gy, gz)
static Point2D stroke_points[kGestureSampleCount];
static uint8_t image_buffer[kImageSize];

// State machine for gesture detection
enum State {
    IDLE,
    CAPTURING,
    PROCESSING
};

static State current_state = IDLE;
static int sample_count = 0;
static int64_t last_detection_time = 0;

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Magic Wand Gesture Recognition");
    ESP_LOGI(TAG, "ESP32-S3 with Grove 9DOF IMU (ICM-20600)");
    ESP_LOGI(TAG, "================================");
    
    // Initialize hardware
    ESP_LOGI(TAG, "Initializing IMU...");
    if (!SetupIMU()) {
        ESP_LOGE(TAG, "Failed to initialize IMU!");
        ESP_LOGE(TAG, "Please check your connections:");
        ESP_LOGE(TAG, "  - SDA: GPIO5");
        ESP_LOGE(TAG, "  - SCL: GPIO6");
        ESP_LOGE(TAG, "  - IMU Address: 0x%02X", kIMU_Address);
        return;
    }
    
    // Initialize TensorFlow Lite
    ESP_LOGI(TAG, "Initializing TensorFlow Lite model...");
    if (!SetupGesturePredictor()) {
        ESP_LOGE(TAG, "Failed to initialize gesture predictor!");
        ESP_LOGE(TAG, "Make sure you have replaced model_data.cc with your trained model.");
        return;
    }
    
    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "Ready! Start performing gestures...");
    ESP_LOGI(TAG, "Move the board and hold still to trigger capture.");
    ESP_LOGI(TAG, "================================\n");
    
    float accel_data[3];
    float gyro_data[3];
    
    while (true) {
        // Read IMU data
        if (!ReadIMU(accel_data, gyro_data)) {
            ESP_LOGW(TAG, "Failed to read IMU data");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        switch (current_state) {
            case IDLE: {
                // Wait for movement to start gesture capture
                if (IsMovementDetected(accel_data)) {
                    // Check cooldown period
                    int64_t current_time = esp_timer_get_time() / 1000;  // Convert to ms
                    if (current_time - last_detection_time > kDetectionCooldownMs) {
                        ESP_LOGI(TAG, "Movement detected! Starting capture...");
                        current_state = CAPTURING;
                        sample_count = 0;
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(10));
                break;
            }
            
            case CAPTURING: {
                // Store IMU data
                int idx = sample_count * 6;
                imu_buffer[idx + 0] = accel_data[0];
                imu_buffer[idx + 1] = accel_data[1];
                imu_buffer[idx + 2] = accel_data[2];
                imu_buffer[idx + 3] = gyro_data[0];
                imu_buffer[idx + 4] = gyro_data[1];
                imu_buffer[idx + 5] = gyro_data[2];
                                
                // Print raw gyro data in the expected format
                printf(">gx,gyro:%.2f\n", gyro_data[0]);
                printf(">gy,gyro:%.2f\n", gyro_data[1]);
                printf(">gz,gyro:%.2f\n", gyro_data[2]);

                sample_count++;

                // Print progress
                if (sample_count % 25 == 0) {
                    ESP_LOGI(TAG, "Capturing... %d/%d samples", sample_count, kGestureSampleCount);
                }
                
                if (sample_count >= kGestureSampleCount) {
                    ESP_LOGI(TAG, "Capture complete! Processing gesture...");
                    current_state = PROCESSING;
                }
                
                vTaskDelay(pdMS_TO_TICKS(kGestureSampleDelayMs));
                break;
            }
            
            case PROCESSING: {
                // Convert IMU data to 2D stroke
                int num_points = ConvertIMUToStroke(imu_buffer, kGestureSampleCount, stroke_points);
                
                // Normalize stroke to fit in the coordinate range
                NormalizeStroke(stroke_points, num_points);
                
                // Rasterize stroke to image
                RasterizeStroke(stroke_points, num_points, image_buffer);
                
                // Run inference
                int gesture_index = 0;
                int8_t confidence = 0;
                
                if (PredictGesture(image_buffer, &gesture_index, &confidence)) {
                    // Check if confidence meets threshold
                    if (confidence >= kConfidenceThreshold) {
                        // Display result with ASCII art
                        HandleOutput(image_buffer, kGestureLabels[gesture_index], confidence);
                    } else {
                        ESP_LOGW(TAG, "Low confidence: %d (threshold: %d)", 
                                confidence, kConfidenceThreshold);
                        printf("\n[Low confidence detection - try again]\n\n");
                    }
                } else {
                    ESP_LOGE(TAG, "Prediction failed!");
                }
                
                // Update last detection time
                last_detection_time = esp_timer_get_time() / 1000;
                
                // Return to idle state
                current_state = IDLE;
                ESP_LOGI(TAG, "Ready for next gesture...\n");
                
                break;
            }
        }
    }
}
