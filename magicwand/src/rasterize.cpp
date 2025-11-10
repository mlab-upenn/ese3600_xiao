#include "rasterize.h"
#include "constants.h"
#include "esp_log.h"
#include <cmath>
#include <cstring>
#include <algorithm>

static const char* TAG = "Rasterize";

int ConvertIMUToStroke(const float* imu_data, int num_samples, Point2D* stroke_points) {
    // Convert IMU data to 2D stroke using velocity integration just like what we used in the colab
    
    float x = 0.0f;
    float y = 0.0f;
    float vx = 0.0f;
    float vy = 0.0f;
    
    int point_count = 0;
    
    for (int i = 0; i < num_samples; i++) {
        // Each sample has 6 values: ax, ay, az, gx, gy, gz
        const float* sample = &imu_data[i * 6];
        float ax = sample[0];
        float ay = sample[1];
        
        // Integrate acceleration to get velocity (simple Euler integration)
        // dt = 1/25 = 0.04 seconds per sample
        constexpr float dt = 1.0f / kSamplingFreq;
        vx += ax * dt;
        vy += ay * dt;
        
        // Integrate velocity to get position
        x += vx * dt;
        y += vy * dt;
        
        // Store the point
        stroke_points[point_count].x = x;
        stroke_points[point_count].y = y;
        point_count++;
    }
    
    return point_count;
}

void NormalizeStroke(Point2D* stroke_points, int num_points) {
    if (num_points == 0) return;
    
    // Find min and max values
    float min_x = stroke_points[0].x;
    float max_x = stroke_points[0].x;
    float min_y = stroke_points[0].y;
    float max_y = stroke_points[0].y;
    
    for (int i = 1; i < num_points; i++) {
        min_x = std::min(min_x, stroke_points[i].x);
        max_x = std::max(max_x, stroke_points[i].x);
        min_y = std::min(min_y, stroke_points[i].y);
        max_y = std::max(max_y, stroke_points[i].y);
    }
    
    // Calculate center
    float center_x = (min_x + max_x) / 2.0f;
    float center_y = (min_y + max_y) / 2.0f;
    
    // Calculate scale to fit within normalization range
    float range_x = max_x - min_x;
    float range_y = max_y - min_y;
    float max_range = std::max(range_x, range_y);
    
    float scale = 1.0f;
    if (max_range > 0) {
        scale = (2.0f * kNormalizationRange) / max_range;
    }
    
    // Normalize: center at origin and scale to ±0.6 range
    for (int i = 0; i < num_points; i++) {
        stroke_points[i].x = (stroke_points[i].x - center_x) * scale;
        stroke_points[i].y = (stroke_points[i].y - center_y) * scale;
        
        // Clamp to range
        stroke_points[i].x = std::max(-kNormalizationRange, 
                                      std::min(kNormalizationRange, stroke_points[i].x));
        stroke_points[i].y = std::max(-kNormalizationRange, 
                                      std::min(kNormalizationRange, stroke_points[i].y));
    }
}

// Helper function to draw a line between two points
static void DrawLine(uint8_t* image, int x0, int y0, int x1, int y1) {
    // Bresenham's line algorithm
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    while (true) {
        // Draw pixel with stroke width
        for (int wy = -kStrokeWidth/2; wy <= kStrokeWidth/2; wy++) {
            for (int wx = -kStrokeWidth/2; wx <= kStrokeWidth/2; wx++) {
                int px = x0 + wx;
                int py = y0 + wy;
                
                if (px >= 0 && px < kImageWidth && py >= 0 && py < kImageHeight) {
                    int idx = (py * kImageWidth + px) * 3;
                    image[idx + 0] = kStrokeColor[0];  // R
                    image[idx + 1] = kStrokeColor[1];  // G
                    image[idx + 2] = kStrokeColor[2];  // B
                }
            }
        }
        
        if (x0 == x1 && y0 == y1) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void RasterizeStroke(const Point2D* stroke_points, int num_points, uint8_t* image) {
    // Initialize image with white background
    for (int i = 0; i < kImageWidth * kImageHeight * 3; i += 3) {
        image[i + 0] = kBackgroundColor[0];
        image[i + 1] = kBackgroundColor[1];
        image[i + 2] = kBackgroundColor[2];
    }
    
    if (num_points < 2) {
        ESP_LOGW(TAG, "Not enough points to rasterize");
        return;
    }
    
    // Convert normalized coordinates (-0.6 to 0.6) to pixel coordinates (0 to 31)
    // Map [-0.6, 0.6] -> [0, 31]
    auto coordToPixel = [](float coord) -> int {
        float normalized = (coord + kNormalizationRange) / (2.0f * kNormalizationRange);
        int pixel = static_cast<int>(normalized * (kImageWidth - 1));
        return std::max(0, std::min(kImageWidth - 1, pixel));
    };
    
    // Draw lines connecting consecutive points
    for (int i = 0; i < num_points - 1; i++) {
        int x0 = coordToPixel(stroke_points[i].x);
        int y0 = coordToPixel(stroke_points[i].y);
        int x1 = coordToPixel(stroke_points[i + 1].x);
        int y1 = coordToPixel(stroke_points[i + 1].y);
        
        DrawLine(image, x0, y0, x1, y1);
    }
    
    ESP_LOGI(TAG, "Rasterized %d points to %dx%d image", num_points, kImageWidth, kImageHeight);
}
