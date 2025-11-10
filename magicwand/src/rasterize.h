#ifndef RASTERIZE_H_
#define RASTERIZE_H_

#include <cstdint>

// Structure to hold a 2D point for the stroke
struct Point2D {
    float x;
    float y;
};

// Convert accelerometer/gyroscope data to 2D stroke points
// imu_data: array of [ax, ay, az, gx, gy, gz] for each sample
// num_samples: number of IMU samples
// stroke_points: output array of 2D points (should be pre-allocated)
// Returns the number of stroke points generated
int ConvertIMUToStroke(const float* imu_data, int num_samples, Point2D* stroke_points);

// Rasterize stroke points to a grayscale or RGB image
// stroke_points: array of 2D points representing the gesture
// num_points: number of points in the stroke
// image: output image buffer (32x32x3 for RGB, will be filled with values 0-255)
void RasterizeStroke(const Point2D* stroke_points, int num_points, uint8_t* image);

// Normalize stroke points to fit within the image bounds
void NormalizeStroke(Point2D* stroke_points, int num_points);

#endif  // RASTERIZE_H_
