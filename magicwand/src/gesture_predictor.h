#ifndef GESTURE_PREDICTOR_H_
#define GESTURE_PREDICTOR_H_

#include <cstdint>

// Initialize the TensorFlow Lite model and interpreter
bool SetupGesturePredictor();

// Run inference on the rasterized image
// image_data: 32x32x3 RGB image (values 0-255)
// gesture_index: output - index of predicted gesture
// confidence: output - confidence score (-128 to 127)
// Returns true if successful
bool PredictGesture(const uint8_t* image_data, int* gesture_index, int8_t* confidence);

#endif  // GESTURE_PREDICTOR_H_
