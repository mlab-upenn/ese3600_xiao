#ifndef OUTPUT_HANDLER_H_
#define OUTPUT_HANDLER_H_

#include <cstdint>

// Display the prediction result with ASCII art
// image: 32x32x3 RGB image to visualize
// gesture_label: name of the predicted gesture
// confidence: confidence score (-128 to 127)
void HandleOutput(const uint8_t* image, const char* gesture_label, int8_t confidence);

#endif  // OUTPUT_HANDLER_H_
