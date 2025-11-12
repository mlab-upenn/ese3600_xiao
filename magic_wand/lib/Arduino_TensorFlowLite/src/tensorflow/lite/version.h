#ifndef TENSORFLOW_LITE_VERSION_H_
#define TENSORFLOW_LITE_VERSION_H_

#include "tensorflow/lite/schema/schema_generated.h"

namespace tflite {

inline int32_t TfLiteVersionNumber() { return TFLITE_SCHEMA_VERSION; }

}  // namespace tflite

#endif  // TENSORFLOW_LITE_VERSION_H_
