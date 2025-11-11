#include <Arduino.h>

#include "tensorflow/lite/micro/debug_log.h"
#include "tensorflow/lite/micro/system_setup.h"

namespace {
constexpr unsigned long kSerialInitTimeoutMs = 2000;
constexpr uint32_t kSerialBaudRate = 115200;
}  // namespace

extern "C" void DebugLog(const char* s) {
  Serial.print(s);
}

namespace tflite {

void InitializeTarget() {
  static bool initialized = false;
  if (initialized) {
    return;
  }
  initialized = true;

  Serial.begin(kSerialBaudRate);
  unsigned long start = millis();
  while (!Serial && (millis() - start < kSerialInitTimeoutMs)) {
    delay(10);
  }
}

}  // namespace tflite
