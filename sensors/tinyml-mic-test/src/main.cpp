#include <Arduino.h>
#include <I2S.h> // for the ESP32-S3 mic

void setup() {
  Serial.begin(115200);
  delay(200);                    

  // XIAO ESP32S3 Sense PDM mic pins: BCLK=2, WS=42, DIN=41
  I2S.setAllPins(2, 42, 41, -1, -1);

  // Start I2S in PDM mono mode at 16 kHz, 16-bit samples
  // Returns false if the peripheral cannot be configured (wrong pins, etc.)
  if (!I2S.begin(PDM_MONO_MODE, 16000, 16)) {
    Serial.println("Failed to initialize I2S!");
    while (1) {}
  }
}

void loop() {
  int sample = I2S.read(); 
  if (sample != -1 && sample != 1) {
    Serial.print(">mic:");          // Teleplot variable name
    Serial.println(sample);         // newline-terminated
  }
}
