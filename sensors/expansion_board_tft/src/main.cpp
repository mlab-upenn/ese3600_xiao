#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

// Use the I2C-based constructor for 128x64 SSD1306 OLED
// Here we let U8g2 use the default Wire (I2C). If needed you could call Wire1 for second I2C.
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void setup() {
  // If using second I2C interface on XIAO expansion (D4/D5), you may need:
  // Wire1.begin();      // On some XIAO variants  
  Wire.begin();
  u8g2.begin();
}

void loop() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.drawStr(0, 24, "Hello, world!");
  u8g2.sendBuffer();

  delay(2000);
}
