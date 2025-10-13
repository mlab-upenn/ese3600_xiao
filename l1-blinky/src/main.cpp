/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>

void setup()
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);

  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.printf("HIGH!\n");

  // wait for a second
  delay(1000);
  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);
  Serial.printf("LOW!\n");

   // wait for a second
  delay(1000);
}
