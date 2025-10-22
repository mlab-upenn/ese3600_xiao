#include <Arduino.h>
#include <Wire.h>

/*
  Grove 9-DoF (ICM20600 + AK09918) on XIAO ESP32S3 Sense
  This version streams ONLY ICM20600 accelerometer & gyroscope data, as that is all we need for the Assignments for this course. 

  Usage:
    1) Open Serial Monitor/Plotter at 115200 baud.
    2) Type 'a' to stream accelerometer (g), 'g' for gyro (deg/s), 's' to stop.
*/

// ICM20600 I2C address (Mostly Seeed boards use 0x69.)
#define ICM20600_ADDR 0x69

// ICM20600 Register addresses
#define ICM20600_PWR_MGMT_1   0x6B  // Power management (reset, sleep, clock)
#define ICM20600_ACCEL_XOUT_H 0x3B  // Accel data start register (X high byte)
#define ICM20600_GYRO_XOUT_H  0x43  // Gyro data start register (X high byte)

char currentMode = '\0';   // 'a' for accel, 'g' for gyro
bool streaming   = false;  // when true, prints samples in loop()

void streamAccelerometer();
void streamGyroscope();

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("Initializing ICM20600 (Accel/Gyro)...");

  Wire.begin();
  Wire.setClock(400000); // 400 kHz I2C
  delay(100);

  // Write 0x80 to PWR_MGMT_1 to issue a device reset.
  Wire.beginTransmission(ICM20600_ADDR);
  Wire.write(ICM20600_PWR_MGMT_1);
  Wire.write(0x80); // Device reset
  Wire.endTransmission();
  delay(100);

  // Write 0x01 to PWR_MGMT_1: auto clock select, exit sleep.
  Wire.beginTransmission(ICM20600_ADDR);
  Wire.write(ICM20600_PWR_MGMT_1);
  Wire.write(0x01); // Auto select clock, wake up
  Wire.endTransmission();
  delay(100);

  Serial.println("✓ ICM20600 initialized");

  Serial.println();
  Serial.println("Commands: a=accel, g=gyro, s=stop");
}

void loop() {
  // Handle single-character serial commands
  if (Serial.available() > 0) {
    char command = Serial.read();
    while (Serial.available() > 0) Serial.read(); // flush any extra chars

    if (command == 'a' || command == 'g') {
      currentMode = command;
      streaming = true;
      Serial.println((command == 'a') ? "Streaming accelerometer..." :
                                         "Streaming gyroscope...");
    } else if (command == 's') {
      streaming = false;
      Serial.println("Stopped");
    }
  }

  // Stream data at ~20 Hz (see delay at bottom)
  if (streaming) {
    if (currentMode == 'a')      streamAccelerometer();
    else if (currentMode == 'g') streamGyroscope();
    delay(50);
  }
}

void streamAccelerometer() {
  // Set the register pointer to ACCEL_XOUT_H using a repeated-start
  // (endTransmission(false)) so we can immediately read without releasing the bus.
  Wire.beginTransmission(ICM20600_ADDR);
  Wire.write(ICM20600_ACCEL_XOUT_H);
  Wire.endTransmission(false);          // repeated-start (no STOP)
  Wire.requestFrom(ICM20600_ADDR, 6);   // read X/Y/Z (high+low bytes)

  // Read raw 16-bit values
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();

  // Scale to 'g' assuming ±2g full-scale (16384 LSB per g)
  //Serial.print(">acc_x:"); Serial.println(ax / 16384.0, 3);
  //Serial.print(">acc_y:"); Serial.println(ay / 16384.0, 3);
  //Serial.print(">acc_z:"); Serial.println(az / 16384.0, 3);



  
  
// One chart named "accel", three lines: ax, ay, az
  Serial.print(">ax,accel:"); Serial.println(ax / 16384.0, 3);
  Serial.print(">ay,accel:"); Serial.println(ay / 16384.0, 3);
  Serial.print(">az,accel:"); Serial.println(az / 16384.0, 3);

}

void streamGyroscope() {
  // Set the register pointer to GYRO_XOUT_H
  Wire.beginTransmission(ICM20600_ADDR);
  Wire.write(ICM20600_GYRO_XOUT_H);
  Wire.endTransmission(false);          // repeated-start (no STOP)
  Wire.requestFrom(ICM20600_ADDR, 6);   // read X/Y/Z (high+low bytes)

  // Read raw 16-bit values
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  // Scale to deg/s assuming ±250 dps (131 LSB per dps)
  //Serial.print(">gyro_x:"); Serial.println(gx / 131.0, 2);
  //Serial.print(">gyro_y:"); Serial.println(gy / 131.0, 2);
  //Serial.print(">gyro_z:"); Serial.println(gz / 131.0, 2);

  // One chart named "gyro", three lines: gx, gy, gz
  Serial.print(">gx,gyro:"); Serial.println(gx / 131.0, 2);
  Serial.print(">gy,gyro:"); Serial.println(gy / 131.0, 2);
  Serial.print(">gz,gyro:"); Serial.println(gz / 131.0, 2);

}
