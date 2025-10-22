#include <Arduino.h>
#include "esp_camera.h"

// Camera pins for XIAO ESP32S3 Sense
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

enum Mode {
  IDLE,
  LIVE_MODE,
  SINGLE_MODE
};

Mode currentMode = IDLE;

void configCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // Image quality settings - CIF resolution (400x296)
  config.frame_size = FRAMESIZE_CIF; // 400x296 - Medium resolution
  config.jpeg_quality = 15; // Good quality
  config.fb_count = 1;
  
  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }
  Serial.println("Camera initialized successfully!");
}

void captureAndPrintHex() {
  Serial.println("\n Capturing Image");
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed!");
    return;
  }
  
  Serial.printf("Image captured! Size: %d bytes\n", fb->len);
  Serial.println("\n Hexadecimal Data (Copy for Colab) ");
  
  // Print hex data with commas
  for (size_t i = 0; i < fb->len; i++) {
    Serial.printf("0x%02X", fb->buf[i]);
    if (i < fb->len - 1) {
      Serial.print(",");
    }
    // Add newline every 16 bytes for readability
    if ((i + 1) % 16 == 0) {
      Serial.println();
    }
  }
  
  Serial.println("\n End of Data\n");
  esp_camera_fb_return(fb);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nXIAO ESP32S3 Sense Camera Test - Serial Output");
  
  // Initialize camera
  configCamera();
  
  Serial.println("\nCamera ready!");
  Serial.println("Commands:");
  Serial.println("  'live' - Start live mode (continuous captures)");
  Serial.println("  'single' - Enter single mode");
  Serial.println("  'capture' - Capture image (only in single mode, waits 3s)");
  Serial.println("  'stop' - Stop live mode");
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();
    
    if (cmd == "live") {
      currentMode = LIVE_MODE;
      Serial.println("\n LIVE MODE STARTED ");
      Serial.println("Continuous captures active. Type 'stop' to exit.\n");
    }
    else if (cmd == "stop") {
      if (currentMode == LIVE_MODE) {
        currentMode = IDLE;
        Serial.println("\n LIVE MODE STOPPED \n");
      }
    }
    else if (cmd == "single") {
      currentMode = SINGLE_MODE;
      Serial.println("\n SINGLE MODE ACTIVE ");
      Serial.println("Type 'capture' to take a picture (3 second countdown).\n");
    }
    else if (cmd == "capture") {
      if (currentMode == SINGLE_MODE) {
        Serial.println("\nPreparing to capture...");
        Serial.println("3...");
        delay(1000);
        Serial.println("2...");
        delay(1000);
        Serial.println("1...");
        delay(1000);
        captureAndPrintHex();
      } else {
        Serial.println("Error: Must be in SINGLE mode to capture. Type 'single' first.");
      }
    }
  }
  
  // If in live mode, continuously capture and print
  if (currentMode == LIVE_MODE) {
    captureAndPrintHex();
    delay(500);  // Small delay between captures (you can adjust this as needed)
  } else {
    delay(100);
  }
}