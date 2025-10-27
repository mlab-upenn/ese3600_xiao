// 
// VWW - Person Detection
// 
// (C) 2023 Heechul Yun <heechul.yun@ku.edu>
//   
// Licensed under MIT License
// 
// Description:
// This is a simple person detection program using ESP32-S3 camera.
// The program uses a pre-trained neural network model to detect a person in the image.
// The result is shown on the serial monitor and the built-in LED.
//

#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h" //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems
#include "NeuralNetwork.h"

#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
#include "camera_pins.h"

#define INPUT_W 96
#define INPUT_H 96
#define LED_BUILT_IN 21

#define DEBUG_TFLITE 0

#if DEBUG_TFLITE==1
#include "img.h"  // Use a static image for debugging
#endif

NeuralNetwork *g_nn;

// Convert RGB565 to RGB888
uint32_t rgb565torgb888(uint16_t color)
{
    uint8_t hb, lb;
    uint32_t r, g, b;

    lb = (color >> 8) & 0xFF;
    hb = color & 0xFF;

    r = (lb & 0x1F) << 3;
    g = ((hb & 0x07) << 5) | ((lb & 0xE0) >> 3);
    b = (hb & 0xF8);

    return (r << 16) | (g << 8) | b;
}

// Get image from camera frame buffer and convert it to the input tensor
int GetImage(camera_fb_t * fb, TfLiteTensor* input) 
{
    assert(fb->format == PIXFORMAT_RGB565);

    // Trimming Image
    int post = 0;
    int startx = (fb->width - INPUT_W) / 2;
    int starty = (fb->height - INPUT_H);
    for (int y = 0; y < INPUT_H; y++) {
        for (int x = 0; x < INPUT_W; x++) {
            int getPos = (starty + y) * fb->width + startx + x;
            // MicroPrintf("input[%d]: fb->buf[%d]=%d\n", post, getPos, fb->buf[getPos]);
            uint16_t color = ((uint16_t *)fb->buf)[getPos];
            uint32_t rgb = rgb565torgb888(color);

            float *image_data = input->data.f;

            image_data[post * 3 + 0] = ((rgb >> 16) & 0xFF);  // R
            image_data[post * 3 + 1] = ((rgb >> 8) & 0xFF);   // G
            image_data[post * 3 + 2] = (rgb & 0xFF);          // B
            post++;
        }
    }
    return 0;
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
  Serial.begin(115200);

  while(!Serial) {
    static int retries = 0;
    delay(1000); // Wait for serial monitor to open
    if (retries++ > 5) {
      break;
    }
  } // When the serial monitor is turned on, the program starts to execute
  
  Serial.setDebugOutput(false);
  
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
  config.frame_size = FRAMESIZE_96X96;
  config.pixel_format = PIXFORMAT_RGB565; // PIXFORMAT_JPEG; // for streaming
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  
  // Pin for LED
  pinMode(LED_BUILT_IN, OUTPUT); // Set the pin as output

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  Serial.printf("Camera init success!\n");
  Serial.printf("frame_size=%d\n", config.frame_size);
  Serial.printf("pixel_format=%d\n", config.pixel_format);

  // Initialize neural network
  Serial.println("Initializing neural network...");
  g_nn = new NeuralNetwork();

}

// Main loop
void loop() {

  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;

  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    res = ESP_FAIL;
  } else {
    if(fb->format != PIXFORMAT_JPEG){
      uint64_t start, dur_prep, dur_infer;
#if DEBUG_TFLITE==0
      // Use camera image
      start = esp_timer_get_time();
      GetImage(fb, g_nn->getInput());
      dur_prep = esp_timer_get_time() - start;
#else
      // Use a static image for debugging
      memcpy(g_nn->getInput()->data.f, img_data, sizeof(img_data));
      printf("input: %.3f %.3f %.3f...\n", 
        g_nn->getInput()->data.f[0], g_nn->getInput()->data.f[1], g_nn->getInput()->data.f[2]);
#endif
      // measure timing 
      start = esp_timer_get_time();
      g_nn->predict();
      dur_infer = esp_timer_get_time() - start;
      Serial.printf("Preprocessing: %llu ms, Inference: %llu ms\n", dur_prep/1000, dur_infer/1000);

      float prob = g_nn->getOutput()->data.f[0];
      Serial.printf("output: %.3f --> ", prob);
      if (prob < 0.5) {
        Serial.println("No person");
        digitalWrite(LED_BUILT_IN, HIGH); //Turn off
      } else {
        Serial.println("Person");
        // show the inference result on LED
        digitalWrite(LED_BUILT_IN, LOW); //Turn on
      }
      esp_camera_fb_return(fb);
      fb = NULL;
    }
  }
}