/* 
   Using MicroMutableOpResolver with all standard ops
   + BUZZER for Keyword 1 and Keyword 2
*/

#include "main_functions.h"
#include "audio_provider.h"
#include "command_responder.h"
#include "driver/gpio.h"
#include "driver/ledc.h"  // Added this for buzzer
#include "feature_provider.h"
#include "micro_model_settings.h"
#include "model.h"
#include "recognize_commands.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/core/c/common.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"

#define BUZZER_GPIO      GPIO_NUM_4  // A3 on expansion board
#define LEDC_TIMER       LEDC_TIMER_0
#define LEDC_MODE        LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL     LEDC_CHANNEL_0
#define LEDC_DUTY_RES    LEDC_TIMER_8_BIT
#define LEDC_DUTY        128
#define FREQ_GO          1000  // High beep for Keyword 1
#define FREQ_STOP        500   // Low beep for Keyword 2

namespace {
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* model_input = nullptr;
FeatureProvider* feature_provider = nullptr;
RecognizeCommands* recognizer = nullptr;
int32_t previous_time = 0;
constexpr gpio_num_t kIndicatorGpio = GPIO_NUM_21;

// Increased memory
constexpr int kTensorArenaSize = 50 * 1024;  // 50KB
uint8_t tensor_arena[kTensorArenaSize];
int8_t feature_buffer[kFeatureElementCount];
int8_t* model_input_buffer = nullptr;
}

void buzzer_init() {
  ledc_timer_config_t ledc_timer = {
    .speed_mode       = LEDC_MODE,
    .duty_resolution  = LEDC_DUTY_RES,
    .timer_num        = LEDC_TIMER,
    .freq_hz          = 1000,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  ledc_channel_config_t ledc_channel = {
    .gpio_num       = BUZZER_GPIO,
    .speed_mode     = LEDC_MODE,
    .channel        = LEDC_CHANNEL,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = LEDC_TIMER,
    .duty           = 0,
    .hpoint         = 0
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void buzzer_tone(uint32_t frequency, uint32_t duration_ms) {
  ledc_set_freq(LEDC_MODE, LEDC_TIMER, frequency);
  ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
  vTaskDelay(pdMS_TO_TICKS(duration_ms));
  ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void setup() {
  model = tflite::GetModel(g_model);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    MicroPrintf("Model schema version %d not equal to supported version %d.",
                model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  // Includes ALL common operators
  static tflite::MicroMutableOpResolver<10> micro_op_resolver;
  
  // Adding BOTH Conv2D types
  if (micro_op_resolver.AddConv2D() != kTfLiteOk) {
    MicroPrintf("Failed to add Conv2D");
    return;
  }
  
  if (micro_op_resolver.AddDepthwiseConv2D() != kTfLiteOk) {
    MicroPrintf("Failed to add DepthwiseConv2D");
    return;
  }
  
  // Adding pooling
  if (micro_op_resolver.AddMaxPool2D() != kTfLiteOk) {
    MicroPrintf("Failed to add MaxPool2D");
    return;
  }
  
  if (micro_op_resolver.AddAveragePool2D() != kTfLiteOk) {
    MicroPrintf("Failed to add AveragePool2D");
    return;
  }
  
  // Adding dense layers
  if (micro_op_resolver.AddFullyConnected() != kTfLiteOk) {
    MicroPrintf("Failed to add FullyConnected");
    return;
  }
  
  // Adding activations
  if (micro_op_resolver.AddSoftmax() != kTfLiteOk) {
    MicroPrintf("Failed to add Softmax");
    return;
  }
  
  // Adding utilities
  if (micro_op_resolver.AddReshape() != kTfLiteOk) {
    MicroPrintf("Failed to add Reshape");
    return;
  }
  
  if (micro_op_resolver.AddQuantize() != kTfLiteOk) {
    MicroPrintf("Failed to add Quantize");
    return;
  }
  
  if (micro_op_resolver.AddDequantize() != kTfLiteOk) {
    MicroPrintf("Failed to add Dequantize");
    return;
  }
  
  //adding Pad (some models need it)
  micro_op_resolver.AddPad();  
  
  MicroPrintf("Registered 10 operators");

  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    MicroPrintf("AllocateTensors() failed");
    return;
  }

  model_input = interpreter->input(0);
  if ((model_input->dims->size != 2) || (model_input->dims->data[0] != 1) ||
      (model_input->dims->data[1] != (kFeatureCount * kFeatureSize)) ||
      (model_input->type != kTfLiteInt8)) {
    MicroPrintf("Bad input tensor parameters");
    return;
  }
  model_input_buffer = tflite::GetTensorData<int8_t>(model_input);

  static FeatureProvider static_feature_provider(kFeatureElementCount,
                                                 feature_buffer);
  feature_provider = &static_feature_provider;

  static RecognizeCommands static_recognizer;
  recognizer = &static_recognizer;

  previous_time = 0;

  gpio_config_t io_conf = {};
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = 1ULL << kIndicatorGpio;
  gpio_config(&io_conf);
  gpio_set_level(kIndicatorGpio, 0);
  

  buzzer_init();
  MicroPrintf("Buzzer ready on GPIO4");
  buzzer_tone(1000, 100);  // Test beep
  vTaskDelay(pdMS_TO_TICKS(100));
  
  MicroPrintf("Setup complete - ready to detect keywords!");
}

void loop() {
  const int32_t current_time = LatestAudioTimestamp();
  int how_many_new_slices = 0;
  TfLiteStatus feature_status = feature_provider->PopulateFeatureData(
      previous_time, current_time, &how_many_new_slices);
  if (feature_status != kTfLiteOk) {
    MicroPrintf("Feature generation failed");
    return;
  }
  previous_time = current_time;
  
  if (how_many_new_slices == 0) {
    return;
  }

  for (int i = 0; i < kFeatureElementCount; i++) {
    model_input_buffer[i] = feature_buffer[i];
  }

  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    MicroPrintf("Invoke failed");
    return;
  }

  TfLiteTensor* output = interpreter->output(0);
  float output_scale = output->params.scale;
  int output_zero_point = output->params.zero_point;
  int max_idx = 0;
  float max_result = 0.0;
  
  for (int i = 0; i < kCategoryCount; i++) {
    float result = (tflite::GetTensorData<int8_t>(output)[i] - 
                    output_zero_point) * output_scale;
    if (result > max_result) {
      max_result = result;
      max_idx = i;
    }
  }
  
  // Feel free to change this threshold value to get better result
  if (max_result > 0.6f) {
    MicroPrintf("Detected %7s, score: %.2f", kCategoryLabels[max_idx],
                static_cast<double>(max_result));
    
    // ADD BUZZER CONTROL HERE
    if (max_idx == 2) {
      // Replace Keyword 1 Eg: GO = High beep
      buzzer_tone(FREQ_GO, 300);
      MicroPrintf("Keyword 1 - High beep!");
    } else if (max_idx == 3) {
      // Replace Keyword 2 Eg: STOP = Low beep
      buzzer_tone(FREQ_STOP, 300);
      MicroPrintf("Keyword 2 - Low beep!");
    }
    
    int num_blinks = 0;
    if (max_idx == 2) num_blinks = 1;      // First keyword
    else if (max_idx == 3) num_blinks = 2; // Second keyword
    
    for (int i = 0; i < num_blinks; i++) {
      gpio_set_level(kIndicatorGpio, 1);
      vTaskDelay(pdMS_TO_TICKS(100));
      gpio_set_level(kIndicatorGpio, 0);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}