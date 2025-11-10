#include "gesture_predictor.h"
#include "constants.h"
#include "model_data.h"
#include "esp_log.h"

#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

static const char* TAG = "GesturePredictor";

namespace {
    const tflite::Model* model = nullptr;
    tflite::MicroInterpreter* interpreter = nullptr;
    TfLiteTensor* input = nullptr;
    TfLiteTensor* output = nullptr;
    
    // Tensor arena for TFLite operations
    constexpr int kTensorArenaSize = 100 * 1024;  // 100KB
    alignas(16) uint8_t tensor_arena[kTensorArenaSize];
}

bool SetupGesturePredictor() {
    ESP_LOGI(TAG, "Initializing TensorFlow Lite...");
    
    // Map the model into a usable data structure
    model = tflite::GetModel(g_model);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "Model schema version %lu doesn't match supported version %d",
                 model->version(), TFLITE_SCHEMA_VERSION);
        return false;
    }
    
    // Create the operator resolver
    static tflite::MicroMutableOpResolver<10> micro_op_resolver;
    
    micro_op_resolver.AddConv2D();
    micro_op_resolver.AddReshape();
    micro_op_resolver.AddFullyConnected();
    micro_op_resolver.AddSoftmax();
    micro_op_resolver.AddQuantize();
    micro_op_resolver.AddDequantize();
    micro_op_resolver.AddMean();
    
    micro_op_resolver.AddMaxPool2D();
    micro_op_resolver.AddRelu();
    micro_op_resolver.AddLogistic();
    
    // Build an interpreter to run the model
    static tflite::MicroInterpreter static_interpreter(
        model, micro_op_resolver, tensor_arena, kTensorArenaSize);
    interpreter = &static_interpreter;
    
    // Allocate memory for the model's tensors
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors() failed");
        return false;
    }
    
    // Get pointers to the model's input and output tensors
    input = interpreter->input(0);
    output = interpreter->output(0);
    
    // Log tensor information
    ESP_LOGI(TAG, "Input tensor:");
    ESP_LOGI(TAG, "  - Dims: %d", input->dims->size);
    for (int i = 0; i < input->dims->size; i++) {
        ESP_LOGI(TAG, "    - Dim[%d]: %d", i, input->dims->data[i]);
    }
    ESP_LOGI(TAG, "  - Type: %d", input->type);
    ESP_LOGI(TAG, "  - Bytes: %d", input->bytes);
    
    ESP_LOGI(TAG, "Output tensor:");
    ESP_LOGI(TAG, "  - Dims: %d", output->dims->size);
    for (int i = 0; i < output->dims->size; i++) {
        ESP_LOGI(TAG, "    - Dim[%d]: %d", i, output->dims->data[i]);
    }
    ESP_LOGI(TAG, "  - Type: %d", output->type);
    
    // Log arena usage
    ESP_LOGI(TAG, "Tensor arena used: %d / %d bytes", 
             interpreter->arena_used_bytes(), kTensorArenaSize);
    
    ESP_LOGI(TAG, "TensorFlow Lite initialized successfully!");
    return true;
}

bool PredictGesture(const uint8_t* image_data, int* gesture_index, int8_t* confidence) {
    if (!interpreter || !input || !output) {
        ESP_LOGE(TAG, "Model not initialized");
        return false;
    }
    
    // Copy image data to input tensor
    // The model expects uint8 values (0-255) for RGB image
    if (input->type == kTfLiteUInt8) {
        // Direct copy for uint8 input
        uint8_t* input_data = input->data.uint8;
        for (int i = 0; i < kInputTensorSize; i++) {
            input_data[i] = image_data[i];
        }
    } else if (input->type == kTfLiteInt8) {
        // Convert to int8 (-128 to 127)
        int8_t* input_data = input->data.int8;
        for (int i = 0; i < kInputTensorSize; i++) {
            input_data[i] = static_cast<int8_t>(image_data[i] - 128);
        }
    } else {
        ESP_LOGE(TAG, "Unsupported input tensor type: %d", input->type);
        return false;
    }
    
    // Run inference
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
        ESP_LOGE(TAG, "Invoke failed");
        return false;
    }
    
    // Find the gesture with highest confidence
    int8_t max_score = -128;
    int max_index = 0;
    
    if (output->type == kTfLiteInt8) {
        int8_t* output_data = output->data.int8;
        for (int i = 0; i < kGestureCount; i++) {
            if (output_data[i] > max_score) {
                max_score = output_data[i];
                max_index = i;
            }
        }
    } else if (output->type == kTfLiteUInt8) {
        uint8_t* output_data = output->data.uint8;
        for (int i = 0; i < kGestureCount; i++) {
            int8_t score = static_cast<int8_t>(output_data[i] - 128);
            if (score > max_score) {
                max_score = score;
                max_index = i;
            }
        }
    } else {
        ESP_LOGE(TAG, "Unsupported output tensor type: %d", output->type);
        return false;
    }
    
    *gesture_index = max_index;
    *confidence = max_score;
    
    ESP_LOGI(TAG, "Prediction: gesture=%d (%s), confidence=%d", 
             max_index, kGestureLabels[max_index], max_score);
    
    return true;
}
