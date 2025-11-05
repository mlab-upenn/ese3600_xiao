// #include <ESP_TF.h>

// #if !defined(CONFIG_NN_OPTIMIZED)
// #error "CONFIG_NN_OPTIMIZED"
// #endif
// #if !defined(CONFIG_IDF_TARGET_ESP32S3)
// #error "CONFIG_IDF_TARGET_ESP32S3"
// #endif

#include "NeuralNetwork.h"
#include "model.cc"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"

static const char *TAG = "vww_nn";

int kTensorArenaSize = 3 * 1024 * 1024;

NeuralNetwork::NeuralNetwork()
{
    ESP_LOGI(TAG, "Free heap: %d", esp_get_free_heap_size());
    ESP_LOGI(TAG, "largest size (8bit): %d", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    ESP_LOGI(TAG, "largest size (default): %d", heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    ESP_LOGI(TAG, "largest size (spiram): %d", heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "largest size (internal): %d", heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

    // get model (.tflite) from flash
    model = tflite::GetModel(gmodel);
    if (model->version() != TFLITE_SCHEMA_VERSION)
    {
        MicroPrintf("Model provided is schema version %d not equal to supported "
                    "version %d.",
                    model->version(), TFLITE_SCHEMA_VERSION);
        return;
    }
#if USE_ALLOPS==1
    static tflite::AllOpsResolver micro_op_resolver;
#else
    static tflite::MicroMutableOpResolver<13> micro_op_resolver;
    micro_op_resolver.AddFullyConnected();
    micro_op_resolver.AddConv2D();
    micro_op_resolver.AddDepthwiseConv2D();
    micro_op_resolver.AddMaxPool2D();
    micro_op_resolver.AddMul();
    micro_op_resolver.AddAdd();
    micro_op_resolver.AddLogistic();
    micro_op_resolver.AddTanh();
    micro_op_resolver.AddRelu();
    micro_op_resolver.AddRelu6();
    micro_op_resolver.AddReshape();
    micro_op_resolver.AddQuantize();
    micro_op_resolver.AddDequantize();
#endif
    tensor_arena = static_cast<uint8_t *>(heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (tensor_arena == NULL)
    {
        ESP_LOGE(TAG, "Couldn't allocate memory of %d bytes", kTensorArenaSize);
        return;
    }
    ESP_LOGI(TAG, "Free heap after arena alloc: %d", esp_get_free_heap_size());

    // Build an interpreter to run the model with.
    static tflite::MicroInterpreter static_interpreter(
        model, micro_op_resolver, tensor_arena, kTensorArenaSize);
    interpreter = &static_interpreter;

    MicroPrintf("interpreter initialization");
    // Allocate memory from the tensor_arena for the model's tensors.
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk)
    {
        MicroPrintf("AllocateTensors() failed");
        return;
    }

    size_t used_bytes = interpreter->arena_used_bytes();
    MicroPrintf("Used bytes %d\n", used_bytes);

    // Obtain pointers to the model's input and output tensors.
    input = interpreter->input(0);
    output = interpreter->output(0);

    ESP_LOGI(TAG, "tensor_arena: %p, input: %p", tensor_arena, input->data.uint8);
    ESP_LOGI(TAG, "input->dims->size: %d", input->dims->size);
    ESP_LOGI(TAG, "input->dims->data[0]: %d", input->dims->data[0]);
    ESP_LOGI(TAG, "input->dims->data[1]: %d", input->dims->data[1]);
    ESP_LOGI(TAG, "input->dims->data[2]: %d", input->dims->data[2]);
    ESP_LOGI(TAG, "input->dims->data[3]: %d", input->dims->data[3]);
    ESP_LOGI(TAG, "input->type: %d", input->type);
    ESP_LOGI(TAG, "input->params.scale: %.3f", input->params.scale);
    ESP_LOGI(TAG, "input->params.zero_point: %d", input->params.zero_point);
    
    float scale = output->params.scale;
    int zero_point = output->params.zero_point; 
    ESP_LOGI(TAG, "output scale=%.3f, zero_point=%d", scale, zero_point);

} 

TfLiteTensor* NeuralNetwork::getInput()
{
    return input;
}

TfLiteStatus NeuralNetwork::predict()
{
    return interpreter->Invoke();
}

TfLiteTensor* NeuralNetwork::getOutput()
{
    return output;
}
