/* Magic Wand - Fixed for ICM-20600 to match Arduino Nano BLE training data like done in Pete's code
 * The training data expects normalized x,y coordinates from integrated gyroscope angles
 */

#include <Arduino.h>
#include <ArduinoBLE.h>

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#include "imu_provider.h"
#include "magic_wand_model_data.h"
#include "rasterize_stroke.h"

#define BLE_SENSE_UUID(val) ("4798e0f2-" val "-4d68-af64-8a8f5258404e")

namespace {

const int VERSION = 0x00000000;
constexpr float kDefaultImuSampleRateHz = 104.0f;

constexpr int stroke_transmit_stride = 2;
constexpr int stroke_transmit_max_length = 160;
constexpr int stroke_max_length = stroke_transmit_max_length * stroke_transmit_stride;
constexpr int stroke_points_byte_count = 2 * sizeof(int8_t) * stroke_transmit_max_length;
constexpr int stroke_struct_byte_count = (2 * sizeof(int32_t)) + stroke_points_byte_count;
constexpr int moving_sample_count = 50;

constexpr int raster_width = 32;
constexpr int raster_height = 32;
constexpr int raster_channels = 3;
constexpr int raster_byte_count = raster_height * raster_width * raster_channels;
int8_t raster_buffer[raster_byte_count];

unsigned long last_detection_ms = 0;
const unsigned long detection_cooldown_ms = 1500;

BLEService        service                       (BLE_SENSE_UUID("0000"));
BLECharacteristic strokeCharacteristic          (BLE_SENSE_UUID("300a"), BLERead, stroke_struct_byte_count);

String name;

// Reduced buffer sizes
constexpr int acceleration_data_length = 300 * 3;
float acceleration_data[acceleration_data_length] = {};
int acceleration_data_index = 0;
float acceleration_sample_rate = kDefaultImuSampleRateHz;

constexpr int gyroscope_data_length = 300 * 3;
float gyroscope_data[gyroscope_data_length] = {};
float orientation_data[gyroscope_data_length] = {};
int gyroscope_data_index = 0;
float gyroscope_sample_rate = kDefaultImuSampleRateHz;

float current_velocity[3] = {0.0f, 0.0f, 0.0f};
float current_position[3] = {0.0f, 0.0f, 0.0f};
float current_gravity[3] = {0.0f, 0.0f, 0.0f};
float current_gyroscope_drift[3] = {0.0f, 0.0f, 0.0f};

int32_t stroke_length = 0;
uint8_t stroke_struct_buffer[stroke_struct_byte_count] = {};
int32_t* stroke_state = reinterpret_cast<int32_t*>(stroke_struct_buffer);
int32_t* stroke_transmit_length = reinterpret_cast<int32_t*>(stroke_struct_buffer + sizeof(int32_t));
int8_t* stroke_points = reinterpret_cast<int8_t*>(stroke_struct_buffer + (sizeof(int32_t) * 2));

enum { 
  eWaiting = 0,
  eDrawing = 1,
  eDone = 2,
};

constexpr int kTensorArenaSize = 25 * 1024;
alignas(16) uint8_t tensor_arena[kTensorArenaSize];

const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;

constexpr int label_count = 10;
const char* labels[label_count] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9"};

void ReadAccelerometerAndGyroscope(int* new_accelerometer_samples, int* new_gyroscope_samples) {
  *new_accelerometer_samples = 0;
  *new_gyroscope_samples = 0;

  float accel_sample[3];
  float gyro_sample[3];
  if (!ReadIMU(accel_sample, gyro_sample)) {
    return;
  }

  const int gyroscope_index = (gyroscope_data_index % gyroscope_data_length);
  gyroscope_data[gyroscope_index] = gyro_sample[0];
  gyroscope_data[gyroscope_index + 1] = gyro_sample[1];
  gyroscope_data[gyroscope_index + 2] = gyro_sample[2];
  gyroscope_data_index += 3;
  *new_gyroscope_samples = 1;

  const int acceleration_index = (acceleration_data_index % acceleration_data_length);
  acceleration_data[acceleration_index] = accel_sample[0];
  acceleration_data[acceleration_index + 1] = accel_sample[1];
  acceleration_data[acceleration_index + 2] = accel_sample[2];
  acceleration_data_index += 3;
  *new_accelerometer_samples = 1;
}

float VectorMagnitude(const float* vec) {
  return sqrtf((vec[0] * vec[0]) + (vec[1] * vec[1]) + (vec[2] * vec[2]));
}

void NormalizeVector(const float* in_vec, float* out_vec) {
  const float magnitude = VectorMagnitude(in_vec);
  if (magnitude < 0.0001f) {
    out_vec[0] = 0.0f;
    out_vec[1] = 0.0f;
    out_vec[2] = 0.0f;
    return;
  }
  out_vec[0] = in_vec[0] / magnitude;
  out_vec[1] = in_vec[1] / magnitude;
  out_vec[2] = in_vec[2] / magnitude;
}

float DotProduct(const float* a, const float* b) {
  return (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
}

void EstimateGravityDirection(float* gravity) {
  int samples_to_average = 50;
  if (samples_to_average >= acceleration_data_index) {
    samples_to_average = acceleration_data_index;
  }

  const int start_index = ((acceleration_data_index + 
    (acceleration_data_length - (3 * (samples_to_average + 1)))) % 
    acceleration_data_length);

  float x_total = 0.0f;
  float y_total = 0.0f;
  float z_total = 0.0f;
  for (int i = 0; i < samples_to_average; ++i) {
    const int index = ((start_index + (i * 3)) % acceleration_data_length);
    const float* entry = &acceleration_data[index];
    x_total += entry[0];
    y_total += entry[1];
    z_total += entry[2];
  }
  gravity[0] = x_total / samples_to_average;
  gravity[1] = y_total / samples_to_average;
  gravity[2] = z_total / samples_to_average;
}

void UpdateVelocity(int new_samples, float* gravity) {
  const float friction_fudge = 0.98f;
  const int start_index = ((acceleration_data_index + 
    (acceleration_data_length - (3 * (new_samples + 1)))) % 
    acceleration_data_length);

  for (int i = 0; i < new_samples; ++i) {
    const int index = ((start_index + (i * 3)) % acceleration_data_length);
    const float* entry = &acceleration_data[index];
    
    current_velocity[0] += (entry[0] - gravity[0]);
    current_velocity[1] += (entry[1] - gravity[1]);
    current_velocity[2] += (entry[2] - gravity[2]);
    
    current_velocity[0] *= friction_fudge;
    current_velocity[1] *= friction_fudge;
    current_velocity[2] *= friction_fudge;
    
    current_position[0] += current_velocity[0];
    current_position[1] += current_velocity[1];
    current_position[2] += current_velocity[2];
  }
}

void EstimateGyroscopeDrift(float* drift) {
  const bool isMoving = VectorMagnitude(current_velocity) > 0.1f;
  if (isMoving) {
    return;
  }
  
  int samples_to_average = 20;
  if (samples_to_average >= gyroscope_data_index) {
    samples_to_average = gyroscope_data_index;
  }

  const int start_index = ((gyroscope_data_index + 
    (gyroscope_data_length - (3 * (samples_to_average + 1)))) % 
    gyroscope_data_length);

  float x_total = 0.0f;
  float y_total = 0.0f;
  float z_total = 0.0f;
  for (int i = 0; i < samples_to_average; ++i) {
    const int index = ((start_index + (i * 3)) % gyroscope_data_length);
    const float* entry = &gyroscope_data[index];
    x_total += entry[0];
    y_total += entry[1];
    z_total += entry[2];
  }
  drift[0] = x_total / samples_to_average;
  drift[1] = y_total / samples_to_average;
  drift[2] = z_total / samples_to_average;
}

// CRITICAL FIX: Fixed the Proper orientation integration matching training data
void UpdateOrientation(int new_samples, float* gravity, float* drift) {
  // Convert deg/s to rad/s and integrate properly
  const float deg_to_rad = 0.017453292519943295f;  // PI/180
  const float dt = 1.0f / gyroscope_sample_rate;  // Time step
  
  const int start_index = ((gyroscope_data_index + 
    (gyroscope_data_length - (3 * (new_samples + 1)))) % 
    gyroscope_data_length);

  for (int i = 0; i < new_samples; ++i) {
    const int index = ((start_index + (i * 3)) % gyroscope_data_length);
    const float* gyro_entry = &gyroscope_data[index];
    
    // Remove drift and convert to radians/second
    const float gx = (gyro_entry[0] - drift[0]) * deg_to_rad;
    const float gy = (gyro_entry[1] - drift[1]) * deg_to_rad;
    const float gz = (gyro_entry[2] - drift[2]) * deg_to_rad;

    // Simple Euler integration: angle += angular_velocity * dt
    float* current_orientation = &orientation_data[index];
    const int previous_index = (index + (gyroscope_data_length - 3)) % gyroscope_data_length;
    const float* previous_orientation = &orientation_data[previous_index];
    
    current_orientation[0] = previous_orientation[0] + (gx * dt);
    current_orientation[1] = previous_orientation[1] + (gy * dt);
    current_orientation[2] = previous_orientation[2] + (gz * dt);
  }
}

bool IsMoving(int samples_before) {
  // Motion detection based on gyroscope changes
  constexpr float moving_threshold = 0.3f;
  
  if ((gyroscope_data_index - samples_before) < moving_sample_count) {
    return false;
  }

  const int start_index = ((gyroscope_data_index + 
    (gyroscope_data_length - (3 * (moving_sample_count + samples_before)))) % 
    gyroscope_data_length);

  float total = 0.0f;
  for (int i = 0; i < moving_sample_count; ++i) {
    const int index = ((start_index + (i * 3)) % gyroscope_data_length);
    const float* current_orientation = &orientation_data[index];
    const int previous_index = (index + (gyroscope_data_length - 3)) % gyroscope_data_length;
    const float* previous_orientation = &orientation_data[previous_index];
    const float dx = current_orientation[0] - previous_orientation[0];
    const float dy = current_orientation[1] - previous_orientation[1];
    const float dz = current_orientation[2] - previous_orientation[2];
    total += (dx * dx) + (dy * dy) + (dz * dz);
  }
  
  return (total > moving_threshold);
}

void UpdateStroke(int new_samples, bool* done_just_triggered) {
  constexpr int minimum_stroke_length = moving_sample_count + 15;
  constexpr float minimum_stroke_size = 0.10f;  // Match training data scale

  *done_just_triggered = false;

  for (int i = 0; i < new_samples; ++i) {
    const int current_head = (new_samples - (i + 1));
    const bool is_moving = IsMoving(current_head);
    const int32_t old_state = *stroke_state;
    
    if ((old_state == eWaiting) || (old_state == eDone)) {
      if (is_moving) {
        stroke_length = moving_sample_count;
        *stroke_state = eDrawing;
      }
    } else if (old_state == eDrawing) {
      if (is_moving) {
        stroke_length += 1;
        if (stroke_length >= stroke_max_length) {
          stroke_length = stroke_max_length;
        }
      } else {
        if (stroke_length > minimum_stroke_length) {
          *stroke_state = eDone;
          *done_just_triggered = true;
        } else {
          stroke_length = 0;
          *stroke_state = eWaiting;
        }
      }
    }
  }

  if (*stroke_state == eDrawing) {
    constexpr int max_orientations = (gyroscope_data_length / 3);
    *stroke_transmit_length = ((stroke_length / stroke_transmit_stride) + 1);
    
    const int orientation_start_index = ((gyroscope_data_index / 3) + 
      (max_orientations - stroke_length));
    
    // FIX: Use orientation_data directly (angles in radians)
    // The training data used X,Y angles directly as coordinates
    
    float x_min = 0, x_max = 0, y_min = 0, y_max = 0;
    
    // First pass: find bounds using X and Y angles
    for (int i = 0; i < stroke_length; i += stroke_transmit_stride) {
      const int orientation_index = (((orientation_start_index + i) % max_orientations) * 3);
      const float* orientation = &orientation_data[orientation_index];
      const float angle_x = orientation[0];  // Roll angle
      const float angle_y = orientation[1];  // Pitch angle

      if (i == 0) {
        x_min = x_max = angle_x;
        y_min = y_max = angle_y;
      } else {
        if (angle_x < x_min) x_min = angle_x;
        if (angle_x > x_max) x_max = angle_x;
        if (angle_y < y_min) y_min = angle_y;
        if (angle_y > y_max) y_max = angle_y;
      }
    }

    const float x_range = (x_max - x_min);
    const float y_range = (y_max - y_min);
    
    // Second pass: normalize and convert to int8
    for (int i = 0; i < stroke_length; i += stroke_transmit_stride) {
      const int orientation_index = (((orientation_start_index + i) % max_orientations) * 3);
      const float* orientation = &orientation_data[orientation_index];
      const float angle_x = orientation[0];
      const float angle_y = orientation[1];

      const int j = (i / stroke_transmit_stride);
      if (j >= stroke_transmit_max_length) break;

      // Normalize to -1 to +1 range, then scale to int8 range
      float norm_x = (x_range > 0.0001f) ? ((angle_x - x_min) / x_range) : 0.5f;
      float norm_y = (y_range > 0.0001f) ? ((angle_y - y_min) / y_range) : 0.5f;
      
      // Convert to -1 to +1
      norm_x = (norm_x * 2.0f) - 1.0f;
      norm_y = (norm_y * 2.0f) - 1.0f;

      const int stroke_index = j * 2;
      int8_t* stroke_entry = &stroke_points[stroke_index];
      
      int32_t val_x = static_cast<int32_t>(roundf(norm_x * 127.0f));
      stroke_entry[0] = (val_x > 127) ? 127 : (val_x < -128) ? -128 : val_x;
      
      int32_t val_y = static_cast<int32_t>(roundf(norm_y * 127.0f));
      stroke_entry[1] = (val_y > 127) ? 127 : (val_y < -128) ? -128 : val_y;
    }
    
    if (*done_just_triggered) {
      if ((x_range < minimum_stroke_size) && (y_range < minimum_stroke_size)) {
        *done_just_triggered = false;
        *stroke_state = eWaiting;
        *stroke_transmit_length = 0;
        stroke_length = 0;
      }
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  tflite::InitializeTarget();

  Serial.println("\n========================================");
  Serial.println("  Magic Wand  ");
  Serial.println("  For ICM-20600");
  Serial.println("========================================");

  if (!SetupIMU()) {
    Serial.println("ERROR: IMU failed!");
    while (1) delay(1000);
  }
  Serial.println("IMU ready");

  if (!BLE.begin()) {
    Serial.println("ERROR: BLE failed!");
    while (1);
  }

  String address = BLE.address();
  address.toUpperCase();
  name = "BLESense-" + address.substring(address.length() - 5);

  BLE.setLocalName(name.c_str());
  BLE.setDeviceName(name.c_str());
  BLE.setAdvertisedService(service);
  service.addCharacteristic(strokeCharacteristic);
  BLE.addService(service);
  BLE.advertise();
  Serial.println("✓ BLE ready");

  model = tflite::GetModel(g_magic_wand_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("ERROR: Model version!");
    return;
  }

  static tflite::MicroMutableOpResolver<4> micro_op_resolver;
  micro_op_resolver.AddConv2D();
  micro_op_resolver.AddMean();
  micro_op_resolver.AddFullyConnected();
  micro_op_resolver.AddSoftmax();

  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("ERROR: Tensor alloc failed!");
    return;
  }

  Serial.println("Model ready");
  Serial.println("========================================");
  Serial.println("Draw digits 0-9!");
  Serial.println("Hold still, then draw deliberately");
  Serial.println("========================================\n");
}

void loop() {
  BLEDevice central = BLE.central();
  
  int accel_samples = 0, gyro_samples = 0;
  ReadAccelerometerAndGyroscope(&accel_samples, &gyro_samples);

  if (accel_samples == 0 && gyro_samples == 0) return;

  bool done = false;
  if (gyro_samples > 0) {
    EstimateGyroscopeDrift(current_gyroscope_drift);
    UpdateOrientation(gyro_samples, current_gravity, current_gyroscope_drift);
    UpdateStroke(gyro_samples, &done);
    if (central && central.connected()) {
      strokeCharacteristic.writeValue(stroke_struct_buffer, stroke_struct_byte_count);
    }
  }

  if (accel_samples > 0) {
    EstimateGravityDirection(current_gravity);
    UpdateVelocity(accel_samples, current_gravity);
  }

  if (done) {
    Serial.println("\n>>> GESTURE <<<");
    
    RasterizeStroke(stroke_points, *stroke_transmit_length, 0.6f, 0.6f, 
                   raster_width, raster_height, raster_buffer);
    
    for (int y = 0; y < raster_height; ++y) {
      for (int x = 0; x < raster_width; ++x) {
        const int8_t* pixel = &raster_buffer[(y * raster_width * raster_channels) + (x * raster_channels)];
        Serial.print((pixel[0] > -128 || pixel[1] > -128 || pixel[2] > -128) ? '#' : '.');
      }
      Serial.println();
    }
    
    TfLiteTensor* model_input = interpreter->input(0);
    for (int i = 0; i < raster_byte_count; ++i) {
      model_input->data.int8[i] = raster_buffer[i];
    }

    if (interpreter->Invoke() != kTfLiteOk) {
      Serial.println("ERROR: Inference failed!");
      return;
    }
   
    TfLiteTensor* output = interpreter->output(0);
    const float scale = output->params.scale;
    const int zp = output->params.zero_point;
    
    float max_prob = 0.0f;
    int best = 0;
    
    for (int i = 0; i < label_count; ++i) {
      float p = scale * (static_cast<int>(output->data.int8[i]) - zp);
      if (p > 0.05f) {
        Serial.print(labels[i]);
        Serial.print(": ");
        Serial.print(p * 100, 1);
        Serial.print("%  ");
      }
      if (p > max_prob) {
        max_prob = p;
        best = i;
      }
    }
    Serial.println();

    unsigned long now = millis();
    if (max_prob >= 0.50f && (now - last_detection_ms) >= detection_cooldown_ms) {
      Serial.print("*** ");
      Serial.print(labels[best]);
      Serial.print(" (");
      Serial.print(max_prob * 100, 0);
      Serial.println("%) ***\n");
      last_detection_ms = now;
    } else {
      Serial.println("[Low confidence or cooldown]\n");
    }
  }
  
  delay(5);
}