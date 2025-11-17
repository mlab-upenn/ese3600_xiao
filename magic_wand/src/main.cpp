/* Magic Wand - ICM-20600
 * HTML-style IMU -> 2D stroke -> 32x32 raster
 * Matches the browser code for integration (gy/gz), but
 * NORMALIZES AFTER THE GESTURE with a margin so curves
 * aren't trimmed at +/-0.6 in the raster.
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

const int   VERSION                     = 0x00000000;
constexpr float kDefaultImuSampleRateHz = 104.0f;

constexpr int stroke_transmit_stride      = 2;
constexpr int stroke_transmit_max_length  = 160;
constexpr int stroke_points_byte_count    = 2 * sizeof(int8_t) * stroke_transmit_max_length;
constexpr int stroke_struct_byte_count    = (2 * sizeof(int32_t)) + stroke_points_byte_count;

constexpr int raster_width    = 32;
constexpr int raster_height   = 32;
constexpr int raster_channels = 3;
constexpr int raster_byte_count = raster_height * raster_width * raster_channels;
int8_t raster_buffer[raster_byte_count];

// ===== Stroke points in float coords ([-0.6,0.6]) =====
struct StrokePointF {
  float x;
  float y;
};

StrokePointF stroke_points_f[stroke_transmit_max_length];
int32_t      stroke_length_f = 0;

// ===== RAW (un-normalized) coords from yaw/pitch integration =====
//float  raw_x[stroke_transmit_max_length];
//float  raw_y[stroke_transmit_max_length];
//int32_t raw_length = 0;

// BLE + state
bool capture_enabled = false;   // Recording between 'r' and 's'
bool manual_done     = false;   // User pressed 's' to finish gesture

BLEService        service              (BLE_SENSE_UUID("0000"));
BLECharacteristic strokeCharacteristic (BLE_SENSE_UUID("300a"), BLERead, stroke_struct_byte_count);

String name;

// Raw IMU buffers (kept mostly for compatibility / debug)
constexpr int acceleration_data_length = 300 * 3;
float acceleration_data[acceleration_data_length] = {};
int   acceleration_data_index = 0;
float acceleration_sample_rate = kDefaultImuSampleRateHz;

constexpr int gyroscope_data_length = 300 * 3;
float gyroscope_data[gyroscope_data_length]   = {};
int   gyroscope_data_index = 0;
float gyroscope_sample_rate = kDefaultImuSampleRateHz;

float current_velocity[3]        = {0.0f, 0.0f, 0.0f};
float current_position[3]        = {0.0f, 0.0f, 0.0f};
float current_gravity[3]         = {0.0f, 0.0f, 0.0f};
float current_gyroscope_drift[3] = {0.0f, 0.0f, 0.0f};

int32_t stroke_length = 0;  // not used for stroke building now
uint8_t stroke_struct_buffer[stroke_struct_byte_count] = {};
int32_t* stroke_state           = reinterpret_cast<int32_t*>(stroke_struct_buffer);
int32_t* stroke_transmit_length = reinterpret_cast<int32_t*>(stroke_struct_buffer + sizeof(int32_t));
int8_t*  stroke_points          = reinterpret_cast<int8_t*>(stroke_struct_buffer + (sizeof(int32_t) * 2));

// TensorFlow Lite Micro
constexpr int kTensorArenaSize = 25 * 1024;
alignas(16) uint8_t tensor_arena[kTensorArenaSize];

const tflite::Model*         model       = nullptr;
tflite::MicroInterpreter*    interpreter = nullptr;

constexpr int label_count = 10;
const char* labels[label_count] = {"0","1","2","3","4","5","6","7","8","9"};

// ======== HTML-STYLE INTEGRATOR CONSTANTS =========
constexpr float kSampleDtSec   = 0.025f;  // SAMPLE_DT
constexpr int   kDecimateN     = 4;       // DECIMATE_N
constexpr float kAngleNormDeg  = 90.0f;   // ANGLE_NORM
constexpr float kCoordScale    = 0.6f;    // final coord range [-0.6,0.6]

float yawDeg   = 0.0f;   // from gz
float pitchDeg = 0.0f;   // from gy
int   sample_counter = 0;

// --------------------------------------------------------------------
// Reset yaw/pitch integrator + stroke buffers
// --------------------------------------------------------------------

void ResetStrokeAndIntegrator() {
  yawDeg         = 0.0f;
  pitchDeg       = 0.0f;
  sample_counter = 0;

  stroke_length_f        = 0;
  *stroke_transmit_length = 0;
}

// --------------------------------------------------------------------
// Integrate gyro into raw_x/raw_y (NO CLAMPING / SCALING YET)
// This matches the JS pipeline except we delay normalization.
// --------------------------------------------------------------------

void UpdateStrokeFromGyroSample(float gy_dps, float gz_dps) {
  if (!capture_enabled) return;
  if (stroke_length_f >= stroke_transmit_max_length) return;

  // integrate gyro → yaw/pitch (degrees)
  yawDeg   += gz_dps * kSampleDtSec;
  pitchDeg += gy_dps * kSampleDtSec;
  sample_counter++;

  // decimate like in HTML
  if ((sample_counter % kDecimateN) != 0) {
    return;
  }

  // ---- same math as HTML / browser ----
  // 1) normalize by 90°
  float x = yawDeg   / kAngleNormDeg;
  float y = pitchDeg / kAngleNormDeg;

  // 2) clamp to [-1, 1]
  if (x >  1.0f) x =  1.0f;
  if (x < -1.0f) x = -1.0f;
  if (y >  1.0f) y =  1.0f;
  if (y < -1.0f) y = -1.0f;

  // 3) shrink into Pete range [-0.6, 0.6]
  x *= kCoordScale;   // kCoordScale = 0.6f
  y *= kCoordScale;

  // 4) orientation: yaw → x,  -pitch → y  (same as HTML)
  float xr =  x;
  float yr = -y;

  // ---- store float stroke points ([-0.6, 0.6]) ----
  stroke_points_f[stroke_length_f].x = xr;
  stroke_points_f[stroke_length_f].y = yr;

  // ---- encode into int8 for RasterizeStroke ([-1,1] -> [-128,127]) ----
  float rx = xr / kCoordScale;   // back to [-1,1]
  float ry = yr / kCoordScale;

  if (rx >  1.0f) rx =  1.0f;
  if (rx < -1.0f) rx = -1.0f;
  if (ry >  1.0f) ry =  1.0f;
  if (ry < -1.0f) ry = -1.0f;

  int32_t val_x = static_cast<int32_t>(roundf(rx * 128.0f));
  if (val_x > 127)  val_x = 127;
  if (val_x < -128) val_x = -128;

  int32_t val_y = static_cast<int32_t>(roundf(ry * 128.0f));
  if (val_y > 127)  val_y = 127;
  if (val_y < -128) val_y = -128;

  stroke_points[2 * stroke_length_f]     = static_cast<int8_t>(val_x);
  stroke_points[2 * stroke_length_f + 1] = static_cast<int8_t>(val_y);

  // update lengths
  stroke_length_f++;
  *stroke_transmit_length = stroke_length_f;
}

// --------------------------------------------------------------------
// IMU reading: fills accel + gyro buffers and ALSO updates raw stroke
// --------------------------------------------------------------------

void ReadAccelerometerAndGyroscope(int* new_accelerometer_samples, int* new_gyroscope_samples) {
  *new_accelerometer_samples = 0;
  *new_gyroscope_samples     = 0;

  float accel_sample[3];
  float gyro_sample[3];
  if (!ReadIMU(accel_sample, gyro_sample)) {
    return;
  }

  // Store gyro in buffer (optional; kept for compatibility)
  const int gyroscope_index = (gyroscope_data_index % gyroscope_data_length);
  gyroscope_data[gyroscope_index]     = gyro_sample[0];
  gyroscope_data[gyroscope_index + 1] = gyro_sample[1];
  gyroscope_data[gyroscope_index + 2] = gyro_sample[2];
  gyroscope_data_index += 3;
  *new_gyroscope_samples = 1;

  // Store accel
  const int acceleration_index = (acceleration_data_index % acceleration_data_length);
  acceleration_data[acceleration_index]     = accel_sample[0];
  acceleration_data[acceleration_index + 1] = accel_sample[1];
  acceleration_data[acceleration_index + 2] = accel_sample[2];
  acceleration_data_index += 3;
  *new_accelerometer_samples = 1;

  // HTML-style stroke building from gyro only
  UpdateStrokeFromGyroSample(gyro_sample[1], gyro_sample[2]);  // gy, gz
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

// Drift = 0 (as before)
void EstimateGyroscopeDrift(float* drift) {
  drift[0] = 0.0f;
  drift[1] = 0.0f;
  drift[2] = 0.0f;
}

// Orientation integration is not used for stroke, kept only if you want it
void UpdateOrientation(int new_samples, float* /*gravity*/, float* drift) {
  const float deg_to_rad = 0.017453292519943295f;  // PI/180
  const float dt         = 1.0f / gyroscope_sample_rate;

  const int start_index = ((gyroscope_data_index +
    (gyroscope_data_length - (3 * (new_samples + 1)))) %
    gyroscope_data_length);

  for (int i = 0; i < new_samples; ++i) {
    const int index = ((start_index + (i * 3)) % gyroscope_data_length);
    const float* gyro_entry = &gyroscope_data[index];

    const float gx = (gyro_entry[0] - drift[0]) * deg_to_rad;
    const float gy = (gyro_entry[1] - drift[1]) * deg_to_rad;
    const float gz = (gyro_entry[2] - drift[2]) * deg_to_rad;

    current_position[0] += gx * dt;
    current_position[1] += gy * dt;
    current_position[2] += gz * dt;
  }
}

// ===== Print strokePoints as wanddata-like JSON =====
void PrintStrokeAsJson(int index, const char* label) {
  Serial.println("{");
  Serial.println("  \"strokes\": [");
  Serial.println("    {");
  Serial.print("      \"index\": ");
  Serial.print(index);
  Serial.println(",");
  Serial.print("      \"label\": \"");
  Serial.print(label);
  Serial.println("\",");

  Serial.println("      \"strokePoints\": [");
  for (int i = 0; i < stroke_length_f; ++i) {
    float x = stroke_points_f[i].x;  // [-0.6, 0.6]
    float y = stroke_points_f[i].y;  // [-0.6, 0.6]
    Serial.print("        {\"x\": ");
    Serial.print(x, 6);
    Serial.print(", \"y\": ");
    Serial.print(y, 6);
    Serial.print("}");
    if (i != stroke_length_f - 1) Serial.println(",");
    else Serial.println();
  }
  Serial.println("      ]");
  Serial.println("    }");
  Serial.println("  ]");
  Serial.println("}");
}

}  // namespace

// ====================================================================
// setup()
// ====================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  tflite::InitializeTarget();

  Serial.println("\n========================================");
  Serial.println("  Magic Wand  ");
  Serial.println("  For ICM-20600 (HTML-style, post-normalized)");
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
    Serial.println("ERROR: Model version mismatch!");
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
  Serial.println("Press 'r' to start, 's' to stop & classify");
  Serial.println("========================================\n");
}

// ====================================================================
// loop()
// ====================================================================
void loop() {
  // Serial commands:
  //   'r' or 'R' -> start capture for one gesture
  //   's' or 'S' -> stop capture and process gesture
  if (Serial.available() > 0) {
    char c = Serial.read();

    if (c == 'r' || c == 'R') {
      capture_enabled = true;
      manual_done     = false;

      *stroke_state = 0;
      stroke_length = 0;
      ResetStrokeAndIntegrator();

      acceleration_data_index = 0;
      gyroscope_data_index    = 0;
      memset(acceleration_data, 0, sizeof(acceleration_data));
      memset(gyroscope_data,    0, sizeof(gyroscope_data));

      current_velocity[0] = current_velocity[1] = current_velocity[2] = 0.0f;
      current_position[0] = current_position[1] = current_position[2] = 0.0f;
      current_gravity[0]  = current_gravity[1]  = current_gravity[2]  = 0.0f;
      current_gyroscope_drift[0] = current_gyroscope_drift[1] = current_gyroscope_drift[2] = 0.0f;

      Serial.println("\n[Capture ON - draw a digit now]");
    } else if (c == 's' || c == 'S') {
      capture_enabled = false;
      manual_done     = true;
      Serial.println("\n[Capture STOP - processing gesture]");
    }
  }

  BLEDevice central = BLE.central();
  
  int accel_samples = 0, gyro_samples = 0;
  ReadAccelerometerAndGyroscope(&accel_samples, &gyro_samples);

  bool done = false;

  if (gyro_samples > 0) {
    EstimateGyroscopeDrift(current_gyroscope_drift);
    UpdateOrientation(gyro_samples, current_gravity, current_gyroscope_drift);
  }

  if (accel_samples > 0) {
    EstimateGravityDirection(current_gravity);
    UpdateVelocity(accel_samples, current_gravity);
  }

  // If user pressed 's', finish gesture by normalizing

  // If user pressed 's', finish gesture (no extra normalization)
  if (manual_done && !done) {
    manual_done = false;

    if (stroke_length_f > 4) {  // require a few points
      done = true;
    } else if (stroke_length_f == 0) {
      Serial.println("No gesture recorded; nothing to process.");
      ResetStrokeAndIntegrator();
    } else {
      Serial.println("Gesture too small / flat; discarded.");
      ResetStrokeAndIntegrator();
    }
  }

  // Single-shot classification per gesture
  if (done) {
    Serial.println("\n>>> GESTURE <<<");

    // Print JSON for Colab
    PrintStrokeAsJson(/*index=*/0, /*label=*/"?");

    // Rasterize from int8 stroke_points
    RasterizeStroke(
      stroke_points,
      *stroke_transmit_length,
      1.0f, 1.0f,           // use full normalized range [-1, 1]
      raster_width,
      raster_height,
      raster_buffer);
    
    // ASCII visualization of 32x32
    for (int y = 0; y < raster_height; ++y) {
      for (int x = 0; x < raster_width; ++x) {
        const int8_t* pixel =
            &raster_buffer[(y * raster_width * raster_channels) + (x * raster_channels)];
        Serial.print((pixel[0] > -128 || pixel[1] > -128 || pixel[2] > -128) ? '#' : '.');
      }
      Serial.println();
    }
    
    // Copy into model input
    // -------- Copy into model input with proper quantization --------
    TfLiteTensor* model_input = interpreter->input(0);

    const float input_scale = model_input->params.scale;
    const int   input_zp    = model_input->params.zero_point;

    // Optional: debug once
    // Serial.print("input_scale = "); Serial.println(input_scale, 6);
    // Serial.print("input_zp    = "); Serial.println(input_zp);

    for (int i = 0; i < raster_byte_count; ++i) {
      // raster_buffer is int8: background ~ -128, stroke up to ~127
      int s = static_cast<int>(raster_buffer[i]);   // [-128,127]
      uint8_t u = static_cast<uint8_t>(s + 128);    // [0,255] like Colab PNGs

      // Quantize float(0–255) -> int8: q = round(f/scale) + zp
      float   f = static_cast<float>(u);
      int32_t q = static_cast<int32_t>(roundf(f / input_scale)) + input_zp;

      if (q < -128) q = -128;
      if (q > 127)  q = 127;

      model_input->data.int8[i] = static_cast<int8_t>(q);
    }


    if (interpreter->Invoke() != kTfLiteOk) {
      Serial.println("ERROR: Inference failed!");
      return;
    }
   
    TfLiteTensor* output = interpreter->output(0);
    const float scale = output->params.scale;
    const int   zp    = output->params.zero_point;

    // Debug: print output tensor shape
    Serial.print("output dims: ");
    for (int i = 0; i < output->dims->size; ++i) {
      Serial.print(output->dims->data[i]);
      Serial.print(" ");
    }
    Serial.println();

    
    // ---- Find top-2 probabilities ----
    float max_prob    = -1.0f;
    float second_prob = -1.0f;
    int   best        = -1;

    for (int i = 0; i < label_count; ++i) {
      // Dequantize: int8 -> float probability
      float p = scale * (static_cast<int>(output->data.int8[i]) - zp);

      // Print only reasonably large probabilities
      if (p > 0.05f) {
        Serial.print(labels[i]);
        Serial.print(": ");
        Serial.print(p * 100, 1);
        Serial.print("%  ");
      }

      // Track top-2
      if (p > max_prob) {
        second_prob = max_prob;
        max_prob    = p;
        best        = i;
      } else if (p > second_prob) {
        second_prob = p;
      }
    }
    Serial.println();

    // ---------- UNKNOWN LOGIC ----------
    const float kMinConfidence = 0.35f;  // minimum absolute confidence
    const float kMinMargin     = 0.08f;  // how much top must beat #2

    float margin      = max_prob - second_prob;
    bool  is_confident = (best >= 0) &&
                         (max_prob >= kMinConfidence) &&
                         (margin   >= kMinMargin);

    if (!is_confident) {
      Serial.print("Best guess: UNKNOWN");
      Serial.print(" (max prob = ");
      Serial.print(max_prob * 100, 1);
      Serial.println("%)");
    } else {
      Serial.print("Best guess: ");
      Serial.print(labels[best]);
      Serial.print(" (");
      Serial.print(max_prob * 100, 1);
      Serial.println("%)");
    }
    Serial.println();


    // Reset state; wait for next 'r'
    capture_enabled         = false;
    *stroke_state           = 0;
    stroke_length           = 0;
    ResetStrokeAndIntegrator();
    Serial.println("[Ready - press 'r' to record another gesture]");
  }
  
  delay(5);
}
