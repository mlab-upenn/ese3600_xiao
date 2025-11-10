# Magic Wand

## Quick Start (3 Commands)

```bash
# 1. Run setup (downloads TensorFlow Lite)
./setup.sh

# 2. Edit these files:
#    - main/model_data.cc (paste your Colab model)
#    - main/constants.h (update gesture count & labels)

# 3. Build and upload
pio run --target upload --target monitor
```

## You Must Edit

1. **main/model_data.cc** - Replace with your trained model from Colab
2. **main/constants.h** - Update these lines:
   ```cpp
   constexpr int kGestureCount = 11;  // Your number
   const char* kGestureLabels[kGestureCount] = {
       "0", "1", "2", ...  // Your labels
   };
   ```

## If Setup Fails

Manual method:
```bash
git clone --recursive https://github.com/espressif/esp-tflite-micro.git components/esp-tflite-micro
pio run
```

## Hardware

- XIAO ESP32-S3 Sense
- Grove 9DOF IMU (ICM-20600)
- Connect: SDA→GPIO5, SCL→GPIO6
