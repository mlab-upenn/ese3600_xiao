#!/bin/bash

echo "=================================================="
echo "Magic Wand - Automatic Setup for Tensor Flow ESP32"
echo "=================================================="
echo ""

# Clone esp-tflite-micro
echo "Downloading TensorFlow Lite for ESP32..."
git clone --recursive https://github.com/espressif/esp-tflite-micro.git components/esp-tflite-micro

if [ -d "components/esp-tflite-micro" ]; then
    echo "TensorFlow Lite installed successfully!"
    echo ""
    echo "========================================="
    echo "Setup Complete!"
    echo "========================================="
    echo ""
    echo "Next steps:"
    echo "1. Replace main/model_data.cc with your Colab model"
    echo "2. Update main/constants.h (gesture count & labels)"
    echo "3. Run: pio run --target upload --target monitor"
    echo ""
else
    echo "Failed to install TensorFlow Lite"
    echo "Please run manually:"
    echo "  git clone --recursive https://github.com/espressif/esp-tflite-micro.git components/esp-tflite-micro"
    exit 1
fi
