#include "output_handler.h"
#include "constants.h"
#include "esp_log.h"
#include <cstdio>

static const char* TAG = "OutputHandler";

// Convert RGB image to ASCII art
static void ImageToASCII(const uint8_t* image, char* ascii_buffer) {
    // ASCII characters for different brightness levels (dark to bright)
    const char* ascii_chars = " .:-=+*#%@";
    const int num_chars = 10;
    
    for (int y = 0; y < kAsciiHeight; y++) {
        for (int x = 0; x < kAsciiWidth; x++) {
            // Get RGB values
            int idx = (y * kImageWidth + x) * 3;
            uint8_t r = image[idx + 0];
            uint8_t g = image[idx + 1];
            uint8_t b = image[idx + 2];
            
            // Convert to grayscale (luminance)
            int gray = (r * 299 + g * 587 + b * 114) / 1000;
            
            // Invert (dark strokes on white background -> light ASCII on dark terminal)
            gray = 255 - gray;
            
            // Map to ASCII character
            int char_index = (gray * (num_chars - 1)) / 255;
            char_index = (char_index < 0) ? 0 : (char_index >= num_chars ? num_chars - 1 : char_index);
            
            ascii_buffer[y * kAsciiWidth + x] = ascii_chars[char_index];
        }
    }
}

void HandleOutput(const uint8_t* image, const char* gesture_label, int8_t confidence) {
    // Create ASCII art buffer
    char ascii_buffer[kAsciiWidth * kAsciiHeight + 1];
    ascii_buffer[kAsciiWidth * kAsciiHeight] = '\0';
    
    ImageToASCII(image, ascii_buffer);
    
    // Print ASCII art (dots/hash pattern)
    for (int y = 0; y < kAsciiHeight; y++) {
        for (int x = 0; x < kAsciiWidth; x++) {
            printf("%c", ascii_buffer[y * kAsciiWidth + x]);
        }
        printf("\n");
    }
    
    // Print prediction result in "Found X (confidence)" format
    printf("\nFound %s (%d)\n\n", gesture_label, confidence);
    
    // Log to ESP_LOG as well
    ESP_LOGI(TAG, "Gesture: %s, Confidence: %d", gesture_label, confidence);
}