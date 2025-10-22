# Micro Speech Example

This example shows how to run a 20 kB model that can recognize 2 keywords,
"yes" and "no", from speech data.

The application listens to its surroundings with a microphone and indicates
when it has detected a word by displaying data in the serial monitor.

## Deploy to ESP32

The following instructions will help you build and deploy this sample using 
the [ESP IDF](https://github.com/espressif/esp-idf) framework in [PlatformIO]
(https://docs.platformio.org/en/latest//boards/espressif32/seeed_xiao_esp32s3.html).

### Building the example

In this directory, first run:
$ git clone https://github.com/espressif/esp-tflite-micro.git

Then reload your VS-Code window to let PlatformIO properly configure your project workspace.

Build the project.
You will see an error saying the wav files have not compiled. You must manually do this with:
$ /Users/yourname/.platformio/packages/tool-ninja/ninja -C .pio/build/seeed_xiao_esp32s3 \
  no_30ms.wav.S no_1000ms.wav.S noise_1000ms.wav.S \
  silence_1000ms.wav.S yes_30ms.wav.S yes_1000ms.wav.S

Then, without cleaning, press build again. This should finish building the project.

### Load and run the example

You can now flash and monitor and speak into the microphone with "yes" or "no" keywords.

### Sample output

  * When a keyword is detected you will see following output sample output on the log screen:

```
Heard yes (<score>) at <time>
```
