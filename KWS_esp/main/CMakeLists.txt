
#
# Main component of TF Micro project 'micro_speech'.
#

idf_component_register(
  SRCS "main.cc" "main_functions.cc" "audio_provider.cc" "feature_provider.cc"
       "recognize_commands.cc" "command_responder.cc" "model.cc" "ringbuf.c"
  INCLUDE_DIRS "."
  REQUIRES esp-tflite-micro
)
# Optional acceleration:
target_compile_definitions(${COMPONENT_TARGET} PRIVATE ESP_NN)
