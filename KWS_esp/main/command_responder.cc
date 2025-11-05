/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "command_responder.h"
#include "tensorflow/lite/micro/micro_log.h"

// If OLED is used
#include "oled_display.h"       
#include <cstring>               

// The default implementation writes out the name of the recognized command
// to the error console and displays it on the OLED screen.
// Real applications will want to take some custom action based on commands.
void RespondToCommand(int32_t current_time, const char* found_command,
                      float score, bool is_new_command) {
  if (is_new_command) {
    // Log to console
    MicroPrintf("Heard %s (%.4f) @%dms", found_command, score, current_time);
    
    // Uncomment the following to display on OLED screen
    oled_display_command(found_command, (uint8_t)(score * 100));
  }
}
