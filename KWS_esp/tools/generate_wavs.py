Import("env")

from os import path

WAV_TARGETS = [
    "no_30ms.wav.S",
    "no_1000ms.wav.S",
    "noise_1000ms.wav.S",
    "silence_1000ms.wav.S",
    "yes_30ms.wav.S",
    "yes_1000ms.wav.S",
]

def generate_wavs(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    ninja_build_file = path.join(build_dir, "build.ninja")
    # Skip on the first configure pass, before ninja files exist.
    if not path.exists(ninja_build_file):
        return

    ninja = path.join(env.get("PROJECT_PACKAGES_DIR"), "tool-ninja", "ninja")
    cmd = [ninja, "-C", build_dir] + WAV_TARGETS
    env.Execute(env.VerboseAction(" ".join(cmd), "Embedding WAV assets"))

env.AddPreAction("buildprog", generate_wavs)


"""
replaces:
/Users/aidanmcgoogan/.platformio/packages/tool-ninja/ninja -C .pio/build/seeed_xiao_esp32s3 \
  no_30ms.wav.S no_1000ms.wav.S noise_1000ms.wav.S \
  silence_1000ms.wav.S yes_30ms.wav.S yes_1000ms.wav.S
"""