Import("env")

from os import path
from SCons.Script import Builder

WAV_FILES = [
    "no_30ms.wav",
    "no_1000ms.wav",
    "noise_1000ms.wav",
    "silence_1000ms.wav",
    "yes_30ms.wav",
    "yes_1000ms.wav",
]


def _ensure_file_to_asm_builder(env):
    if "FileToAsm" in env.get("BUILDERS", {}):
        return

    tool_cmake = env.PioPlatform().get_package_dir("tool-cmake") or ""
    cmake_binary = path.join(tool_cmake, "bin", "cmake")
    framework_dir = env.PioPlatform().get_package_dir("framework-espidf") or ""
    data_embed_script = path.join(
        framework_dir,
        "tools",
        "cmake",
        "scripts",
        "data_file_embed_asm.cmake",
    )

    def transform_to_asm(target, source, env):
        files = [path.join("$BUILD_DIR", src.name + ".S") for src in source]
        return files, source

    env.Append(
        BUILDERS=dict(
            FileToAsm=Builder(
                action=env.VerboseAction(
                    " ".join(
                        [
                            cmake_binary,
                            "-DDATA_FILE=$SOURCE",
                            "-DSOURCE_FILE=$TARGET",
                            "-DFILE_TYPE=$FILE_TYPE",
                            "-P",
                            data_embed_script,
                        ]
                    ),
                    "Generating assembly for $TARGET",
                ),
                emitter=transform_to_asm,
                single_source=True,
            )
        )
    )


_ensure_file_to_asm_builder(env)

wav_sources = [path.join("$PROJECT_DIR", "test_data", filename) for filename in WAV_FILES]

generated = env.FileToAsm(wav_sources, FILE_TYPE="BINARY")

env.Requires(path.join("$BUILD_DIR", "${PROGNAME}.elf"), generated)

"""
Generate the assembly stubs that embed the WAV assets using the same CMake
script ESP-IDF employs internally. This mirrors the original `ninja` targets
so SCons can build the assets directly during the PlatformIO build.
"""
