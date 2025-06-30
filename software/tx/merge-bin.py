#!/usr/bin/python3

# Adds PlatformIO post-processing to merge all the ESP flash images into a single image.

import os

Import("env", "projenv")

board_config = env.BoardConfig()

bootloader_offset = "0x0000"
bootloader_bin = "${BUILD_DIR}/bootloader.bin"
partitions_offset = "0x8000"
partitions_bin = "${BUILD_DIR}/partitions.bin"
boot_app0_offset = "0x9000"
boot_app0_bin = "${PACKAGES_DIR}/framework-arduinoespressif32/tools/partitions/boot_app0.bin"
firmware_offset = "0x10000"
firmware_bin = "${BUILD_DIR}/${PROGNAME}.bin"
merged_bin = os.environ.get("MERGED_BIN_PATH", "${BUILD_DIR}/${PROGNAME}-merged.bin")
spiffs_bin = "${BUILD_DIR}/spiffs.bin"
spiffs_offset = "0x20b000"

def merge_bin_action(source, target, env):
    flash_images = [
        bootloader_offset, 
        bootloader_bin,
        partitions_offset,
        partitions_bin,
        firmware_offset,
        firmware_bin,
        spiffs_offset,
        spiffs_bin,
    ]
    merge_cmd = " ".join(
        [
            '"$PYTHONEXE"',
            '"$OBJCOPY"',
            "--chip",
            board_config.get("build.mcu", "esp32"),
            "merge_bin",
            "-o",
            merged_bin,
            "--flash_mode",
            "dio",
            "--flash_freq",
            "80m",
            "--flash_size",
            "4MB",
            *flash_images,
        ]
    )
    env.Execute(merge_cmd)


env.AddCustomTarget(
    name="mergebin",
    dependencies=firmware_bin,
    actions=merge_bin_action,
    title="Merge binary",
    description="Build combined image",
    always_build=True,
)