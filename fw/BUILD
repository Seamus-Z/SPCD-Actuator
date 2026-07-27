# === Bare-metal compile options (shared by bootloader and app) ===
BAREMETAL_COPTS = [
    "-mcpu=cortex-m4",
    "-mthumb",
    "-ffreestanding",
    "-fno-exceptions",
    "-fno-rtti",
    "-O2",
    "-g0",
]

BAREMETAL_LINKOPTS = [
    "-nostartfiles",
    "-nostdlib",
    "-Wl,--gc-sections",
]

# === Bootloader ===
# Package root keeps only vectors.S + linker script; code under inc/ + src/.
cc_binary(
    name = "bootloader.elf",
    srcs = [
        "bootloader/src/bootloader.cc",
        "bootloader/src/BL_CanDriver.cc",
        "bootloader/src/BL_FlashWriter.cc",
        "bootloader/src/BL_CommandServer.cc",
        "bootloader/inc/BL_CanDriver.h",
        "bootloader/inc/BL_FlashWriter.h",
        "bootloader/inc/BL_CommandServer.h",
        "bootloader/inc/BL_Config.h",
        "bootloader/inc/boot_mjlib.h",
        "bootloader/inc/stm32g4xx_fdcan_typedefs.h",
        "bootloader/inc/bootloader.h",
        "bootloader/vectors.S",
    ],
    deps = ["@com_github_ARMmbed_mbed-g4//:mbed"],
    includes = ["bootloader/inc"],
    copts = BAREMETAL_COPTS + ["-std=c++17"],
    linkopts = BAREMETAL_LINKOPTS + ["-T$(location bootloader/stm32g474_bootloader.ld)"],
    additional_linker_inputs = ["bootloader/stm32g474_bootloader.ld"],
)

genrule(
    name = "app_binary",
    srcs = ["app.elf"],
    outs = ["app.bin"],
    cmd = "$(OBJCOPY) -O binary $(location app.elf) $@",
    toolchains = ["@bazel_tools//tools/cpp:current_cc_toolchain"],
)

# === Application ===
# main.cpp holds the Application state machine + CRT entry.
cc_binary(
    name = "app.elf",
    srcs = [
        "app/main.cpp",
        "app/app_vectors.S",
        "bootloader/src/enter_bootloader.cc",
        "bootloader/inc/bootloader.h",
    ],
    deps = [
        "//fw/HAL:hal",
        "@com_github_ARMmbed_mbed-g4//:mbed",
    ],
    includes = [
        ".",
        "bootloader/inc",
    ],
    copts = BAREMETAL_COPTS + ["-std=c++17"],
    linkopts = BAREMETAL_LINKOPTS + ["-T$(location app/stm32g474_app.ld)"],
    additional_linker_inputs = ["app/stm32g474_app.ld"],
)

genrule(
    name = "xtellar.combined",
    srcs = ["bootloader.elf", "app_binary"],
    outs = ["xtellar.combined.bin"],
    cmd = (
        "$(OBJCOPY) -O binary $(location bootloader.elf) $(@D)/bl_full.bin && " +
        "python3 -c \"import os,sys;s=os.path.getsize('$(@D)/bl_full.bin');" +
        "sys.exit('bootloader binary too big (%d > 65536)' % s) if s > 65536 else 0\" && " +
        "cp $(@D)/bl_full.bin $@ && " +
        "python3 -c \"import os;f=open('$@','ab');s=os.fstat(f.fileno()).st_size;" +
        "f.truncate(max(s,65536))\" && " +
        "cat $(location app_binary) >> $@"
    ),
    toolchains = ["@bazel_tools//tools/cpp:current_cc_toolchain"],
)
