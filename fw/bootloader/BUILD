BAREMETAL_COPTS = [
    "-mcpu=cortex-m4",
    "-mthumb",
    "-ffreestanding",
    "-fno-exceptions",
    "-fno-rtti",
    "-O2",
    "-g0",
    "-std=c++17",
]

BAREMETAL_LINKOPTS = [
    "-nostartfiles",
    "-nostdlib",
    "-Wl,--gc-sections",
]

# Shared headers (multiplex helpers / memory map) for APP telemetry.
cc_library(
    name = "boot_headers",
    hdrs = [
        "inc/boot_mjlib.h",
        "inc/BL_Config.h",
        "inc/bootloader.h",
    ],
    includes = ["inc"],
    visibility = ["//visibility:public"],
)

# Soft-reset entry used by the application image.
cc_library(
    name = "enter_bootloader",
    srcs = ["src/enter_bootloader.cc"],
    deps = [
        ":boot_headers",
        "@com_github_ARMmbed_mbed-g4//:mbed",
    ],
    copts = BAREMETAL_COPTS,
    visibility = ["//fw:__pkg__"],
)

cc_binary(
    name = "bootloader.elf",
    srcs = [
        "src/bootloader.cc",
        "src/BL_CanDriver.cc",
        "src/BL_FlashWriter.cc",
        "src/BL_CommandServer.cc",
        "inc/BL_CanDriver.h",
        "inc/BL_FlashWriter.h",
        "inc/BL_CommandServer.h",
        "inc/BL_Config.h",
        "inc/boot_mjlib.h",
        "inc/stm32g4xx_fdcan_typedefs.h",
        "inc/bootloader.h",
        "vectors.S",
    ],
    deps = ["@com_github_ARMmbed_mbed-g4//:mbed"],
    includes = ["inc"],
    copts = BAREMETAL_COPTS,
    linkopts = BAREMETAL_LINKOPTS + ["-T$(location stm32g474_bootloader.ld)"],
    additional_linker_inputs = ["stm32g474_bootloader.ld"],
    visibility = ["//fw:__pkg__"],
)
