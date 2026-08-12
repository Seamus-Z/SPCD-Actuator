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

# Re-export bootloader ELF at //fw for existing flash scripts.
alias(
    name = "bootloader.elf",
    actual = "//fw/bootloader:bootloader.elf",
)

genrule(
    name = "app_binary",
    srcs = ["app.elf"],
    outs = ["app.bin"],
    cmd = "$(OBJCOPY) -O binary $(location app.elf) $@",
    toolchains = ["@bazel_tools//tools/cpp:current_cc_toolchain"],
)

# === Application service boundaries ===
cc_library(
    name = "app_interfaces",
    hdrs = [
        "app/inc/application_ports.h",
        "app/inc/runtime_config_service.h",
    ],
    includes = ["app/inc"],
    visibility = [
        "//fw:__pkg__",
        "//fw:__subpackages__",
    ],
    deps = ["//fw/middleware:runtime_config_model"],
)

cc_library(
    name = "application_core",
    srcs = ["app/src/application.cc"],
    hdrs = [
        "app/inc/application.h",
        "app/inc/application_policy.h",
        "app/inc/app_state.h",
    ],
    includes = ["app/inc"],
    deps = [
        ":app_interfaces",
        "//fw/middleware:calibration",
        "//fw/middleware:encoder",
        "//fw/middleware:motor_control",
    ],
    copts = BAREMETAL_COPTS + ["-std=c++17"],
    visibility = ["//fw:__pkg__"],
)

cc_test(
    name = "application_policy_test",
    size = "small",
    srcs = [
        "app/test/application_policy_test.cc",
        "app/inc/application_policy.h",
        "app/inc/app_state.h",
    ],
    includes = ["app/inc"],
    deps = ["@com_google_googletest//:gtest_main"],
    copts = ["-std=c++17"],
)

cc_library(
    name = "platform_service",
    srcs = ["board/xtellar_stm32g4/platform_service.cc"],
    hdrs = ["board/xtellar_stm32g4/platform_service.h"],
    includes = [".", "app/inc"],
    deps = [
        ":app_interfaces",
        "//fw/HAL:hal",
        "//fw/bootloader:enter_bootloader",
        "//fw/device:drv8353s",
        "@com_github_ARMmbed_mbed-g4//:mbed",
    ],
    copts = BAREMETAL_COPTS + ["-std=c++17"],
    visibility = ["//fw:__pkg__"],
)

cc_library(
    name = "runtime_config_service",
    srcs = ["board/xtellar_stm32g4/runtime_config_service.cc"],
    hdrs = [
        "board/xtellar_stm32g4/board_config.h",
        "board/xtellar_stm32g4/runtime_config_service.h",
    ],
    includes = [".", "app/inc"],
    deps = [
        ":app_interfaces",
        "//fw/HAL:hal",
        "//fw/device:drv8353s",
        "//fw/device:ma600",
        "//fw/device:motor",
        "//fw/math:base",
        "//fw/math:foc",
        "//fw/math:servo_mode",
        "//fw/middleware:calibration",
        "//fw/middleware:encoder",
        "//fw/middleware:motor_control",
        "//fw/middleware:motor_calibration_store",
        "//fw/middleware:runtime_config_store",
    ],
    copts = BAREMETAL_COPTS + ["-std=c++17"],
    visibility = ["//fw:__pkg__"],
)

cc_library(
    name = "firmware_composition",
    srcs = ["board/xtellar_stm32g4/firmware_composition.cc"],
    hdrs = [
        "board/xtellar_stm32g4/board_config.h",
        "board/xtellar_stm32g4/firmware_composition.h",
    ],
    includes = [".", "app/inc"],
    deps = [
        ":application_core",
        ":platform_service",
        ":runtime_config_service",
        "//fw/HAL:hal",
        "//fw/device:drv8353s",
        "//fw/device:ma600",
        "//fw/device:motor",
        "//fw/math:base",
        "//fw/math:foc",
        "//fw/math:servo_mode",
        "//fw/middleware:calibration",
        "//fw/middleware:communication",
        "//fw/middleware:encoder",
        "//fw/middleware:motor_control",
        "//fw/middleware:snapshot",
        "//fw/core:memory_pool",
        "//fw/middleware:binary_link",
    ],
    copts = BAREMETAL_COPTS + ["-std=c++17"],
    visibility = ["//fw:__pkg__"],
)

# === Firmware image ===
cc_binary(
    name = "app.elf",
    srcs = [
        "app/main.cpp",
        "app/app_vectors.S",
    ],
    deps = [
        ":firmware_composition",
        "//fw/HAL:hal",
        "//fw/core:memory_pool",
        "@com_github_ARMmbed_mbed-g4//:mbed",
    ],
    includes = ["."],
    copts = BAREMETAL_COPTS + ["-std=c++17"],
    linkopts = BAREMETAL_LINKOPTS + ["-T$(location app/stm32g474_app.ld)"],
    additional_linker_inputs = ["app/stm32g474_app.ld"],
)

genrule(
    name = "xtellar.combined",
    srcs = ["//fw/bootloader:bootloader.elf", "app_binary"],
    outs = ["xtellar.combined.bin"],
    cmd = (
        "$(OBJCOPY) -O binary $(location //fw/bootloader:bootloader.elf) $(@D)/bl_full.bin && " +
        "python3 -c \"import os,sys;s=os.path.getsize('$(@D)/bl_full.bin');" +
        "sys.exit('bootloader binary too big (%d > 65536)' % s) if s > 65536 else 0\" && " +
        "cp $(@D)/bl_full.bin $@ && " +
        "python3 -c \"import os;f=open('$@','ab');s=os.fstat(f.fileno()).st_size;" +
        "f.truncate(max(s,65536))\" && " +
        "cat $(location app_binary) >> $@"
    ),
    toolchains = ["@bazel_tools//tools/cpp:current_cc_toolchain"],
)
