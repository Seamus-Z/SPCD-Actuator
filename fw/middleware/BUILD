# Business services and their communication/persistence adapters.
# Middleware may compose device/HAL-facing code and pure algorithms; math and
# protocol remain dependency-free foundations, while Application only orchestrates.
COPTS = [
    "-mcpu=cortex-m4",
    "-mthumb",
    "-ffreestanding",
    "-fno-exceptions",
    "-fno-rtti",
    "-O2",
    "-g0",
    "-std=c++17",
]

cc_library(
    name = "runtime_config_model",
    hdrs = ["inc/middleware/config/runtime_config.h"],
    includes = ["inc"],
    copts = COPTS,
    visibility = ["//fw:__subpackages__"],
)

cc_library(
    name = "motor_calibration_store",
    hdrs = ["inc/middleware/persistence/motor_calibration_store.h"],
    includes = ["inc"],
    deps = [
        "//fw/math:base",
        "@com_github_ARMmbed_mbed-g4//:mbed",
    ],
    copts = COPTS,
    visibility = ["//fw:__subpackages__"],
)

cc_library(
    name = "runtime_config_store",
    hdrs = ["inc/middleware/persistence/runtime_config_store.h"],
    includes = ["inc"],
    deps = [
        ":runtime_config_model",
        "@com_github_ARMmbed_mbed-g4//:mbed",
    ],
    copts = COPTS,
    visibility = ["//fw:__subpackages__"],
)

cc_library(
    name = "encoder",
    hdrs = ["inc/middleware/encoder/encoder_service.h"],
    includes = ["inc"],
    deps = [
        "//fw/math:base",
        "//fw/math:servo_mode",
        "//fw/ports:ports",
    ],
    copts = COPTS,
    visibility = ["//fw:__subpackages__"],
)

cc_library(
    name = "calibration",
    hdrs = ["inc/middleware/calibration/calibration_manager.h"],
    includes = ["inc"],
    deps = [
        ":encoder",
        ":motor_calibration_store",
        "//fw/HAL:hal",
        "//fw/math:calibration",
        "//fw/math:foc",
        "//fw/math:servo_mode",
    ],
    copts = COPTS,
    visibility = ["//fw:__subpackages__"],
)

cc_library(
    name = "binary_link",
    srcs = ["src/communication/binary_link.cc"],
    hdrs = [
        "inc/middleware/communication/binary_link.h",
        "inc/middleware/communication/binary_link_policy.h",
    ],
    includes = ["inc"],
    deps = [
        "//fw/HAL:hal",
        "//fw/protocol:xt_can",
    ],
    copts = COPTS,
    visibility = ["//fw:__subpackages__"],
)

cc_test(
    name = "binary_link_policy_test",
    size = "small",
    srcs = [
        "test/binary_link_policy_test.cc",
        "inc/middleware/communication/binary_link_policy.h",
    ],
    includes = ["inc"],
    deps = [
        "//fw/protocol:xt_can",
        "@com_google_googletest//:gtest_main",
    ],
    copts = ["-std=c++17"],
)

cc_library(
    name = "snapshot",
    hdrs = ["inc/middleware/snapshot/snapshot_service.h"],
    includes = ["inc"],
    deps = [
        ":binary_link",
        "//fw/protocol:xt_can",
    ],
    copts = COPTS,
    visibility = ["//fw:__subpackages__"],
)

cc_library(
    name = "motor_control",
    srcs = ["src/control/motor_control_service.cc"],
    hdrs = ["inc/middleware/control/motor_control_service.h"],
    includes = ["inc"],
    deps = [
        ":binary_link",
        ":calibration",
        ":encoder",
        ":snapshot",
        "//fw/HAL:hal",
        "//fw/device:drv8353s",
        "//fw/math:base",
        "//fw/math:calibration",
        "//fw/math:foc",
        "//fw/math:servo_mode",
        "//fw/protocol:xt_can",
    ],
    copts = COPTS,
    visibility = ["//fw:__subpackages__"],
)

cc_library(
    name = "communication",
    srcs = [
        "src/communication/can_command_adapter.cc",
        "src/communication/telemetry_publisher.cc",
    ],
    hdrs = [
        "inc/middleware/communication/can_command_adapter.h",
        "inc/middleware/communication/telemetry_publisher.h",
    ],
    includes = ["inc"],
    deps = [
        ":binary_link",
        ":calibration",
        ":encoder",
        ":motor_control",
        ":runtime_config_model",
        ":snapshot",
        "//fw:app_interfaces",
        "//fw/HAL:hal",
        "//fw/math:base",
        "//fw/math:calibration",
        "//fw/protocol:xt_can",
    ],
    copts = COPTS,
    visibility = ["//fw:__subpackages__"],
)
