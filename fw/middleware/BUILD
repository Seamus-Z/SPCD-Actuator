# Business services: compose device drivers, pure algorithms, and persistence.
# Middleware is the only layer allowed to depend on both device/HAL-facing code
# and math. Application orchestrates middleware; math remains dependency-free.
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
    name = "encoder",
    hdrs = ["inc/middleware/encoder/encoder_service.h"],
    includes = ["inc"],
    deps = [
        "//fw/HAL:hal",
        "//fw/device:ma600",
        "//fw/math:base",
        "//fw/math:servo_mode",
        "//fw/pool:pool",
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
        "//fw/HAL:hal",
        "//fw/math:calibration",
        "//fw/math:foc",
        "//fw/math:servo_mode",
        "//fw/nvs:encoder_cal_store",
    ],
    copts = COPTS,
    visibility = ["//fw:__subpackages__"],
)
