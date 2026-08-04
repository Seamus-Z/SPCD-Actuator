# Motor / encoder calibration routines (bring-up and production).
cc_library(
    name = "encoder_phase",
    hdrs = ["inc/calibration/encoder_phase.h"],
    includes = ["inc"],
    deps = ["//fw/math:math"],
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-std=c++17",
    ],
    visibility = ["//fw:__subpackages__"],
)
