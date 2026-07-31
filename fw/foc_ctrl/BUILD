# FOC controllers (voltage open-loop + current loop). Not the host control protocol.
cc_library(
    name = "voltage_foc",
    hdrs = ["inc/foc_ctrl/voltage_foc.h"],
    includes = ["inc"],
    deps = ["//fw/math:math"],
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-std=c++17",
    ],
    visibility = ["//fw:__subpackages__"],
)

cc_library(
    name = "current_loop",
    hdrs = ["inc/foc_ctrl/current_loop.h"],
    includes = ["inc"],
    deps = ["//fw/math:math"],
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-std=c++17",
    ],
    visibility = ["//fw:__subpackages__"],
)
