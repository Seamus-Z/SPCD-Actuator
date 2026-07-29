# Control modes (open-loop voltage FOC, current loop, later position).
cc_library(
    name = "voltage_foc",
    hdrs = ["inc/control/voltage_foc.h"],
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
    hdrs = ["inc/control/current_loop.h"],
    includes = ["inc"],
    deps = ["//fw/math:math"],
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-std=c++17",
    ],
    visibility = ["//fw:__subpackages__"],
)
