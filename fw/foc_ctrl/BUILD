# FOC controllers (moteus-aligned current/position + voltage open-loop).
cc_library(
    name = "simple_pi",
    hdrs = ["inc/foc_ctrl/simple_pi.h"],
    includes = ["inc"],
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-std=c++17",
    ],
    visibility = ["//fw:__subpackages__"],
)

cc_library(
    name = "pid",
    hdrs = ["inc/foc_ctrl/pid.h"],
    includes = ["inc"],
    deps = [":simple_pi"],
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-std=c++17",
    ],
    visibility = ["//fw:__subpackages__"],
)

cc_library(
    name = "encoder_pll",
    hdrs = ["inc/foc_ctrl/encoder_pll.h"],
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
    name = "position_loop",
    hdrs = ["inc/foc_ctrl/position_loop.h"],
    includes = ["inc"],
    deps = [
        ":pid",
        "//fw/math:math",
    ],
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-std=c++17",
    ],
    visibility = ["//fw:__subpackages__"],
)

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
    deps = [
        ":simple_pi",
        "//fw/math:math",
    ],
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-std=c++17",
    ],
    visibility = ["//fw:__subpackages__"],
)

# Deprecated: kept for reference; app uses position_loop.
cc_library(
    name = "speed_loop",
    hdrs = ["inc/foc_ctrl/speed_loop.h"],
    includes = ["inc"],
    deps = ["//fw/math:math"],
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-std=c++17",
    ],
    visibility = ["//fw:__subpackages__"],
)
