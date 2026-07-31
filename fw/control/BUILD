# Reserved for the 8-byte host control protocol (not FOC).
# FOC controllers: //fw/foc_ctrl:*
# GUI/Monitor protocol: //fw/telemetry:binary_link
cc_library(
    name = "host_ctrl",
    hdrs = ["inc/control/host_ctrl.h"],
    includes = ["inc"],
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-std=c++17",
    ],
    visibility = ["//fw:__subpackages__"],
)

# Compatibility shims so old #include "control/..." still jumps to foc_ctrl.
cc_library(
    name = "voltage_foc",
    hdrs = ["inc/control/voltage_foc.h"],
    includes = ["inc"],
    deps = ["//fw/foc_ctrl:voltage_foc"],
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
    deps = ["//fw/foc_ctrl:current_loop"],
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-std=c++17",
    ],
    visibility = ["//fw:__subpackages__"],
)
