config_setting(
    name = "bare_metal",
    constraint_values = ["@platforms//os:none"],
)

COPTS = select({
    ":bare_metal": [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-ffreestanding",
        "-fno-exceptions",
        "-fno-rtti",
        "-std=c++17",
    ],
    "//conditions:default": ["-std=c++17"],
})

# Stable firmware/host CAN wire ABI. Transport and business services depend on it.
cc_library(
    name = "xt_can",
    hdrs = ["inc/protocol/xt_can.h"],
    includes = ["inc"],
    copts = COPTS,
    visibility = ["//fw:__subpackages__"],
)
