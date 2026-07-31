# Compatibility shims for old #include "protocol/...".
# Real GUI/Monitor protocol: //fw/telemetry:xt_can and //fw/telemetry:binary_link
cc_library(
    name = "xt_can",
    hdrs = ["inc/protocol/xt_can.h"],
    includes = ["inc"],
    deps = ["//fw/telemetry:xt_can"],
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-std=c++17",
    ],
    visibility = ["//fw:__subpackages__"],
)

cc_library(
    name = "binary_link",
    hdrs = ["inc/protocol/binary_link.h"],
    includes = ["inc"],
    deps = ["//fw/telemetry:binary_link"],
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-std=c++17",
    ],
    visibility = ["//fw:__pkg__"],
)
