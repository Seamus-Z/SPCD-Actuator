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

cc_library(
    name = "bemf_ident",
    hdrs = ["inc/calibration/bemf_ident.h"],
    includes = ["inc"],
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-std=c++17",
    ],
    visibility = ["//fw:__subpackages__"],
)

cc_library(
    name = "r_ident",
    hdrs = ["inc/calibration/r_ident.h"],
    includes = ["inc"],
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-std=c++17",
    ],
    visibility = ["//fw:__subpackages__"],
)

cc_library(
    name = "l_ident",
    hdrs = ["inc/calibration/l_ident.h"],
    includes = ["inc"],
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-std=c++17",
    ],
    visibility = ["//fw:__subpackages__"],
)

cc_library(
    name = "cogging",
    hdrs = ["inc/calibration/cogging.h"],
    includes = ["inc"],
    deps = ["//fw/math:math"],
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-std=c++17",
    ],
    visibility = ["//fw:__subpackages__"],
)
