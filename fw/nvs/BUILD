# Flash-backed persistent config (encoder cal today; moteus-like conf write).
cc_library(
    name = "encoder_cal_store",
    hdrs = ["inc/nvs/encoder_cal_store.h"],
    includes = ["inc"],
    deps = [
        "//fw/math:base",
        "@com_github_ARMmbed_mbed-g4//:mbed",
    ],
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-std=c++17",
    ],
    visibility = ["//fw:__subpackages__"],
)

cc_library(
    name = "runtime_config_store",
    hdrs = ["inc/nvs/runtime_config_store.h"],
    includes = ["inc"],
    deps = [
        "@com_github_ARMmbed_mbed-g4//:mbed",
    ],
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-std=c++17",
    ],
    visibility = ["//fw:__subpackages__"],
)
