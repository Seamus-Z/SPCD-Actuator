cc_library(
    name = "drv8353s",
    srcs = ["src/drv8353s.cc"],
    hdrs = ["inc/device/drv8353s.h"],
    includes = ["inc"],
    deps = [
        "//fw/HAL:hal",
        "//fw/telemetry:text_format",
        "@com_github_ARMmbed_mbed-g4//:mbed",
    ],
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-ffreestanding",
        "-fno-exceptions",
        "-fno-rtti",
        "-O2",
        "-g0",
        "-std=c++17",
    ],
    visibility = ["//fw:__pkg__"],
)
