# Header-only motor parameter tables (DM4310, ...).
cc_library(
    name = "motor",
    hdrs = ["inc/device/motor.h"],
    includes = ["inc"],
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-std=c++17",
    ],
    visibility = ["//fw:__subpackages__"],
)

cc_library(
    name = "ma600",
    hdrs = ["inc/device/ma600.h"],
    includes = ["inc"],
    deps = ["//fw/ports:ports"],
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-ffreestanding",
        "-fno-exceptions",
        "-fno-rtti",
        "-std=c++17",
    ],
    visibility = ["//fw:__subpackages__"],
)

cc_library(
    name = "drv8353s",
    srcs = ["src/drv8353s.cc"],
    hdrs = ["inc/device/drv8353s.h"],
    includes = ["inc"],
    deps = [
        "//fw/HAL:hal",
        "//fw/core:text_format",
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
    visibility = [
        "//fw:__pkg__",
        "//fw/middleware:__pkg__",
    ],
)
