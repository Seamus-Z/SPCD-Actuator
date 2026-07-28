BAREMETAL_COPTS = [
    "-mcpu=cortex-m4",
    "-mthumb",
    "-ffreestanding",
    "-fno-exceptions",
    "-fno-rtti",
    "-O2",
    "-g0",
    "-std=c++17",
]

cc_library(
    name = "hal",
    srcs = ["fdcan.cc"],
    hdrs = [
        "fdcan.h",
        "millisecond_timer.h",
    ],
    include_prefix = "HAL",
    deps = ["@com_github_ARMmbed_mbed-g4//:mbed"],
    copts = BAREMETAL_COPTS,
    visibility = ["//visibility:public"],
)
