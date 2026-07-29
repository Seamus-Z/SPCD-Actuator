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
    srcs = [
        "src/fdcan.cc",
        "src/phase_current_adc.cc",
        "src/phase_pwm.cc",
    ],
    hdrs = [
        "inc/fdcan.h",
        "inc/millisecond_timer.h",
        "inc/phase_current_adc.h",
        "inc/phase_pwm.h",
    ],
    includes = ["inc"],
    strip_include_prefix = "inc",
    include_prefix = "HAL",
    deps = ["@com_github_ARMmbed_mbed-g4//:mbed"],
    copts = BAREMETAL_COPTS,
    visibility = ["//visibility:public"],
)
