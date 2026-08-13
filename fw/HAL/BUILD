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
    name = "system_clock",
    srcs = ["src/system_clock.cc"],
    hdrs = ["inc/HAL/system_clock.h"],
    # Physical path inc/HAL/foo.h matches #include "HAL/foo.h".
    includes = ["inc"],
    deps = ["@com_github_ARMmbed_mbed-g4//:mbed"],
    copts = BAREMETAL_COPTS,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "hal",
    srcs = [
        "src/fdcan.cc",
        "src/phase_current_adc.cc",
        "src/phase_pwm.cc",
        "src/vt_sense_adc.cc",
    ],
    hdrs = [
        "inc/HAL/fdcan.h",
        "inc/HAL/millisecond_timer.h",
        "inc/HAL/phase_current_adc.h",
        "inc/HAL/phase_pwm.h",
        "inc/HAL/stm32_spi.h",
        "inc/HAL/vt_sense_adc.h",
    ],
    # Physical path inc/HAL/foo.h matches #include "HAL/foo.h".
    includes = ["inc"],
    deps = [
        ":system_clock",
        "//fw/math:protection",
        "//fw/ports:ports",
        "@com_github_ARMmbed_mbed-g4//:mbed",
    ],
    copts = BAREMETAL_COPTS,
    visibility = ["//visibility:public"],
)
