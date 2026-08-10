# Pure motor-control algorithms. These targets must not depend on HAL, device,
# telemetry, NVS, or application code.

COPTS = [
    "-mcpu=cortex-m4",
    "-mthumb",
    "-ffreestanding",
    "-fno-exceptions",
    "-fno-rtti",
    "-std=c++17",
]

cc_library(
    name = "base",
    hdrs = [
        "inc/math/constants.h",
        "inc/math/commutation.h",
        "inc/math/cogging.h",
        "inc/math/encoder_comp.h",
    ],
    includes = ["inc"],
    copts = COPTS,
    visibility = ["//fw:__subpackages__"],
)

cc_library(
    name = "foc",
    hdrs = [
        "inc/math/foc/transform.h",
        "inc/math/foc/pi.h",
        "inc/math/foc/controller.h",
        "inc/math/foc/modulator.h",
    ],
    includes = ["inc"],
    deps = [":base"],
    copts = COPTS,
    visibility = ["//fw:__subpackages__"],
)

cc_library(
    name = "servo_mode",
    hdrs = [
        "inc/math/servo_mode/pid.h",
        "inc/math/servo_mode/encoder_pll.h",
        "inc/math/servo_mode/servo_mode.h",
    ],
    includes = ["inc"],
    deps = [":base", ":foc"],
    copts = COPTS,
    visibility = ["//fw:__subpackages__"],
)

cc_library(
    name = "calibration",
    hdrs = [
        "inc/math/calibration/encoder_phase.h",
        "inc/math/calibration/bemf_ident.h",
        "inc/math/calibration/r_ident.h",
        "inc/math/calibration/l_ident.h",
        "inc/math/calibration/cogging.h",
    ],
    includes = ["inc"],
    deps = [":base"],
    copts = COPTS,
    visibility = ["//fw:__subpackages__"],
)
