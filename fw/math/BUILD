# Pure motor-control algorithms. These targets must not depend on HAL, device,
# telemetry, NVS, or application code.
#
# Header-only and dependency-free, so the same targets build both for the
# stm32g4 cross platform and for the host (unit tests: bazel test //fw/math:...).

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
        "inc/math/servo_mode/mit_mode.h",
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

# ---- Host unit tests (run with: tools/bazel test //fw/math:all) ----

TEST_COPTS = ["-std=c++17"]

cc_test(
    name = "pid_test",
    size = "small",
    srcs = ["test/pid_test.cc"],
    deps = [
        ":servo_mode",
        "@com_google_googletest//:gtest_main",
    ],
    copts = TEST_COPTS,
)

cc_test(
    name = "mit_mode_test",
    size = "small",
    srcs = ["test/mit_mode_test.cc"],
    deps = [
        ":servo_mode",
        "@com_google_googletest//:gtest_main",
    ],
    copts = TEST_COPTS,
)

cc_test(
    name = "encoder_pll_test",
    size = "small",
    srcs = ["test/encoder_pll_test.cc"],
    deps = [
        ":servo_mode",
        "@com_google_googletest//:gtest_main",
    ],
    copts = TEST_COPTS,
)

cc_test(
    name = "servo_mode_test",
    size = "small",
    srcs = ["test/servo_mode_test.cc"],
    deps = [
        ":servo_mode",
        "@com_google_googletest//:gtest_main",
    ],
    copts = TEST_COPTS,
)

test_suite(
    name = "math_tests",
    tests = [
        ":encoder_pll_test",
        ":mit_mode_test",
        ":pid_test",
        ":servo_mode_test",
    ],
)
