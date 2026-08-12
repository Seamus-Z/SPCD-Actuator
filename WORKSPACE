workspace(name = "xtellar_mbed")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# --- googletest: host-side unit tests for //fw/math (run with --config=host) ---
http_archive(
    name = "com_google_googletest",
    urls = ["https://github.com/google/googletest/archive/refs/tags/release-1.12.1.tar.gz"],
    strip_prefix = "googletest-release-1.12.1",
    sha256 = "81964fe578e9bd7c94dfdb09c8e4d6e6759e19967e397dbea48d1c10e45d0df2",
)

BAZEL_VERSION = "7.4.1"
BAZEL_VERSION_SHA = "c97f02133adce63f0c28678ac1f21d65fa8255c80429b588aeeba8a1fac6202b"

load("//tools/workspace:github_archive.bzl", "github_archive")
load("//tools/workspace/rules_mbed:repository.bzl", "rules_mbed_repository")
load("//tools/workspace/bazel_toolchain:repository.bzl", "bazel_toolchain_repository")

# --- rules_mbed (mbed OS build rules) ---
rules_mbed_repository()
load("@com_github_mjbots_rules_mbed//:rules.bzl", "mbed_register")

# --- Toolchain helper rules (vendored in third_party/) ---
# Host targets (unit tests, tools) use Bazel's auto-detected local cc
# toolchain; only the stm32g4 cross toolchain comes from rules_mbed.
bazel_toolchain_repository()
load("@com_github_mjbots_bazel_toolchain//toolchain:deps.bzl", "bazel_toolchain_dependencies")
bazel_toolchain_dependencies()

# --- mbed OS for STM32G474 ---
mbed_register()

load("@com_github_mjbots_rules_mbed//tools/workspace/mbed:repository.bzl", "mbed_repository")

mbed_repository(
    name = "com_github_ARMmbed_mbed-g4",
    target = "targets/TARGET_STM/TARGET_STM32G4/TARGET_STM32G474xE/TARGET_NUCLEO_G474RE",
    config = {
        "mbed_target": "targets/TARGET_STM/TARGET_STM32G4/TARGET_STM32G474xE/TARGET_NUCLEO_G474RE",
        "MBED_CONF_RTOS_PRESENT": "0",
        "MBED_CONF_TARGET_LSE_AVAILABLE": "0",
        "NDEBUG": "1",

        # TIM5 is motor PWM (family 3 / x1); move mbed us ticker to TIM15.
        "MBED_US_TIMER_TIM": "TIM15",
        "MBED_US_TIMER_TIM_USCORE": "TIM15_",
        "MBED_US_TIMER_USCORE_TIM": "_TIM15",
        "TIM_MST_IRQ": "TIM1_BRK_TIM15_IRQn",
        "TIM_MST_BIT_WIDTH": "16",
    },
)
