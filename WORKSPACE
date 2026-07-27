workspace(name = "xtellar_mbed")

BAZEL_VERSION = "7.4.1"
BAZEL_VERSION_SHA = "c97f02133adce63f0c28678ac1f21d65fa8255c80429b588aeeba8a1fac6202b"

load("//tools/workspace:github_archive.bzl", "github_archive")
load("//tools/workspace/rules_mbed:repository.bzl", "rules_mbed_repository")
load("//tools/workspace/bazel_toolchain:repository.bzl", "bazel_toolchain_repository")

# --- rules_mbed (mbed OS build rules) ---
rules_mbed_repository()
load("@com_github_mjbots_rules_mbed//:rules.bzl", "mbed_register")

# --- LLVM/Clang toolchain ---
bazel_toolchain_repository()
load("@com_github_mjbots_bazel_toolchain//toolchain:deps.bzl", "bazel_toolchain_dependencies")
bazel_toolchain_dependencies()

# Use locally cached LLVM from moteus build (no re-download)
local_repository(
    name = "llvm_toolchain",
    path = "/home/zzr/.cache/bazel/_bazel_zzr/f2961c8520cfcf5401a1e141bd11dfb6/external/llvm_toolchain",
)

load("@llvm_toolchain//:toolchains.bzl", "llvm_register_toolchains")
llvm_register_toolchains()

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
    },
)
