# === Bare-metal compile options (shared by bootloader and app) ===
BAREMETAL_COPTS = [
    "-mcpu=cortex-m4",
    "-mthumb",
    "-ffreestanding",
    "-fno-exceptions",
    "-fno-rtti",
    "-O2",
    "-g0",
]

BAREMETAL_LINKOPTS = [
    "-nostartfiles",
    "-nostdlib",
    "-Wl,--gc-sections",
]

# Re-export bootloader ELF at //fw for existing flash scripts.
alias(
    name = "bootloader.elf",
    actual = "//fw/bootloader:bootloader.elf",
)

genrule(
    name = "app_binary",
    srcs = ["app.elf"],
    outs = ["app.bin"],
    cmd = "$(OBJCOPY) -O binary $(location app.elf) $@",
    toolchains = ["@bazel_tools//tools/cpp:current_cc_toolchain"],
)

# === Application ===
# main.cpp holds the Application state machine + CRT entry.
cc_binary(
    name = "app.elf",
    srcs = [
        "app/main.cpp",
        "app/application.cc",
        "app/application.h",
        "app/app_state.h",
        "app/app_telemetry.cc",
        "app/app_telemetry.h",
        "app/board_config.h",
        "app/app_vectors.S",
    ],
    deps = [
        "//fw/HAL:hal",
        "//fw/device:drv8353s",
        "//fw/pool:pool",
        "//fw/telemetry:telemetry",
        "//fw/bootloader:enter_bootloader",
        "//fw/bootloader:boot_headers",
        "@com_github_ARMmbed_mbed-g4//:mbed",
    ],
    includes = [
        ".",
        "app",
    ],
    copts = BAREMETAL_COPTS + ["-std=c++17"],
    linkopts = BAREMETAL_LINKOPTS + ["-T$(location app/stm32g474_app.ld)"],
    additional_linker_inputs = ["app/stm32g474_app.ld"],
)

genrule(
    name = "xtellar.combined",
    srcs = ["//fw/bootloader:bootloader.elf", "app_binary"],
    outs = ["xtellar.combined.bin"],
    cmd = (
        "$(OBJCOPY) -O binary $(location //fw/bootloader:bootloader.elf) $(@D)/bl_full.bin && " +
        "python3 -c \"import os,sys;s=os.path.getsize('$(@D)/bl_full.bin');" +
        "sys.exit('bootloader binary too big (%d > 65536)' % s) if s > 65536 else 0\" && " +
        "cp $(@D)/bl_full.bin $@ && " +
        "python3 -c \"import os;f=open('$@','ab');s=os.fstat(f.fileno()).st_size;" +
        "f.truncate(max(s,65536))\" && " +
        "cat $(location app_binary) >> $@"
    ),
    toolchains = ["@bazel_tools//tools/cpp:current_cc_toolchain"],
)
