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

# === Bootloader (bare-metal, CMSIS headers from mbed-g4) ===
# .vectors at 0x08000000, code at 0x0800C000
cc_binary(
    name = "bootloader.elf",
    srcs = [
        "bootloader/bootloader.cc",
        "bootloader/vectors.S",
        "bootloader/boot_mjlib.h",
        "bootloader/stm32g4xx_fdcan_typedefs.h",
        "bootloader/bootloader.h",
    ],
    deps = ["@com_github_ARMmbed_mbed-g4//:mbed"],
    copts = BAREMETAL_COPTS + ["-std=c++17"],
    linkopts = BAREMETAL_LINKOPTS + ["-T$(location bootloader/stm32g474_bootloader.ld)"],
    additional_linker_inputs = ["bootloader/stm32g474_bootloader.ld"],
)

genrule(
    name = "app_binary",
    srcs = ["app.elf"],
    outs = ["app.bin"],
    cmd = "$(OBJCOPY) -O binary $(location app.elf) $@",
    toolchains = ["@bazel_tools//tools/cpp:current_cc_toolchain"],
)

# === Application (bare-metal, CMSIS headers from mbed-g4) ===
# .isr_vector + code at 0x08010000
cc_binary(
    name = "app.elf",
    srcs = [
        "main.cpp",
        "app_vectors.S",
        "bootloader/enter_bootloader.cc",
        "bootloader/bootloader.h",
    ],
    deps = [
        "//fw/HAL:hal",
        "@com_github_ARMmbed_mbed-g4//:mbed",
    ],
    copts = BAREMETAL_COPTS + ["-std=c++17"],
    linkopts = BAREMETAL_LINKOPTS + ["-T$(location stm32g474_app.ld)"],
    additional_linker_inputs = ["stm32g474_app.ld"],
)

# === Combined flash image (ISR + bootloader + app) ===
genrule(
    name = "xtellar.combined",
    srcs = ["bootloader.elf", "app_binary"],
    outs = ["xtellar.combined.bin"],
    cmd = (
        # Bootloader: .vectors at 0x08000000, code at 0x0800C000 (linker handles layout)
        "$(OBJCOPY) -O binary $(location bootloader.elf) $(@D)/bl_full.bin && " +
        "python3 -c \"import os,sys;s=os.path.getsize('$(@D)/bl_full.bin');" +
        "sys.exit('bootloader binary too big (%d > 65536)' % s) if s > 65536 else 0\" && " +
        "cp $(@D)/bl_full.bin $@ && " +
        # Pad to app start at 0x10000 (65536) if bootloader is smaller
        "python3 -c \"import os;f=open('$@','ab');s=os.fstat(f.fileno()).st_size;" +
        "f.truncate(max(s,65536))\" && " +
        # App at 0x08010000
        "cat $(location app_binary) >> $@"
    ),
    toolchains = ["@bazel_tools//tools/cpp:current_cc_toolchain"],
)
