cc_library(
    name = "text_format",
    hdrs = ["text_format.h"],
    include_prefix = "telemetry",
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-std=c++17",
    ],
    visibility = ["//fw:__subpackages__"],
)

cc_library(
    name = "telemetry",
    srcs = ["diagnostic_server.cc"],
    hdrs = [
        "diagnostic_server.h",
        "status_registry.h",
        "text_format.h",
    ],
    include_prefix = "telemetry",
    deps = [
        ":text_format",
        "//fw/HAL:hal",
        "//fw/bootloader:boot_headers",
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
    visibility = ["//fw:__pkg__"],
)
