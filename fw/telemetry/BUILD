cc_library(
    name = "text_format",
    hdrs = ["inc/text_format.h"],
    includes = ["inc"],
    strip_include_prefix = "inc",
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
    srcs = ["src/diagnostic_server.cc"],
    hdrs = [
        "inc/diagnostic_server.h",
        "inc/status_registry.h",
        "inc/text_format.h",
    ],
    includes = ["inc"],
    strip_include_prefix = "inc",
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
