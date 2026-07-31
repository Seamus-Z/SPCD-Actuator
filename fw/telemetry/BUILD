cc_library(
    name = "text_format",
    hdrs = ["inc/telemetry/text_format.h"],
    includes = ["inc"],
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-std=c++17",
    ],
    visibility = ["//fw:__subpackages__"],
)

# GUI / Monitor wire format (CAN-FD binary today; later also classic ≤8B pages).
cc_library(
    name = "xt_can",
    hdrs = ["inc/telemetry/xt_can.h"],
    includes = ["inc"],
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-std=c++17",
    ],
    visibility = ["//fw:__subpackages__"],
)

cc_library(
    name = "binary_link",
    srcs = ["src/binary_link.cc"],
    hdrs = [
        "inc/telemetry/binary_link.h",
        "inc/telemetry/xt_can.h",
    ],
    includes = ["inc"],
    deps = [
        ":xt_can",
        "//fw/HAL:hal",
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
    visibility = ["//fw:__subpackages__"],
)

cc_library(
    name = "telemetry",
    srcs = ["src/diagnostic_server.cc"],
    hdrs = [
        "inc/telemetry/diagnostic_server.h",
        "inc/telemetry/status_registry.h",
        "inc/telemetry/text_format.h",
    ],
    includes = ["inc"],
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
