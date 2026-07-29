cc_library(
    name = "pool",
    hdrs = ["inc/pool.h"],
    includes = ["inc"],
    strip_include_prefix = "inc",
    include_prefix = "pool",
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-ffreestanding",
        "-fno-exceptions",
        "-fno-rtti",
        "-std=c++17",
    ],
    visibility = ["//fw:__subpackages__"],
)
