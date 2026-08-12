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
    name = "memory_pool",
    hdrs = ["inc/core/memory/pool.h"],
    includes = ["inc"],
    copts = COPTS,
    visibility = ["//fw:__subpackages__"],
)

cc_library(
    name = "text_format",
    hdrs = ["inc/core/format/text_format.h"],
    includes = ["inc"],
    copts = COPTS,
    visibility = ["//fw:__subpackages__"],
)
