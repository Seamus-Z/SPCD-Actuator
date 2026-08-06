# FOC / control math (header-only for now).
cc_library(
    name = "math",
    hdrs = [
        "inc/math/constants.h",
        "inc/math/commutation.h",
        "inc/math/cogging.h",
        "inc/math/foc.h",
    ],
    includes = ["inc"],
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
