load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "serial_stream",
    srcs = ["serial_stream.cc"],
    deps = [
        ":stream",
        "//cyber",
        "//modules/drivers/rs485/util:rs485_util",
    ],
)

cc_library(
    name = "stream",
    hdrs = ["stream.h"],
    deps = [
        "//cyber",
        "//modules/drivers/rs485/util:rs485_util",
    ],
)

cpplint()
