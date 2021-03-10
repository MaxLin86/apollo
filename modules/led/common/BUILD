load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "led_common",
    srcs = ["led_gflags.cc"],
    hdrs = ["led_gflags.h"],
    copts = ["-DMODULE_NAME=\\\"led\\\""],
    deps = [
        "//external:gflags",
    ],
)

cpplint()
