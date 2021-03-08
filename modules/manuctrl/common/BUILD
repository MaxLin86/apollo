load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "manuctrl_common",
    srcs = ["manuctrl_gflags.cc"],
    hdrs = ["manuctrl_gflags.h"],
    copts = ["-DMODULE_NAME=\\\"manuctrl\\\""],
    deps = [
        "//external:gflags",
    ],
)

cpplint()
