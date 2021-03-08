load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tmc_common",
    srcs = ["tmc_gflags.cc"],
    hdrs = ["tmc_gflags.h"],
    copts = ["-DMODULE_NAME=\\\"tmc\\\""],
    deps = [
        "//external:gflags",
    ],
)

cpplint()
