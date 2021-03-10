load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lidar_common",
    srcs = ["lidar_gflags.cc"],
    hdrs = ["lidar_gflags.h"],
    copts = ["-DMODULE_NAME=\\\"lidar\\\""],
    deps = [
        "//external:gflags",
    ],
)

cpplint()
