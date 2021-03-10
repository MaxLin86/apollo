load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "roadguide_common",
    srcs = ["roadguide_gflags.cc"],
    hdrs = ["roadguide_gflags.h"],
    copts = ["-DMODULE_NAME=\\\"roadguide\\\""],
    deps = [
        "//external:gflags",
    ],
)

cpplint()
