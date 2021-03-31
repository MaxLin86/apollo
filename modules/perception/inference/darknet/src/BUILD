load("//tools:cpplint.bzl", "cpplint")
load("//tools:cuda_library.bzl", "cuda_library")

package(default_visibility = ["//visibility:public"])



cc_library(
    name = "darknet_src",
    srcs = glob(["*.c"]),
    hdrs = glob(["*.h"]),
    deps = [
        "//modules/perception/base:blob",
        "//modules/perception/inference:layer_lib",
        "@cuda",
        "@eigen",
    ],
)

cpplint()