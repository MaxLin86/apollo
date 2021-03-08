load("//tools:cpplint.bzl", "cpplint")
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "uhnder_radar_api",
    srcs =[
        "libsrapi.a",
    ],
    includes = [
        "driver",
    ],
    linkstatic = True,
)


cc_binary(
    name = "libuhnradar.so",
    linkopts = ["-shared"],
    linkstatic = True,
    deps = [":uhnder_radar_api"],
)

cpplint()