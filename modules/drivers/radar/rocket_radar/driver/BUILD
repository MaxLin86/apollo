load("//tools:cpplint.bzl", "cpplint")
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rocket_radar_api",
    srcs =[
        "libsrapi.a",
    ],
    includes = [
        "driver",
    ],
    linkstatic = True,
)


cc_binary(
    name = "librocketradar.so",
    linkopts = ["-shared"],
    linkstatic = True,
    deps = [":rocket_radar_api"],
)

cpplint()