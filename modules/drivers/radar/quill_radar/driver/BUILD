load("//tools:cpplint.bzl", "cpplint")
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "quill_radar_api",
    srcs =[
        "librrapi.a",
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
    deps = [":quill_radar_api"],
)

cpplint()