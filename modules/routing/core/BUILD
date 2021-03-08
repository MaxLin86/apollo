load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "core",
    deps = [
        ":routing_navigator",
    ],
)

cc_library(
    name = "routing_navigator",
    srcs = ["navigator.cc"],
    hdrs = ["navigator.h"],
    copts = ['-DMODULE_NAME=\\"routing\\"',
    "-DMODULE_NAME=\\\"planning\\\"",
            "-fopenmp",],
    #20200911--------------------
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
    ],
    deps = [
                #"//modules/planning/open_space/coarse_trajectory_generator:hybrid_a_star",
        ":routing_black_list_range_generator",
        ":routing_result_generator",
        "//modules/common/util",
        "//modules/routing/strategy",

    ],
)

cc_library(
    name = "routing_black_list_range_generator",
    srcs = ["black_list_range_generator.cc"],
    hdrs = ["black_list_range_generator.h"],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [
        "//modules/routing/graph",
    ],
)

cc_library(
    name = "routing_result_generator",
    srcs = ["result_generator.cc"],
    hdrs = ["result_generator.h"],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [
        "//modules/routing/graph",
    ],
)

cpplint()
