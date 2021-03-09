load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "obstacle_multi_sensor_fusion",
    srcs = ["obstacle_multi_sensor_fusion.cc"],
    hdrs = ["obstacle_multi_sensor_fusion.h"],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/fusion/base",
        "//modules/perception/fusion/lib/interface",
        "//modules/perception/lib/registerer",
        "@eigen",
    ],
)

cc_library(
    name = "obstacle_tracker_fusion",
    srcs = ["obstacle_tracker_fusion.cc"],
    hdrs = ["obstacle_tracker_fusion.h"],
    deps = [
        ":target_fusion",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/obstacle/tracker/common",
        "//modules/perception/onboard/proto:fusion_obstacle_tracker_proto",
        "@eigen",
    ],
)

cc_library(
    name = "target_fusion",
    srcs = ["target_fusion.cc"],
    hdrs = ["target_fusion.h"],
    deps = [
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/obstacle/tracker/common",
        "//modules/perception/fusion/common:kalman_filter",
        "//modules/perception/onboard/proto:fusion_obstacle_tracker_proto",
        "@eigen",
    ],
)

cpplint()
