load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

# add by shzhw
cc_library(
    name = "kalman_filter",
    srcs = ["kalman_filter.cc"],
    hdrs = ["kalman_filter.h"],
    deps = [

    ],
)

cc_library(
    name = "extended_kalman_filter",
    srcs = ["extended_kalman_filter.cc"],
    hdrs = ["extended_kalman_filter.h"],
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
    ],
    deps = [
        "kalman_filter",
    ],
)

cc_library(
    name = "tracking_obj_ekf",
    srcs = ["tracking_obj_ekf.cc"],
    hdrs = ["tracking_obj_ekf.h"],
    deps = [
        "extended_kalman_filter",
    ],
)

cc_library(
    name = "gps_kalman_filter",
    srcs = ["gps_kalman_filter.cc"],
    hdrs = ["gps_kalman_filter.h"],
    deps = [
        "extended_kalman_filter",
    ],
)

cc_library(
    name = "self_car_localization_ekf",
    srcs = ["self_car_localization_ekf.cc"],
    hdrs = ["self_car_localization_ekf.h"],
    deps = [
        "extended_kalman_filter",
    ],
)

cc_library(
    name = "ins_guidepost_fusion_ekf",
    srcs = ["ins_guidepost_fusion_ekf.cc"],
    hdrs = ["ins_guidepost_fusion_ekf.h"],
    deps = [
        "extended_kalman_filter",
    ],
)
#--------------------------------

cc_library(
    name = "digital_filter",
    srcs = ["digital_filter.cc"],
    hdrs = ["digital_filter.h"],
    deps = [
        ":digital_filter_coefficients",
        "//cyber/common:log",
    ],
)

cc_library(
    name = "mean_filter",
    srcs = ["mean_filter.cc"],
    hdrs = ["mean_filter.h"],
    deps = [
        "//cyber/common:log",
    ],
)

cc_library(
    name = "digital_filter_coefficients",
    srcs = ["digital_filter_coefficients.cc"],
    hdrs = ["digital_filter_coefficients.h"],
    deps = [
        "//cyber",
    ],
)

cc_test(
    name = "digital_filter_test",
    size = "small",
    srcs = ["digital_filter_test.cc"],
    deps = [
        ":digital_filter",
        ":digital_filter_coefficients",
        "//cyber",
        "@gtest//:main",
    ],
)

cc_test(
    name = "mean_filter_test",
    size = "small",
    srcs = ["mean_filter_test.cc"],
    deps = [
        ":mean_filter",
        "@gtest//:main",
    ],
)

cc_test(
    name = "digital_filter_coefficients_test",
    size = "small",
    srcs = ["digital_filter_coefficients_test.cc"],
    deps = [
        ":digital_filter_coefficients",
        "//cyber",
        "@gtest//:main",
    ],
)

cpplint()
