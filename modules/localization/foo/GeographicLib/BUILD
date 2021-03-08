load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "geographic_lib",
    srcs = [
        "Geocentric.cc",
        "LocalCartesian.cc",
        "Math.cc",
    ],
    hdrs = [
        "Geocentric.h",
        "LocalCartesian.h",
                "Math.h",
                "Config.h",
                "Constants.h",
    ],
    copts = ["-DMODULE_NAME=\\\"localization\\\""],
    deps = [
        "//cyber",
                "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "//modules/common/math",

        "@eigen",
        "@gtest",
    ],
)



cpplint()
