load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "librocket_radar_component.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":rocket_radar_component_lib"],
)



cc_library(
    name = "rocket_radar_component_lib",
    srcs = ["rocket_radar_component.cc"],
    hdrs = ["rocket_radar_component.h"],
    deps = [
        "//cyber",
        "//modules/drivers/radar/rocket_radar/proto:rocket_cc_proto",
        "//modules/drivers/radar/rocket_radar/driver:rocket_radar_api",
        "//modules/drivers/proto:ultra_radar_proto",
        "//modules/common/time",
        "//modules/common/adapters:adapter_gflags",
        "//modules/localization/proto:localization_proto",
        "//modules/common/util:message_util",
        "//modules/drivers/radar/uhnder_radar:radar_track",
        "//modules/canbus/proto:canbus_proto",
    ],
    includes = [
        "driver",
    ],
)



cpplint()
