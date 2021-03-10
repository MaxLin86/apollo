load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libuhnder_radar_component.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":uhnder_radar_component_lib"],
)



cc_library(
    name = "uhnder_radar_component_lib",
    srcs = ["uhnder_radar_component.cc"],
    hdrs = ["uhnder_radar_component.h"],
    deps = [
        "//cyber",
        "//modules/drivers/radar/uhnder_radar/proto:uhnder_cc_proto",
        "//modules/drivers/radar/uhnder_radar/driver:uhnder_radar_api",
        "//modules/common/time",
        "//modules/common/adapters:adapter_gflags",
        "//modules/localization/proto:localization_proto",
        "//modules/common/util:message_util",
        "//modules/drivers/radar/uhnder_radar:radar_track",
        "//modules/drivers/proto:ultra_radar_proto",
    ],
    includes = [
        "driver",
    ],
)



cc_library(
    name = "radar_track",
    srcs = ["objProcess.cc"],
    hdrs = ["objProcess.h"],
    deps = [
        "//cyber",
        "@eigen",
    ],
)


cpplint()
