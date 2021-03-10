load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libquill_radar_component.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":quill_radar_component_lib"],
)



cc_library(
    name = "quill_radar_component_lib",
    srcs = ["quill_radar_component.cc"],
    hdrs = ["quill_radar_component.h"],
    deps = [
        "//cyber",
        "//modules/drivers/radar/quill_radar/proto:quill_cc_proto",
        "//modules/drivers/radar/quill_radar/driver:quill_radar_api",
        "//modules/drivers/proto:ultra_radar_proto",
        "//modules/common/time",
        "//modules/common/adapters:adapter_gflags",
        "//modules/localization/proto:localization_proto",
        "//modules/common/util:message_util",
        "//modules/drivers/radar/uhnder_radar:radar_track",
        "//modules/common/monitor_log",
    ],
    includes = [
        "driver",
    ],
)



cpplint()
