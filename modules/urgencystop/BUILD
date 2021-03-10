load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "urgency_stop_component_lib",
    srcs = ["urgency_stop_component.cc"],
    hdrs = ["urgency_stop_component.h"],
    copts = ['-DMODULE_NAME=\\"AEBS\\"'],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/time",
        "//modules/urgencystop/proto:aeb_cmd_cc_proto",
        "//modules/urgencystop/proto:urgency_cc_proto",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/util:message_util",
        "//modules/localization/proto:localization_proto",
        "//modules/drivers/proto:ultra_radar_proto",
        "//modules/common/configs:vehicle_config_helper",
        ],
)


cc_binary(
    name = "lib_urgency_stop_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":urgency_stop_component_lib"],
)


cpplint()
