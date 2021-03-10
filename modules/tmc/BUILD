load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tmc_component_lib",
    srcs = ["tmc_component.cc"],
    hdrs = ["tmc_component.h"],
    copts = ['-DMODULE_NAME=\\"tmc\\"'],
    deps = [
        "//cyber",
        "//external:gflags",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/tmc/common:tmc_common",
        "//modules/tmc/proto:tmc_proto",
        "//modules/tmc/proto:tmc_conf_proto",
        "//modules/canbus/proto:canbus_proto",
        "//modules/guardian/proto:guardian_proto",
        "//modules/manuctrl/proto:manuctrl_proto",
        "//modules/urgencystop/proto:aeb_cmd_cc_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/localization/proto:pose_proto",
        "//modules/map/relative_map/proto:navigation_proto",
        "//modules/localization/msf/common/util:localization_msf_common_util_frame_transform",
        ],
)

cc_binary(
    name = "lib_tmc_component.so",
    linkopts = ["-shared"],
    linkshared = True,
    linkstatic = False,
    deps = [":tmc_component_lib"],
)

cpplint()
