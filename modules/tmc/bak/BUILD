package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "tmc_proto",
    deps = [
        ":tmc_proto_lib",
    ],
)

proto_library(
    name = "tmc_proto_lib",
    srcs = ["tmc.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib",
        "//modules/control/proto:control_command_proto_lib",
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/common/proto:drive_state_proto_lib",
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:vehicle_signal_proto_lib",
    ],
)

cc_proto_library(
    name = "tmc_conf_proto",
    deps = [
        ":tmc_conf_proto_lib",
    ],
)

proto_library(
    name = "tmc_conf_proto_lib",
    srcs = ["tmc_conf.proto"],
)
