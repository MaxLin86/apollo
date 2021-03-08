

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "urgency_cc_proto",
    deps = [
        ":urgency_proto",
    ],
)

proto_library(
    name = "urgency_proto",
    srcs = ["urgency.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)


cc_proto_library(
    name = "aeb_cmd_cc_proto",
    deps = [
        ":aeb_cmd_proto",
    ],
)

proto_library(
    name = "aeb_cmd_proto",
    srcs = ["aeb_cmd.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib",
        "//modules/control/proto:control_command_proto_lib",
    ],
)

