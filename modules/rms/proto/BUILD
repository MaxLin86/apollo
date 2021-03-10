package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "rms_cc_proto",
    deps = [
        ":rms_proto_lib",
    ],
)

proto_library(
    name = "rms_proto_lib",
    srcs = ["rms.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
    ],
)

cc_proto_library(
    name = "rms_conf_cc_proto",
    deps = [
        ":rms_conf_proto_lib",
    ],
)

proto_library(
    name = "rms_conf_proto_lib",
    srcs = ["rms_conf.proto"],
)
