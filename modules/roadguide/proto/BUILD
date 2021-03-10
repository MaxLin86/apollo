package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "roadguide_proto",
    deps = [
        ":roadguide_proto_lib",
    ],
)

proto_library(
    name = "roadguide_proto_lib",
    srcs = ["roadguide.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib",
        "//modules/common/proto:error_code_proto_lib",
    ],
)

cc_proto_library(
    name = "roadguide_conf_proto",
    deps = [
        ":roadguide_conf_proto_lib",
    ],
)

proto_library(
    name = "roadguide_conf_proto_lib",
    srcs = ["roadguide_conf.proto"],
)
