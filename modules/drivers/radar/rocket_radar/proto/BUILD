package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "rocket_cc_proto",
    deps = [
        ":rocket_proto",
    ],
)

proto_library(
    name = "rocket_proto",
    srcs = ["rocket.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)


