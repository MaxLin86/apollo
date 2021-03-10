package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "quill_cc_proto",
    deps = [
        ":quill_proto",
    ],
)

proto_library(
    name = "quill_proto",
    srcs = ["quill.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)


