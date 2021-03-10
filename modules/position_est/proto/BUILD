
package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "position_cc_proto",
    deps = [
        ":position_conf_proto",
    ],
)

proto_library(
    name = "position_conf_proto",
    srcs = ["position_est_conf.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)
