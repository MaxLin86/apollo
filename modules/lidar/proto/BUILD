load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "lidar_proto",
    deps = [
        ":lidar_proto_lib",
    ],
)

proto_library(
    name = "lidar_proto_lib",
    srcs = ["lidar.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib", 
    ],
)

cpplint()
