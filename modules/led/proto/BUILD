load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "led_proto",
    deps = [
        ":led_proto_lib",
    ],
)

proto_library(
    name = "led_proto_lib",
    srcs = ["led.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib", 
    ],
)

cpplint()
