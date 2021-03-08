load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "position_est_component_lib",
    srcs = ["position_est_component.cc"],
    hdrs = ["position_est_component.h"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/time",
        "//modules/position_est/proto:position_cc_proto",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/util:message_util",
        "//modules/drivers/proto:sensor_proto",
        "//modules/position_est/proto:position_est_out_proto",
        "//modules/control/proto:control_proto",
        "@eigen",
        ],
)


cc_binary(
    name = "lib_position_est_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":position_est_component_lib"],
)


cpplint()
