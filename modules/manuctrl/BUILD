load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "manuctrl_component_lib",
    srcs = ["manuctrl_component.cc"],
    hdrs = ["manuctrl_component.h"],
    deps = [
        "//cyber",
        "//external:gflags",
        "//modules/manuctrl/common:manuctrl_common",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/adapters:adapter_gflags",
        "//modules/manuctrl/proto:manuctrl_proto",
        "//modules/manuctrl/proto:manuctrl_conf_proto",
        "//modules/common/util:message_util",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/joystick/stream:serial_stream",
        ],
)

cc_binary(
    name = "lib_manuctrl_component.so",
    linkopts = ["-shared"],
    linkshared = True,
    linkstatic = False,
    deps = [":manuctrl_component_lib"],
)

cpplint()
