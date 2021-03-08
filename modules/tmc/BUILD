load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tmc_component_lib",
    srcs = ["tmc_component.cc"],
    hdrs = ["tmc_component.h"],
    deps = [
        "//cyber",
        "//external:gflags",
        "//modules/tmc/common:tmc_common",
        "//modules/common/adapters:adapter_gflags",
        "//modules/tmc/proto:tmc_proto",
        "//modules/tmc/proto:tmc_conf_proto",
        "//modules/common/util:message_util",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/proto:sensor_proto",
	"//modules/guardian/proto:guardian_proto",
        ],
)

cc_binary(
    name = "lib_tmc_component.so",
    linkopts = ["-shared"],
    linkshared = True,
    linkstatic = False,
    deps = [":tmc_component_lib"],
)

cpplint()
