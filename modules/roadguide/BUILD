load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "roadguide_component_lib",
    srcs = ["roadguide_component.cc"],
    hdrs = [
        "roadguide_component.h",
        ],
    deps = [
        "//cyber",
        "//external:gflags",
        "//modules/roadguide/common:roadguide_common",
        "//modules/common/adapters:adapter_gflags",
        "//modules/roadguide/proto:roadguide_proto",
        "//modules/roadguide/proto:roadguide_conf_proto",
        "//modules/common/util:message_util",
        "//modules/roadguide:transfer_lib",
        "//modules/roadguide:rxmodule_lib",
        ],
)

cc_library(
    name = "util_lib",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    deps = ["//cyber"],
)

cc_library(
    name = "slip_lib",
    srcs = ["slip.cc"],
    hdrs = ["slip.h"],
    deps = ["//cyber"],
)

cc_library(
    name = "rxmodule_lib",
    srcs = ["rxmodule.cc"],
    hdrs = ["rxmodule.h", "rxboard_def.h"],
    deps = [
        "//cyber",
        "//modules/roadguide:transfer_lib",
        "//modules/roadguide/proto:roadguide_proto",
        ],
)

cc_library(
    name = "transfer_lib",
    srcs = ["transfer.cc"],
    hdrs = ["transfer.h", "CRC.h", "rxboard_def.h"],
    deps = [
        "//cyber",
        "//modules/drivers/rs485/stream:serial_stream",
        "//modules/roadguide:util_lib",
        "//modules/roadguide:slip_lib",
        ],
)

cc_binary(
    name = "lib_roadguide_component.so",
    linkopts = ["-shared"],
    linkshared = True,
    linkstatic = False,
    deps = [
        ":roadguide_component_lib",
        ],
)

cpplint()
