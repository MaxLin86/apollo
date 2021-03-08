load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "remoteManage_lib",
    srcs = ["remoteManage.cc"],
    hdrs = ["remoteManage.h"],
    deps = [
        "//cyber",
        "//modules/remoteManage/proto:remoteManage_proto",
        "//modules/common/util:message_util",
	"//modules/common/proto:geometry_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/canbus/proto:canbus_proto",
        ],
)

cc_binary(
    name = "lib_remoteManage.so",
    linkopts = ["-shared"],
    linkshared = True,
    linkstatic = False,
    deps = [":remoteManage_lib"],
)

cpplint()
