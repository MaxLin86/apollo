package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "remoteManage_proto",
    deps = [
        ":remoteManage_proto_lib",
    ],
)

proto_library(
    name = "remoteManage_proto_lib",
    srcs = ["remoteManage.proto"],
    deps = [
	    "//modules/common/proto:header_proto_lib",
	    "//modules/common/proto:geometry_proto_lib",
    ],
)