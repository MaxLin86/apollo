load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "benewake",
    srcs = ["benewake.cc"],
    hdrs = ["benewake.h"],
    copts = ['-DMODULE_NAME=\\"lidar\\"'],
    deps = [
        "//cyber", 
        "//modules/lidar/proto:lidar_proto",
        "//modules/led/proto:led_proto",
	"//modules/common/util:message_util",
    ]
)

cc_library(
    name = "lidar_component",
    srcs = ["lidar_component.cc"],
    hdrs = ["lidar_component.h"],
    copts = ['-DMODULE_NAME=\\"lidar\\"'],
    deps = [
        "//cyber", 
        ":benewake" ,
    ],
)

cc_binary(
    name = "lidar.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":lidar_component",
    ],
)

cpplint()
