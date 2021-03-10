load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "serial",
    srcs = ["serial.cc"],
    hdrs = ["serial.h"],
    copts = ['-DMODULE_NAME=\\"led\\"'],
    deps = [
        "//cyber", 
        "//modules/led/proto:led_proto",
	"//modules/common/util:message_util",
    ]
)

cc_library(
    name = "led_component",
    srcs = ["led_component.cc"],
    hdrs = ["led_component.h"],
    copts = ['-DMODULE_NAME=\\"led\\"'],
    deps = [
        "//cyber", 
        ":serial" ,
    ],
)

cc_binary(
    name = "led.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":led_component",
    ],
)

cpplint()
