load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "drivers_tire_pressure_protocol",
    srcs = glob([
        "*.cc",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/proto:tire_pressure_proto",
    ],
)

cpplint()
