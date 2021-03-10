package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "tire_pressure_conf_proto",
    deps = [
        ":tire_pressure_proto_lib",
    ],
)

proto_library(
    name = "tire_pressure_proto_lib",
    srcs = ["tire_pressure_conf.proto"],
    deps = [
        "//modules/drivers/canbus/proto:canbus_proto_lib",
    ],
)
