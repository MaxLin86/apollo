load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tire_pressure_message_manager",
    srcs = ["tire_pressure_message_manager.cc"],
    hdrs = ["tire_pressure_message_manager.h"],
    deps = [
        "//modules/common/util:message_util",
        "//modules/common/time",
        "//modules/drivers/canbus:sensor_gflags",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        #"//modules/drivers/proto:sensor_proto",
        "//modules/drivers/tire_pressure/protocol:drivers_tire_pressure_protocol",
        "//modules/common/adapters:adapter_gflags",
        "//modules/drivers/proto:tire_pressure_proto",
    ],
)

cc_library(
    name = "tire_pressure_canbus_lib",
    srcs = ["tire_pressure_canbus.cc"],
    hdrs = ["tire_pressure_canbus.h"],
    deps = [
        ":tire_pressure_message_manager",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/drivers/canbus:sensor_gflags",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        #"//modules/drivers/proto:sensor_proto",
        "//modules/drivers/tire_pressure/proto:tire_pressure_conf_proto",
        "//modules/drivers/proto:tire_pressure_proto",
        
    ],
)

cc_library(
    name = "tire_pressure_canbus_component_lib",
    srcs = ["tire_pressure_canbus_component.cc"],
    hdrs = ["tire_pressure_canbus_component.h"],
    copts = ['-DMODULE_NAME=\\"ultrasonic_radar_canbus\\"'],
    deps = [
        ":tire_pressure_canbus_lib",
        "//modules/common/adapters:adapter_gflags",
        "//modules/drivers/tire_pressure/protocol:drivers_tire_pressure_protocol",
        "//modules/drivers/proto:tire_pressure_proto",
    ],
)

cc_binary(
    name = "libtire_pressure_canbus_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":tire_pressure_canbus_component_lib"],
)

cpplint()
