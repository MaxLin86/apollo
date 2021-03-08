load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "self_car_tracking_lib",
    srcs = ["self_car_tracking.cc"],
    hdrs = ["self_car_tracking.h"],
    copts = ["-DMODULE_NAME=\\\"localization\\\"" ],
   deps = [
        "//modules/common/filters:self_car_localization_ekf",
        "//modules/common/filters:gps_kalman_filter",
        "//modules/common/configs:vehicle_config_helper",
        # "//modules/drivers/canbus/proto:canbus_proto",

        "//modules/canbus/proto:canbus_proto",
        "//modules/common/proto:drive_state_proto",
        "//modules/common/proto:geometry_proto",
        "//modules/common/proto:vehicle_signal_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/common/time",
    ],
)

cc_library(
    name = "tracking_object_lib",
    srcs = ["tracking_object.cc"],
    hdrs = ["tracking_object.h"],
    copts = ["-DMODULE_NAME=\\\"map\\\"" ],
   deps = [
        "//modules/common/filters:ins_guidepost_fusion_ekf",
        "//modules/common/configs:vehicle_config_helper",
        # "//modules/drivers/canbus/proto:canbus_proto",

        "//modules/canbus/proto:canbus_proto",
        "//modules/common/proto:drive_state_proto",
        "//modules/common/proto:geometry_proto",
        "//modules/common/proto:vehicle_signal_proto",

        "//modules/localization/proto:localization_proto",
         "//modules/routing/proto:routing_proto",
    ],
)

cc_library(
    name = "foo_localization_component_lib",
    srcs = [
        "foo_localization.cc",
        "foo_localization_component.cc",
    ],
    hdrs = [
        "foo_localization.h",
        "foo_localization_component.h",
    ],
    copts = ["-DMODULE_NAME=\\\"localization\\\""],
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/common/monitor_log",
        "//modules/common/proto:geometry_proto",
        "//modules/common/time",
        "//modules/common/util:message_util",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/localization/proto:gps_proto",
        "//modules/localization/proto:imu_proto",
        "//modules/drivers/gnss/proto:imu_proto",
        "//modules/localization/proto:localization_proto",

        "//modules/routing/proto:routing_proto",
        "//modules/localization/proto:foo_config_proto",
        "//modules/transform:transform_broadcaster_lib",
        "@eigen",
        "@gtest",
        "//modules/localization/foo/imu_gps_localizer:imu_gps_localizer",
        "//modules/common/util",

        "//modules/map/hdmap:hdmap_util",
        "//modules/map/pnc_map",
        ":self_car_tracking_lib",
        ":tracking_object_lib",
        "//modules/localization/msf/common/util:localization_msf_common_util_frame_transform",
    ],
)

cc_binary(
    name = "libfoo_localization_component.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":foo_localization_component_lib"],
)

cc_test(
    name = "gps_to_utm_test",
    size = "small",
    timeout = "short",
    srcs = ["gps_to_utm_test.cc"],
    copts = [
        "-Dprivate=public",
    ],
    data = ["//modules/localization:localization_testdata"],
    deps = [
        "//cyber",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/localization/foo:foo_localization_component_lib",
        "//modules/localization/msf/common/util:localization_msf_common_util_frame_transform",
        "@gtest//:main",
    ],
)

cpplint()
