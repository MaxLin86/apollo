load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "imu_gps_localizer",
    srcs = [
        "imu_gps_localizer.cc",
        "gps_processor.cc",
        "imu_processor.cc",
        "initializer.cc",
    ],
    hdrs = [
        "base_type.h",
        "imu_gps_localizer.h",
        "gps_processor.h",
        "imu_processor.h",
        "initializer.h",
        "utils.h",
    ],
    copts = ["-DMODULE_NAME=\\\"localization\\\""],
    deps = [
        "//cyber",
        
        "//modules/common/math",
        "//modules/common/status",
        #"//modules/common/util",

        "@eigen",
        #"@glog",
        "//modules/localization/foo/GeographicLib:geographic_lib",
        "//modules/localization/msf/common/util:localization_msf_common_util_frame_transform",
        
    ],
)


cpplint()
