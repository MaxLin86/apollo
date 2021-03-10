load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
        name = "lib_rms.cc",
        srcs = ["rms.cc"],
        hdrs = [
                "rms.h",
        ],
	copts = ['-DMODULE_NAME=\\"rms\\"'],
        deps = [
                "//cyber",
                "//modules/rms/proto:rms_cc_proto",
                "//modules/rms/proto:rms_conf_cc_proto",
                "//modules/common/time:time",
                "//modules/common/util:message_util",
                "//modules/common/adapters:adapter_gflags",
                "//modules/common/proto:geometry_proto",
                "//modules/canbus/proto:canbus_proto",
                "//modules/guardian/proto:guardian_proto",
		"//modules/tmc/proto:tmc_proto",
                "//modules/manuctrl/proto:manuctrl_proto",
                "//modules/urgencystop/proto:aeb_cmd_cc_proto",
                "//modules/drivers/proto:sensor_proto",
                "//modules/drivers/gnss/proto:gnss_proto",
                "//modules/planning/proto:planning_proto",
                "//modules/localization/proto:pose_proto",
                "//modules/map/relative_map/proto:navigation_proto",
                "//modules/localization/msf/common/util:localization_msf_common_util_frame_transform",
        ],
)

cc_binary(
        name = "lib_rms.so",
        linkopts = ["-shared"],
        linkshared = True,
        linkstatic = False,
        deps = [
                ":lib_rms.cc",
        ],
)

cpplint()
