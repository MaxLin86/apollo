load("//tools:cpplint.bzl", "cpplint")
load("//tools:cuda_library.bzl", "cuda_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "darknet_lib",
    srcs = ["darknet_api.cc"],
    hdrs = [
        "darknet_api.h",
    ],
    linkopts = ["-lopencv_core -lnvinfer_plugin -lopencv_imgproc -lopencv_highgui"],
    deps = [
        "//modules/perception/inference:inference_lib",
        "@com_google_protobuf//:protobuf",
       "//modules/perception/inference/darknet/src:darknet_src",
    ],
)


cpplint()
