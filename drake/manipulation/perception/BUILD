# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_googletest",
)

package(
    default_visibility = ["//visibility:public"],
)

drake_cc_library(
    name = "optitrack_pose_extractor",
    srcs = ["optitrack_pose_extractor.cc"],
    hdrs = ["optitrack_pose_extractor.h"],
    deps = [
        "//drake/math:geometric_transform",
        "//drake/systems/framework",
        "@optitrack_driver//lcmtypes:optitrack_lcmtypes",
    ],
)

drake_cc_library(
    name = "pose_smoother",
    srcs = ["pose_smoother.cc"],
    hdrs = ["pose_smoother.h"],
    deps = [
        "//drake/manipulation/util:moving_average_filter",
        "//drake/math:geometric_transform",
        "//drake/systems/framework",
    ],
)

drake_cc_googletest(
    name = "optitrack_pose_extractor_test",
    deps = [
        ":optitrack_pose_extractor",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

drake_cc_googletest(
    name = "pose_smoother_test",
    deps = [
        ":pose_smoother",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

cpplint()
