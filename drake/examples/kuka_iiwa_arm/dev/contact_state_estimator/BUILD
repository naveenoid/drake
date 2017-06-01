# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_library",
    "drake_cc_googletest",
)

drake_cc_library(
    name = "geometric_contact_state_estimator",
    srcs = ["geometric_contact_state_estimator.cc"],
    hdrs = ["geometric_contact_state_estimator.h"],
    visibility = ["//drake/examples/kuka_iiwa_arm:__subpackages__"],
    deps = [
            "//drake/common",
            "//drake/lcm",
            "//drake/lcmtypes:viewer",
            "//drake/multibody/rigid_body_plant:create_load_robot_message",
            "//drake/multibody/rigid_body_plant:drake_visualizer",
            "//drake/systems/framework",
            "//drake/systems/rendering:drake_visualizer_client",
            "//drake/multibody/rigid_body_plant",
    ],
)
