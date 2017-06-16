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
    #        "//drake/lcmtypes:viewer",
    #        "//drake/multibody/rigid_body_plant:create_load_robot_message",
    #        "//drake/multibody/rigid_body_plant:drake_visualizer",
    #        "//drake/systems/rendering:drake_visualizer_client",
            "//drake/multibody/rigid_body_plant",
    ],
)

drake_cc_library(
    name = "iiwa_and_object_state_aggregator",
    srcs = ["iiwa_and_object_state_aggregator.cc"],
    hdrs = ["iiwa_and_object_state_aggregator.h"],
    visibility = ["//drake/examples/kuka_iiwa_arm:__subpackages__"],
    deps = [
        ":geometric_contact_state_estimator",
        "//drake/systems/framework",
        "@optitrack_driver//lcmtypes:optitrack_lcmtypes", 
        "@bot_core_lcmtypes",
    ],
)

drake_cc_binary(
    name = "contact_visualization_demo",
    srcs = ["contact_visualization_demo.cc"],
    data = [
            "//drake/examples/kuka_iiwa_arm:models",
            "//drake/manipulation/models/iiwa_description:models",            
        ],
    deps = [
        ":iiwa_and_object_state_aggregator",
        ":geometric_contact_state_estimator",
        "//drake/lcm",
        "//drake/lcmtypes:iiwa",
        "@bot_core_lcmtypes",
        "@com_github_gflags_gflags//:gflags",        
    ]
)