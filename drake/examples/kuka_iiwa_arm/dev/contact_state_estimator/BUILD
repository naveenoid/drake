# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_library",
    "drake_cc_googletest",
)
#
#drake_cc_library(
#    name = "geometric_contact_state_estimator",
#    srcs = ["geometric_contact_state_estimator.cc"],
#    hdrs = ["geometric_contact_state_estimator.h"],
#    visibility = ["//drake/examples/kuka_iiwa_arm:__subpackages__"],
#    deps = [
#        "//drake/common",
#        "//drake/lcm",
#        "//drake/multibody/rigid_body_plant",
#    ],
#)

drake_cc_library(
    name = "iiwa_and_object_state_aggregator",
    srcs = ["iiwa_and_object_state_aggregator.cc"],
    hdrs = ["iiwa_and_object_state_aggregator.h"],
    visibility = ["//drake/examples/kuka_iiwa_arm:__subpackages__"],
    deps = [
        #        ":geometric_contact_state_estimator",
        "//drake/lcmtypes:iiwa",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/rigid_body_plant:compliant_contact_model",
        "//drake/systems/framework",
        # "@bot_core_lcmtypes",
        "@optitrack_driver//lcmtypes:optitrack_lcmtypes",
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
        "//drake/examples/kuka_iiwa_arm:iiwa_lcm",
        "//drake/examples/kuka_iiwa_arm/iiwa_world:world_sim_tree_builder",
        "//drake/lcm",
        "//drake/lcmtypes:contact_info_for_viz",
        "//drake/lcmtypes:contact_results_for_viz",
        "//drake/lcmtypes:iiwa",
        "//drake/manipulation/perception:optitrack_pose_extractor",
        "//drake/manipulation/perception:pose_smoother",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/rigid_body_plant:contact_results_to_lcm",
        "//drake/systems/lcm:lcm_driven_loop",
        "@com_github_gflags_gflags//:gflags",
        "@optitrack_driver//lcmtypes:optitrack_lcmtypes",
    ],
)

drake_cc_binary(
    name = "optitrack_pose_identification_demo",
    srcs = ["optitrack_pose_identification_demo.cc"],
    data = [
        "//drake/examples/kuka_iiwa_arm:models",
        "//drake/manipulation/models/iiwa_description:models",
    ],
    deps = [
        ":iiwa_and_object_state_aggregator",
        "//drake/examples/kuka_iiwa_arm:iiwa_lcm",
        "//drake/examples/kuka_iiwa_arm/iiwa_world:world_sim_tree_builder",
        "//drake/lcm",
        "//drake/lcmtypes:contact_info_for_viz",
        "//drake/lcmtypes:contact_results_for_viz",
        "//drake/lcmtypes:iiwa",
        "//drake/manipulation/perception:optitrack_pose_extractor",
        "//drake/manipulation/perception:pose_smoother",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/rigid_body_plant:contact_results_to_lcm",
        "//drake/systems/lcm:lcm_driven_loop",
        "@com_github_gflags_gflags//:gflags",
        "@optitrack_driver//lcmtypes:optitrack_lcmtypes",
    ],
)

cpplint()
