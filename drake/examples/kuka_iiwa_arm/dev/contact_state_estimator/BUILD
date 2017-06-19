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
            "//drake/multibody/rigid_body_plant",
    ],
)

drake_cc_library(
    name = "optitrack_pose_extractor",
    srcs = ["optitrack_pose_extractor.cc"],
    hdrs = ["optitrack_pose_extractor.h"],
    visibility = ["//drake/examples/kuka_iiwa_arm:__subpackages__"],
        deps = [
            "//drake/systems/framework",
            "@optitrack_driver//lcmtypes:optitrack_lcmtypes", 
            "@bot_core_lcmtypes",
            "//drake/examples/kuka_iiwa_arm/dev/tools:optitrack_pose_translator",
        ],
)

drake_cc_library(
    name = "iiwa_and_object_state_aggregator",
    srcs = ["iiwa_and_object_state_aggregator.cc"],
    hdrs = ["iiwa_and_object_state_aggregator.h"],
    visibility = ["//drake/examples/kuka_iiwa_arm:__subpackages__"],
    deps = [
        ":geometric_contact_state_estimator",
        ":optitrack_pose_extractor",
        "//drake/systems/framework",
        "@optitrack_driver//lcmtypes:optitrack_lcmtypes", 
        "@bot_core_lcmtypes",
        "//drake/lcmtypes:iiwa",
        "//drake/multibody:rigid_body_tree",
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
        "//drake/examples/kuka_iiwa_arm:iiwa_lcm",
        ":iiwa_and_object_state_aggregator",
        ":geometric_contact_state_estimator",
        "//drake/lcm",
        "//drake/lcmtypes:iiwa",
        "//drake/systems/lcm:lcm_driven_loop",
        "@bot_core_lcmtypes",
        "@com_github_gflags_gflags//:gflags", 
        "@optitrack_driver//lcmtypes:optitrack_lcmtypes",
        "//drake/examples/kuka_iiwa_arm/iiwa_world:world_sim_tree_builder",
        "//drake/multibody/rigid_body_plant:contact_results_to_lcm",
        "//drake/lcmtypes:contact_info_for_viz",
        "//drake/lcmtypes:contact_results_for_viz",
        "//drake/multibody:rigid_body_tree",
    ]
)

drake_cc_binary(
    name = "robot_visualization_demo",
    srcs = ["robot_visualization_demo.cc"],
    data = ["//drake/examples/kuka_iiwa_arm:models"],
    deps = [
        "//drake/examples/kuka_iiwa_arm:iiwa_lcm",
        "//drake/lcm",
        "//drake/lcmtypes:iiwa",
        "//drake/systems/lcm:lcm_driven_loop",
        "@bot_core_lcmtypes",
        "@com_github_gflags_gflags//:gflags", 
        "//drake/examples/kuka_iiwa_arm/iiwa_world:world_sim_tree_builder",
    ]
)


drake_cc_binary(
    name = "box_pose_visualization_demo",
    srcs = ["box_pose_visualization_demo.cc"],
    data = ["//drake/manipulation/models/iiwa_description:models"],
    deps = [
        "//drake/examples/kuka_iiwa_arm:iiwa_lcm",
        "//drake/lcm",
        "//drake/lcmtypes:iiwa",
        "//drake/systems/lcm:lcm_driven_loop",
        ":optitrack_pose_extractor",
        "@bot_core_lcmtypes",
        "@com_github_gflags_gflags//:gflags", 
        "@optitrack_driver//lcmtypes:optitrack_lcmtypes",
        "//drake/examples/kuka_iiwa_arm/iiwa_world:world_sim_tree_builder",
    ]
)
