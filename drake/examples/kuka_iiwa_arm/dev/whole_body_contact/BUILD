# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
)


package(
    default_visibility = ["//drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place:__subpackages__"],
)

drake_cc_library(
    name = "kuka_whole_body_manipulation",
    srcs = [
#        "demo_diagram_builder.cc",
#        "whole_body_state_machine_system.cc",
#        "synchronous_world_state.cc",
    ],
    hdrs = [
        "demo_diagram_builder.h",
#        "pick_and_place_common.h",
#        "state_machine_system.h",
#        "synchronous_world_state.h",
    ],
    deps = [
#        "//drake/examples/kuka_iiwa_arm:iiwa_common",
#        "//drake/examples/kuka_iiwa_arm:robot_plan_interpolator",
#        "//drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/action_primitives",
#        "//drake/examples/kuka_iiwa_arm/dev/pick_and_place:kuka_pick_and_place",
      #  "//drake/examples/kuka_iiwa_arm/iiwa_world:iiwa_wsg_diagram_factory",
        "//drake/examples/kuka_iiwa_arm/iiwa_world:world_sim_tree_builder",
#        "//drake/examples/schunk_wsg:schunk_wsg_lcm",
#        "//drake/lcm",
#        "//drake/lcmtypes:iiwa",
        "//drake/multibody:rigid_body_tree",
#        "//drake/multibody/parsers",
    ],
)

drake_cc_binary(
    name = "demo_tree_visual_test",
    srcs = ["demo_tree_visual_test.cc"],
    data = [
            "//drake/examples/kuka_iiwa_arm:models",
            "//drake/manipulation/models/iiwa_description:models",
        ],
        test_rule_args = ["--num_configurations=1"],
        deps = [
            ":kuka_whole_body_manipulation",
            "//drake/examples/kuka_iiwa_arm/dev/tools:simple_tree_visualizer",
            "//drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place:kuka_pick_and_place_monolithic",
            "//drake/common",
            "//drake/lcm",
            "//drake/multibody/parsers",
            "//drake/systems/framework",
            "@gflags",
        ],
)