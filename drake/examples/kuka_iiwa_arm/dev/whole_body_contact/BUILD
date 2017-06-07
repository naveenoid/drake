# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
)


package(
    default_visibility = ["//drake/examples/kuka_iiwa_arm/dev:__subpackages__"],
)

drake_cc_library(
    name = "kuka_whole_body_manipulation",
    srcs = [
        "demo_diagram_builder.cc",
        "scripted_state_machine_system.cc",
        "iiwa_diagram_factory.h",
#        "synchronous_world_state.cc",
    ],
    hdrs = [
        "demo_diagram_builder.h",
#        "pick_and_place_common.h",
        "scripted_state_machine_system.h",
        "whole_body_common.h",
        "iiwa_diagram_factory.cc",
#        "synchronous_world_state.h",
    ],
    deps = [
#        "//drake/examples/kuka_iiwa_arm:iiwa_common",
#        "//drake/examples/kuka_iiwa_arm:robot_plan_interpolator",
#        "//drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/action_primitives",
#        "//drake/examples/kuka_iiwa_arm/dev/pick_and_place:kuka_pick_and_place",
      #  "//drake/examples/kuka_iiwa_arm/iiwa_world:iiwa_wsg_diagram_factory",
        "//drake/examples/kuka_iiwa_arm/iiwa_world:world_sim_tree_builder",
        "//drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place:kuka_pick_and_place_monolithic",
#        "//drake/examples/schunk_wsg:schunk_wsg_lcm",
        "//drake/lcm",
        "//drake/lcmtypes:iiwa",
        "@bot_core_lcmtypes",
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
            "@com_github_gflags_gflags//:gflags",
        ],
)

drake_cc_binary(
    name = "whole_body_contact_manipulation_demo",
    srcs = ["whole_body_contact_manipulation_demo.cc"],
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
            "@com_github_gflags_gflags//:gflags",
        ],
)