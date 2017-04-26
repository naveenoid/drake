# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_googletest",
)

# === test/ ===

drake_cc_googletest(
    name = "optitrak_driver_test",
    srcs = ["optitrack_test.cc"],
    deps = [
        "@optitrack_driver//lcmtypes:optitrack_lcmtypes",
        "@gflags//:gflags",
    ],
)
