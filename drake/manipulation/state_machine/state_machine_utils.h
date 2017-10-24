#pragma once

#include "drake/systems/rendering/pose_bundle.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"

namespace drake {
namespace manipulation {
namespace state_machine {
using systems::rendering::PoseBundle;

enum GripperStatus : bool {
  Closed = true,
  Opened = false
};

enum GripperCommand : bool {
  Close = true,
  Open = false
};

struct ManipulationWorldStatus {
  VectorX<double> robot_status_{};
  GripperStatus gripper_status_{
      GripperStatus::Closed };
  PoseBundle<double> targets_{0};
  int state_id_{0};
};

struct ManipulationWorldCommand {
  PiecewisePolynomialTrajectory manipulator_plan_{
      PiecewisePolynomial<double>()};
  bool is_new_manipulator_plan_{false};
  GripperCommand gripper_plan_{};
};

}
}
}