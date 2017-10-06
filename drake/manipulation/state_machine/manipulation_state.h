#pragma once

#include "drake/manipulation/state_machine/state_machine_utils.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"

namespace drake {
namespace manipulation {
namespace state_machine {

/**
 * Abstract Parent to each of the abstract states that are composed
 * into a StateMachine.
 */
class ManipulationState {

 public:
  ManipulationState() { }

  typedef std::function<void(
  const PiecewisePolynomialTrajectory*)> CalcManipulatorPlan;
  typedef std::function<void(
      const GripperStatus*)> CalcGripperPlan;

  virtual void Execute(
      const ManipulationWorldStatus& manipulation_world_status,
      const CalcManipulatorPlan& manipulator_callback,
      const CalcGripperPlan& gripper_callback) = 0;
};

} // namespace state_machine
} // namespace manipulation
} // namespace drake