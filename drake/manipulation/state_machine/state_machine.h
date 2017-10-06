#pragma once

#include <memory>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/manipulation/state_machine/state_machine_utils.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/systems/framework/value.h"
#include "drake/manipulation/state_machine/state_machine_utils.h";
#include "drake/manipulation/state_machine/manipulation_state.h"

namespace drake {
namespace manipulation {
namespace state_machine {

/**
 * Abstract parent to State Machines that describe various
 * manipulation demos.
 */
class StateMachine {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(StateMachine)

  // Default constructor
  StateMachine() {}

  virtual ManipulationState* ComputeStateTransition(
      const ManipulationWorldStatus& manipulation_world_status) = 0;

 private:
  std::vector<std::unique_ptr<ManipulationState>> state_list;
  ManipulationState* last_successful_action_{nullptr};
};

} // namespace state_machine
} // namespace manipulation
} // namespace drake