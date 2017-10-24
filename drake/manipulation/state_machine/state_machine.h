#pragma once

#include <memory>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/manipulation/state_machine/state_machine_utils.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/manipulation/state_machine/state_machine_utils.h"
#include "drake/manipulation/state_machine/manipulation_state.h"
#include "drake/common/copyable_unique_ptr.h"

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

  virtual std::unique_ptr<StateMachine> Clone() const = 0;

  virtual ManipulationState* ComputeStateTransition(
      const ManipulationWorldStatus& manipulation_world_status) = 0;

  virtual ~StateMachine() = 0;
 protected:
  std::vector<copyable_unique_ptr<ManipulationState>> state_list;
  ManipulationState* current_state_{nullptr};
  int current_state_id_{-1};
};

} // namespace state_machine
} // namespace manipulation
} // namespace drake