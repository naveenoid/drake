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
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ManipulationState)

  ManipulationState() { }

  virtual std::unique_ptr<ManipulationState> Clone() const = 0;

  virtual ~ManipulationState() = 0;

  virtual void Execute(
      const ManipulationWorldStatus& manipulation_world_status,
      ManipulationWorldCommand* manipulation_world_command) = 0;

  bool IsActive() { return is_active_; }

 protected:
  bool is_active_{false};

};

} // namespace state_machine
} // namespace manipulation
} // namespace drake