#pragma once

#include <vector>

#include "drake/manipulation/state_machine/state_machine_utils.h"
#include "drake/manipulation/state_machine/state_machine.h"
#include "drake/manipulation/state_machine/manipulation_state.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace nuevo_pick_and_place {
using manipulation::state_machine::StateMachine;
using manipulation::state_machine::ManipulationState;
using manipulation::state_machine::ManipulationWorldStatus;

class PickAndPlaceStateMachine : public StateMachine {

  PickAndPlaceStateMachine(
      const std::vector<Isometry3<double>>& place_locations,
      bool loop);

  std::unique_ptr<StateMachine> Clone() {
    std::make_unique<PickAndPlaceStateMachine>(place_locations_, loop_);
  }

  ManipulationState* ComputeStateTransition(
      const ManipulationWorldStatus& manipulation_world_status,
      double current_time);

 private:
  std::vector<Isometry3<double>> place_locations_{};
  bool loop_{false};
  std::string current_state_{};
};

} // namespace nuevo_pick_and_place
} // namespace kuka_iiwa_arm
} // namespace examples
} // namespace drake
