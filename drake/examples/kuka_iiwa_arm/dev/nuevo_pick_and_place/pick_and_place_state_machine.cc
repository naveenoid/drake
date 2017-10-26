#include "drake/examples/kuka_iiwa_arm/dev/nuevo_pick_and_place/pick_and_place_state_machine.h"

#include <map>
#include <memory>

#include "drake/manipulation/state_machine/manipulation_state.h"
#include "drake/manipulation/state_machine/state_machine_utils.h"
#include "drake/examples/kuka_iiwa_arm/dev/nuevo_pick_and_place/pick_and_place_states.h"
#include "drake/common/copyable_unique_ptr.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace nuevo_pick_and_place {
namespace {
std::map<std::string, copyable_unique_ptr<ManipulationState>> BuildStates() {
  std::map<std::string, copyable_unique_ptr<ManipulationState>> state_name_map;

  std::unique_ptr<ManipulationState> open_gripper =
      std::make_unique<ManipulationState>(OpenGripper());
  state_name_map.insert(
      std::pair<std::string,copyable_unique_ptr<ManipulationState>>(
          "OpenGripper",  open_gripper));


};


} // namespace

using manipulation::state_machine::ManipulationState;
using manipulation::state_machine::ManipulationWorldStatus;



PickAndPlaceStateMachine::PickAndPlaceStateMachine(
    const std::vector<Isometry3<double>>& place_locations, bool loop) :
  place_locations_(place_locations), loop_(loop) {



}

ManipulationState* PickAndPlaceStateMachine::ComputeStateTransition(
    const ManipulationWorldStatus& manipulation_world_status,
    double current_time) {

}

} // namespace nuevo_pick_and_place
} // namespace kuka_iiwa_arm
} // namespace examples
} // namespace drake