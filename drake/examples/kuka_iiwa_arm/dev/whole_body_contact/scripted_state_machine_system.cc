#include "drake/examples/kuka_iiwa_arm/dev/whole_body_contact/scripted_state_machine_system.h"

#include <utility>
#include <vector>

#include "bot_core/robot_state_t.hpp"

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/action_primitives/action_primitives_common.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_common.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/synchronous_world_state.h"
#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/pick_and_place_common.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/action_primitives/action_primitive_base.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/synchronous_world_state.h"
#include "drake/examples/kuka_iiwa_arm/dev/whole_body_contact/whole_body_common.h"

using bot_core::robot_state_t;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {
/* index of iiwastate */
const int kStateIndex = 0;
}

using manipulation::planner::ConstraintRelaxingIk;
using monolithic_pick_and_place::IiwaActionInput;
using monolithic_pick_and_place::SynchronousWorldState;
using monolithic_pick_and_place::kIiwaUrdf;
using monolithic_pick_and_place::kIiwaEndEffectorName;
using monolithic_pick_and_place::ActionPrimitiveState;

namespace whole_body_contact {

namespace {
const Eigen::MatrixXd GenerateJointSequence(){
  Eigen::MatrixXd joint_sequence = Eigen::MatrixXd::Zero(
      4 /* num of sequences */, 7 /* robot DOF */);
  joint_sequence.row(0) = Eigen::VectorXd::Zero(7);
  joint_sequence.row(1) = (
      Eigen::VectorXd() << 0.75 , 1.5, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
  joint_sequence.row(2) = (
      Eigen::VectorXd() << 0.75 , 1.5, 1.5, 1.75, 0.0, -1.5, 0.5).finished();
  joint_sequence.row(3) = (
      Eigen::VectorXd() << 0.90 , 1.5, 1.5, 1.90, 0.0, -1.75, 0.5).finished();

  return joint_sequence;
}
}


struct ScriptedStateMachineSystem::InternalState {
  InternalState() {
    pick_and_place_state = PickAndPlaceState::APPROACH_PICK_PREGRASP;
    iiwa_current_action.is_valid = false;
  }
  ~InternalState() {}

  // This state is used to decide the current state of the
  // finite state machine logic.
  PickAndPlaceState pick_and_place_state;

  // The input to be applied to the IiwaMove ActionPrimitive.
  IiwaActionInput iiwa_current_action;

  // The previous input that was applied to the IiwaMove ActionPrimitive.
  IiwaActionInput previous_action;

  // A logic flag for the validity of the current IiwaActionInput stored
  // within the internal state.
  bool iiwa_action_initiated{false};

//  // Poses used for storing end-points of Iiwa trajectories at various states
//  // of the demo.
//  Isometry3<double> X_WEndEffector0, X_WEndEffector1;
//
//  // Desired object end pose relative to the base of the iiwa arm.
//  Isometry3<double> X_IiwaObj_desired;
//
//  // Desired object end pose in the world frame.
//  Isometry3<double> X_WObj_desired;
};

ScriptedStateMachineSystem::ScriptedStateMachineSystem(
    const Isometry3<double>& iiwa_base, const double update_interval)
    : iiwa_base_(iiwa_base),
      kJointSequence(GenerateJointSequence()),
      planner_(std::make_unique<ConstraintRelaxingIk>(
          drake::GetDrakePath() + kIiwaUrdf, kIiwaEndEffectorName, iiwa_base_)),
      world_state_(
          std::make_unique<SynchronousWorldState>(planner_->get_robot())) {
  input_port_iiwa_state_ = this->DeclareAbstractInputPort().get_index();
  input_port_box_state_ = this->DeclareAbstractInputPort().get_index();
  input_port_iiwa_action_status_ = this->DeclareAbstractInputPort().get_index();
  output_port_iiwa_action_ = this->DeclareAbstractOutputPort().get_index();
  this->DeclarePeriodicUnrestrictedUpdate(update_interval, 0);
}

std::unique_ptr<systems::AbstractValues>
ScriptedStateMachineSystem::AllocateAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals;
  abstract_vals.push_back(std::unique_ptr<systems::AbstractValue>(
      new systems::Value<InternalState>(InternalState())));
  return std::make_unique<systems::AbstractValues>(std::move(abstract_vals));
}

std::unique_ptr<systems::AbstractValue>
ScriptedStateMachineSystem::AllocateOutputAbstract(
    const systems::OutputPortDescriptor<double>& descriptor) const {
  std::unique_ptr<systems::AbstractValue> return_val;
  /* allocate iiwa action and wsg output port */
  if (descriptor.get_index() == output_port_iiwa_action_) {
    return_val =
        systems::AbstractValue::Make<monolithic_pick_and_place::IiwaActionInput>(
            IiwaActionInput());
  }
  return return_val;
}

void ScriptedStateMachineSystem::SetDefaultState(
    const systems::Context<double>&,
    systems::State<double>* state) const {
  InternalState& internal_state =
      state->get_mutable_abstract_state<InternalState>(kStateIndex);

  internal_state.pick_and_place_state = PickAndPlaceState::APPROACH_PICK;
  internal_state.iiwa_current_action.is_valid = false;
  internal_state.iiwa_current_action.q.clear();
  internal_state.iiwa_current_action.time.clear();

//  internal_state.X_WEndEffector0 = Isometry3<double>::Identity();
//  internal_state.X_WEndEffector1 = Isometry3<double>::Identity();
//  internal_state.X_IiwaObj_desired = Isometry3<double>::Identity();
//  internal_state.X_WObj_desired = Isometry3<double>::Identity();
}

void ScriptedStateMachineSystem::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  /* Call actions based on state machine logic */

  IiwaActionInput& iiwa_primitive_input =
      output->GetMutableData(output_port_iiwa_action_)
          ->GetMutableValue<IiwaActionInput>();

  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);

  iiwa_primitive_input = internal_state.iiwa_current_action;
}

void ScriptedStateMachineSystem::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  // Extract Internal state.
  InternalState& internal_state =
      state->get_mutable_abstract_state<InternalState>(kStateIndex);

  IiwaActionInput& iiwa_action_input = internal_state.iiwa_current_action;

  /* Update world state from inputs. */
//  const robot_state_t& iiwa_state =
//      this->EvalAbstractInput(context, input_port_iiwa_state_)
//          ->GetValue<robot_state_t>();
//  const robot_state_t& box_state =
//      this->EvalAbstractInput(context, input_port_box_state_)
//          ->GetValue<robot_state_t>();
  const ActionPrimitiveState& iiwa_primitive_state =
      this->EvalAbstractInput(context, input_port_iiwa_action_status_)
          ->GetValue<ActionPrimitiveState>();
//
//  world_state_->UnpackIiwaStatusMessage(&iiwa_state);
//  world_state_->UnpackObjectStatusMessage(&box_state);

  std::vector<double> times;
  std::vector<Eigen::VectorXd> q_des;

  /* state machine logic */
  switch (internal_state.pick_and_place_state) {
    case PickAndPlaceState::APPROACH_PICK_PREGRASP:
//
      if (!internal_state.iiwa_action_initiated) {
//        // Computes the desired end effector pose in the world frame to be
//        // kPreGraspHeightOffset above the object.
        drake::log()->info("StateMachine : APPROACH_PICK_PREGRASP at {}",
                           context.get_time());

        times.push_back(0);
        times.push_back(2);

        q_des.push_back(kJointSequence.row(0));
        q_des.push_back(kJointSequence.row(1));

        iiwa_action_input.is_valid = true;
        iiwa_action_input.time = times;
        iiwa_action_input.q = q_des;
        internal_state.iiwa_action_initiated = true;

      } else if (iiwa_primitive_state == ActionPrimitiveState::WAITING) {
//        // This means that the action was already intiated and primitive
//        // has returned to WAITING state, i.e. it is complete.
//        // resetting iiwa_action.
//
        times.clear();
        q_des.clear();
        internal_state.iiwa_action_initiated = false;
        internal_state.pick_and_place_state = PickAndPlaceState::APPROACH_PICK;
//      }
      break;
    case PickAndPlaceState::APPROACH_PICK:
      if (!internal_state.iiwa_action_initiated) {
        drake::log()->info("StateMachine : APPROACH_PICK at {}",
                           context.get_time());

        times.push_back(0);
        times.push_back(2);

        q_des.push_back(kJointSequence.row(1));
        q_des.push_back(kJointSequence.row(2));

        internal_state.iiwa_action_initiated = true;
        iiwa_action_input.is_valid = true;
        iiwa_action_input.time = times;
        iiwa_action_input.q = q_des;

      } else if (iiwa_primitive_state == ActionPrimitiveState::WAITING) {
        // This means that the action was already intiated and primitive
        // has returned to WAITING state, i.e. it is complete.
        // resetting iiwa_action.
        times.clear();
        q_des.clear();

        internal_state.iiwa_action_initiated = false;
        internal_state.pick_and_place_state = PickAndPlaceState::GRASP;
      }
      break;
    case PickAndPlaceState::GRASP:
      // change output to gripper action primitive.
      if (!internal_state.iiwa_action_initiated) {
//        // Perform wsg action.
        drake::log()->info("StateMachine : GRASP at {}", context.get_time());

        times.push_back(0);
        times.push_back(2);

        q_des.push_back(kJointSequence.row(2));
        q_des.push_back(kJointSequence.row(3));

        internal_state.iiwa_action_initiated = true;
        iiwa_action_input.is_valid = true;
        iiwa_action_input.time = times;
        iiwa_action_input.q = q_des;
      } else if (iiwa_primitive_state == ActionPrimitiveState::WAITING) {
//        // This means that the action was already intiated and primitive
//        // has returned to WAITING state, i.e. it is complete.
//        // resetting wsg_action.
        internal_state.iiwa_action_initiated = false;
//        internal_state.pick_and_place_state = PickAndPlaceState::LIFT_FROM_PICK;
      }
      break;
  }
}

}  // namespace whole_body_contact
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
