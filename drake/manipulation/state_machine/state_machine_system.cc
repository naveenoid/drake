#include "drake/manipulation/state_machine/state_machine_system.h"

#include <utility>
#include <vector>
#include <memory>

#include "drake/systems/framework/abstract_values.h"

namespace drake {
namespace manipulation {
namespace state_machine {
using systems::AbstractValues;
using systems::AbstractValue;
namespace {
struct InternalState {
  InternalState(std::unique_ptr<StateMachine> state_machine) :
    manipulator_plan_(PiecewisePolynomial()),
    state_machine_(std::move(state_machine)) {
  }

  ManipulationWorldStatus world_status_{};
  PiecewisePolynomialTrajectory manipulator_plan_{
      PiecewisePolynomial()};
  bool is_new_manipulation_plan_{true};
  GripperCommand gripper_plan_{GripperCommand::Open};
  bool is_new_gripper_plan_{true};
  std::unique_ptr<StateMachine> state_machine_;
};
}

StateMachineSystem::StateMachineSystem(
    std::unique_ptr<StateMachine> manipulation_state_machine,
    const double period_sec)
    :
  input_port_manipulator_status_(this->DeclareAbstractInputPort().get_index()),
  input_port_gripper_status_(this->DeclareAbstractInputPort().get_index()),
  input_port_target_status_(this->DeclareAbstractInputPort().get_index()),
  output_port_manipulator_plan_(
      this->DeclareAbstractOutputPort(
              &StateMachineSystem::CalcManipulatorPlan)
          .get_index()),
  output_port_gripper_plan_(
      this->DeclareAbstractOutputPort(
              &StateMachineSystem::CalcGripperPlan)
          .get_index()) {

  this->DeclareAbstractState(
      AbstractValue::Make<InternalState>(
          InternalState(manipulation_state_machine));
  this->DeclarePeriodicUnrestrictedUpdate(period_sec, 0);
}

void StateMachineSystem::CalcManipulatorPlan(
    const systems::Context<double>& context,
    PiecewisePolynomialTrajectory* manipulator_plan) const {
  /* Call actions based on state machine logic */

  // Check plan status bit.

  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);
  *manipulator_plan = internal_state.manipulator_plan_;
}


void StateMachineSystem::CalcGripperPlan(
    const systems::Context<double>& context,
    GripperCommand * gripper_plan) const {
  /* Call actions based on state machine logic */

  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);
  *gripper_plan = internal_state.gripper_plan_;
}

void StateMachineSystem::DoCalcUnrestrictedUpate(
    const systems::Context<double> &context,
    const std::vector<const systems::UnrestrictedUpdateEvent<double> *> &,
    systems::State<double> *state) const {

  InternalState& internal_state =
      state->get_mutable_abstract_state<InternalState>(0);

  /* Update world state from inputs. */
  const VectorX<double>& manipulator_status =
      this->EvalAbstractInput(context, input_port_manipulator_status_)
          ->GetValue<VectorX<double>>();
  const GripperStatus& gripper_status =
      this->EvalAbstractInput(context, input_port_gripper_status_)
          ->GetValue<GripperStatus>();
  const PoseBundle& target_status =
      this->EvalAbstractInput(context, input_port_target_status_)
          ->GetValue<PoseBundle>();

    ManipulationState::CalcManipulatorPlan manipulator_callback =
      ([&](const PiecewisePolynomialTrajectory* plan) {
        internal_state.manipulator_plan_ = *plan;
      });

  ManipulationState::CalcGripperPlan gripper_callback =
      ([&](const GripperStatus* msg) {
        internal_state.gripper_plan_ = *msg;
      });

  ManipulationState* next_state = internal_state.state_machine_->ComputeStateTransition(
      internal_state.world_status_);

}



}  // namespace state_machine
}  // namespace manipulation
}  // namespace drake
