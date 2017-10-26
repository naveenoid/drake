#include "drake/manipulation/state_machine/state_machine_system.h"

#include <utility>
#include <vector>
#include <memory>

#include "drake/systems/framework/abstract_values.h"
#include "drake/common/copyable_unique_ptr.h"

namespace drake {
namespace manipulation {
namespace state_machine {

struct StateMachineSystem::InternalState {
  InternalState(std::unique_ptr<StateMachine> state_machine) :
    state_machine_(std::move(state_machine)) {
  }

  ManipulationWorldStatus world_status_{};
  ManipulationWorldCommand world_command_{};
  copyable_unique_ptr<StateMachine> state_machine_;
};

using systems::AbstractValue;

StateMachineSystem::StateMachineSystem(
    std::unique_ptr<StateMachine> manipulation_state_machine,
    const double period_sec) :
    input_port_manipulator_status_(this->DeclareAbstractInputPort().get_index()),
    input_port_target_status_(this->DeclareAbstractInputPort().get_index()),
    input_port_gripper_status_(this->DeclareAbstractInputPort().get_index()),
    output_port_manipulator_plan_(
      this->DeclareAbstractOutputPort(
              PiecewisePolynomialTrajectory(PiecewisePolynomial<double>()),
              &StateMachineSystem::CalcManipulatorPlan)
          .get_index()),
    output_port_gripper_plan_(
      this->DeclareAbstractOutputPort(
              &StateMachineSystem::CalcGripperPlan)
          .get_index()) {
  this->DeclareAbstractState(
      AbstractValue::Make<InternalState>(
          InternalState(std::move(manipulation_state_machine))));
  this->DeclarePeriodicUnrestrictedUpdate(period_sec, 0);
}

void StateMachineSystem::CalcManipulatorPlan(
    const systems::Context<double>& context,
    PiecewisePolynomialTrajectory* manipulator_plan) const {

  /* Call actions based on state machine logic */
  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);
  *manipulator_plan = internal_state.world_command_.manipulator_plan_;
}

void StateMachineSystem::CalcGripperPlan(
    const systems::Context<double>& context,
    GripperCommand * gripper_plan) const {

  /* Call actions based on state machine logic */
  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);
  *gripper_plan = internal_state.world_command_.gripper_plan_;
}

void StateMachineSystem::DoCalcUnrestrictedUpdate(
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
  const PoseBundle<double>& target_status =
      this->EvalAbstractInput(context, input_port_target_status_)
          ->GetValue<PoseBundle<double>>();

  internal_state.world_status_.robot_status_ = manipulator_status;
  internal_state.world_status_.gripper_status_= gripper_status;
  internal_state.world_status_.targets_ = target_status;

  ManipulationState* next_state = internal_state.state_machine_->ComputeStateTransition(
      internal_state.world_status_);

  next_state->Execute(internal_state.world_status_, context.get_time(), &internal_state.world_command_);

  if(internal_state.world_command_.is_new_manipulator_plan_) {
    internal_state.world_command_.manipulator_plan_.shiftRight(context.get_time());
    // reset internal flag for new plan.
    internal_state.world_command_.is_new_manipulator_plan_ = false;

  }
}

}  // namespace state_machine
}  // namespace manipulation
}  // namespace drake
