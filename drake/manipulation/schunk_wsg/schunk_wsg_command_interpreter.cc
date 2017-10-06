#include "drake/manipulation/schunk_wsg/schunk_wsg_command_interpreter.h"

#include "drake/manipulation/state_machine/state_machine_utils.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {
using state_machine::GripperCommand;

SchunkWsgCommandInterpreter::SchunkWsgCommandInterpreter(
    double opening_width, double closing_width) :
    opening_width_(opening_width), closing_width_(closing_width),
    input_port_gripper_command_(this->DeclareAbstractInputPort().get_index()),
    output_port_gripper_plan_(this->DeclareVectorOutputPort(
        &SchunkWsgCommandInterpreter::CalcGripperPlan).get_index()) {
  // Sanity Check.
  DRAKE_DEMAND(opening_width_ > closing_width_);
}

void SchunkWsgCommandInterpreter::CalcGripperPlan(
    const Context<double>& context, VectorX<double>* gripper_plan) const {
  VectorX<double> output_vector = VectorX<double>::Zero(1);

  const GripperCommand& current_command = this->EvalAbstractInput(
      context, input_port_gripper_command_)->GetValue<GripperCommand>();

  switch(current_command) {
    case GripperCommand::Open :
      output_vector[0] = opening_width_;
      break;
    case GripperCommand::Close :
      output_vector[0] = closing_width_;
      break;
  }

  *gripper_plan = output_vector;
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
