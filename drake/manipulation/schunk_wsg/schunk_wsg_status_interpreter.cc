#include "drake/manipulation/schunk_wsg/schunk_wsg_status_interpreter.h"


namespace drake {
namespace manipulation {
namespace schunk_wsg {

SchunkWsgStatusInterpreter::SchunkWsgStatusInterpreter(
    double open_width_threshold, double closing_width) :
    opening_width_(open_width_threshold), closing_width_(closing_width),
    input_port_state_(this->DeclareVectorInputPort(5).get_index()),
    output_port_status_(this->DeclareAbstractOutputPort(
        &SchunkWsgStatusInterpreter::CalcGripperStatus).get_index()) {
  // Sanity Check.
  DRAKE_DEMAND(opening_width_ >= closing_width_);
}

void SchunkWsgStatusInterpreter::CalcGripperStatus(
    const Context<double>& context, GripperStatus* gripper_status) const {

  const VectorX<double>& input_vector = this->EvalVectorInput(
      context, input_port_state_)->CopyToVector();

  if(input_vector[0] > opening_width_) {
    *gripper_status = GripperStatus::Opened;
  } else {
    *gripper_status = GripperStatus::Closed;
  }
}
}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake

