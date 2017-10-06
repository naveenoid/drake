#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/context.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {
using systems::LeafSystem;
using systems::InputPortDescriptor;
using systems::Context;
/**
 * This system interprets a `manipulation::state_machine::GripperCommand`
 * into an input for the `SchunkWsgStatusController.
 * In the default case, the GripperCommand::Open is interpreted as
 * a command to open to 110mm and a GripperCommand::Close is
 * interpreted as a command to close to 0 mm.
 */
class SchunkWsgCommandInterpreter : public LeafSystem<double> {
 public :
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SchunkWsgCommandInterpreter)

  /**
   * Constructs the SchunkWsgCommandInterpreter.
   * @param opening_width The width corresponding to the
   * GripperCommand::Open.
   * @param closing_width The width corresponding to the
   * GripperCommand::Close.
   */
  SchunkWsgCommandInterpreter(double opening_width = 110,
                              double closing_width = 0);

  const systems::InputPortDescriptor<double>& get_command_input_port() const {
    return this->get_input_port(input_port_gripper_command_);
  }

  const systems::OutputPort<double>& get_command_output_port() const {
    return this->get_output_port(output_port_gripper_plan_);
  }
 protected :

  void CalcGripperPlan(const Context<double>& context,
                       VectorX<double>* gripper_plan) const;

 private:
  double opening_width_{110};
  double closing_width_{0};

  const int input_port_gripper_command_{-1};
  const int output_port_gripper_plan_{-1};
};

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
