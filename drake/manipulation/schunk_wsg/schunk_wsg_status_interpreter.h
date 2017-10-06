#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/context.h"
#include "drake/common/eigen_types.h"
#include "drake/manipulation/state_machine/state_machine_utils.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {
using systems::LeafSystem;
using systems::InputPortDescriptor;
using systems::Context;
using state_machine::GripperStatus;
/**
 * This system interprets the state of a SchunkWsg gripper (one of the
 * ModelInstances in a systems::RigidBodyPlant) into a
 * state_machine::GripperStatus. The thresholds for defining
 * of GripperStatus::Open and GripperStatus::Close can be specified at
 * construction time and are in the default case set as 110mm and 0mm
 * respectively.
 */
class SchunkWsgStatusInterpreter : public LeafSystem<double> {
 public :
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SchunkWsgStatusInterpreter)

  /**
   * Constructs the SchunkWsgStatusInterpreter.
   * @param open_width_threshold The width corresponding to the
   * GripperCommand::Open.
   * @param closing_width The width corresponding to the
   * GripperCommand::Close.
   */
  SchunkWsgStatusInterpreter(double open_width_threshold = 110,
                             double closing_width = 0);

  const systems::InputPortDescriptor<double>& get_state_input_port() const {
    return this->get_input_port(input_port_state_);
  }

  const systems::OutputPort<double>& get_status_output_port() const {
    return this->get_output_port(output_port_status_);
  }
 protected :

  void CalcGripperStatus(const Context<double>& context,
                         GripperStatus* gripper_status) const;

 private:
  double opening_width_{110};
  double closing_width_{0};

  const int input_port_state_{-1};
  const int output_port_status_{-1};
};

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
