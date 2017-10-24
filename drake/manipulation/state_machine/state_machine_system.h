#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/manipulation/state_machine/state_machine.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_symbolic_inspector.h"


namespace drake {
namespace manipulation {
namespace state_machine {

/**
 * A system that wraps a ManipulationState an implementation of the
 * Finite-State-Machine logic for the various manipulation demos. This system
 * opens the following ports :
 * - robot_status_port (input): A vector valued port containing the joint
 * state of the robot. The joint state may or may not include joint velocities
 * depending on the particular implementation of the encapsuled
 * ManipulationStateMachine.
 * - gripper_status_port (input) : An abstract valued (GripperStatus) port
 * containing the gripper's status. The actual status needs to be defined
 * externally specific to the implementation of the gripper's control / state.
 * - target_pose_bundle (input) : An abstract valued (PoseBundle) port with
 * a PoseBundle containing the poses of the various target objects. The
 * composition of this PoseBundle depends on the particular implementation
 * of the demo and state estimation systems.
 * - manipulator_plan (output) : An abstract valued
 * (PiecewisePolynomialTrajectory) port containing the manipulation plan output
 * by the state machine.
 * - gripper_plan (output) : An abstract valued (GripperStatus) gripper plan
 * output by the state machine.
 */
class StateMachineSystem : public systems::LeafSystem<double> {
 public:
  /**
   * Constructor for the PickAndPlaceStateMachineSystem
   * @param iiwa_base, The pose of the base of the IIWA robot system.
   * @param period_sec : The update interval of the unrestricted update of
   * this system. This should be bigger than that of the PlanSource components.
   */
  StateMachineSystem(
      std::unique_ptr<StateMachine> manipulation_state_machine,
      const double period_sec = 0.01);

  // This kind of a system is not a direct feedthrough.
  optional<bool> DoHasDirectFeedthrough(int, int) const final {
    return false;
  }

  void DoCalcUnrestrictedUpdate(
      const systems::Context<double>& context,
      const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
      systems::State<double>* state) const;

  /**
   * Getter for the input port corresponding to the abstract input with iiwa
   * state message (LCM `robot_state_t` message).
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_manipulator_status()
      const {
    return this->get_input_port(input_port_manipulator_status_);
  }

  /**
   * Getter for the input port corresponding to the abstract input with box
   * state message (LCM `botcore::robot_state_t` message).
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_target_status() const {
    return this->get_input_port(input_port_target_status_);
  }

  /**
   * Getter for the input port corresponding to the abstract input with the gripper
   * status message (GripperStatus).
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_gripper_status()
      const {
    return this->get_input_port(input_port_gripper_status_);
  }

  const systems::OutputPort<double>& get_output_port_manipulator_plan()
      const {
    return this->get_output_port(output_port_manipulator_plan_);
  }

  const systems::OutputPort<double>& get_output_port_gripper_plan()
      const {
    return this->get_output_port(output_port_gripper_plan_);
  }

 private:
  void CalcManipulatorPlan(
      const systems::Context<double>& context,
      PiecewisePolynomialTrajectory* manipulator_plan) const;

  void CalcGripperPlan(
      const systems::Context<double>& context,
      GripperCommand* gripper_plan) const;

  struct InternalState;

  // Input ports.
  const int input_port_manipulator_status_{-1};
  const int input_port_target_status_{-1};
  const int input_port_gripper_status_{-1};
  // Output ports.
  const int output_port_manipulator_plan_{-1};
  const int output_port_gripper_plan_{-1};


};

}  // namespace state_machine
}  // namespace manipulation
}  // namespace drake
