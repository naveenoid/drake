# pragma once

#include<memory>

#include "drake/multibody/rigid_body_plant/contact_results.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/examples/kuka_iiwa_arm/dev/contact_state_estimator/geometric_contact_state_estimator.h"

#include "drake/systems/framework/leaf_system.h"
namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace contact_state_estimator {

class IiwaAndObjectStateAggregator : public systems::LeafSystem<double> {
 public:
  IiwaAndObjectStateAggregator(std::unique_ptr<RigidBodyTreed> iiwa_object_tree,
                               const double period_sec = 0.001);

  /**
  * Getter for the input port corresponding to the abstract input with iiwa
  * state message (LCM `robot_state_t` message).
  * @return The corresponding `sytems::InputPortDescriptor`.
  */
  const systems::InputPortDescriptor<double>& get_input_port_iiwa_state()
  const {
    return this->get_input_port(input_port_iiwa_state_);
  }

  /**
   * Getter for the input port corresponding to the abstract input with box
   * state message (LCM `botcore::robot_state_t` message).
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_object_state()
  const {
    return this->get_input_port(input_port_object_state_);
  }


  const systems::OutputPort<double>& get_output_port_visualizer_state()
  const {
    return this->get_output_port(output_port_visualizer_state_);
  }

  const systems::OutputPort<double>& get_output_port_contact_state()
  const {
    return this->get_output_port(output_port_contact_state_);
  }

  const RigidBodyTreed& get_rigid_body_tree() const;


  void DoCalcDiscreteVariableUpdates(
      const systems::Context<double>& context,
      systems::DiscreteValues<double>* discrete_state) const override;

 private:
  void CalcVisualizerStateOutput(const systems::Context<double>& context,
                                 systems::BasicVector<double>* output) const;
  void CalcContactResultsOutput(const systems::Context<double>& context,
                                systems::ContactResults<double>* contacts) const;
  int input_port_iiwa_state_{-1};
  int input_port_object_state_{-1};
  int output_port_visualizer_state_{-1};
  int output_port_contact_state_{-1};

//  RigidBodyTreed& iiwa_object_tree_;
  const int kNumPositions{-1};
  const int kNumVelocities{-1};
  std::unique_ptr<GeometricContactStateEstimator<double>> geometric_contact_state_estimator_{nullptr};
};

} // contact_state_estimator
} // kuka_iiwa_arm
} // examples
} // drake