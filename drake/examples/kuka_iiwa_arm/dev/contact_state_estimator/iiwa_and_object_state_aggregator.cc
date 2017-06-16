#include "drake/examples/kuka_iiwa_arm/dev/contact_state_estimator/iiwa_and_object_state_aggregator.h"

#include "bot_core/robot_state_t.hpp"
#include "optitrack/optitrack_frame_t.hpp"

#include "drake/multibody/rigid_body_plant/contact_results.h"
#include "drake/examples/kuka_iiwa_arm/dev/contact_state_estimator/geometric_contact_state_estimator.h"


namespace drake {
using systems::Context;
using systems::BasicVector;
using systems::ContactResults;
namespace examples {
namespace kuka_iiwa_arm {
namespace contact_state_estimator {

struct IiwaAndObjectStateAggregator::InternalState {
  InternalState(const int num_positions, const int num_velocities) {
    aggregate_state = VectorX<double>::Zero(num_positions + num_velocities);

  }


  ~InternalState() {}

  bot_core::robot_state_t iiwa_state;
  optitrack::optitrack_frame_t optitrack_frame;
  VectorX<double> aggregate_state;

};

IiwaAndObjectStateAggregator::IiwaAndObjectStateAggregator(
    std::unique_ptr<RigidBodyTreed> iiwa_object_tree) : iiwa_object_tree_(
    std::move(iiwa_object_tree)) {
  this->set_name("IiwaAndObjectStateAggregator");
  // Add some drake demands for tree num states.

  // Adds a vector output port for the visualizer state.
  output_port_visualizer_state_ =
  this->DeclareVectorOutputPort(
      systems::BasicVector<double>(iiwa_object_tree_->get_num_positions()),
      &IiwaAndObjectStateAggregator::CalcVisualizerStateOutput).get_index();

  // Declares an abstract valued output port for contact information.
  output_port_contact_state_ =
     this->DeclareAbstractOutputPort(
              ContactResults<double>(),
              &IiwaAndObjectStateAggregator::CalcContactResultsOutput)
          .get_index();

  input_port_iiwa_state_ = this->DeclareAbstractInputPort().get_index();
  input_port_object_state_ = this->DeclareAbstractInputPort().get_index();

}

const RigidBodyTreed& IiwaAndObjectStateAggregator::get_rigid_body_tree() const {
  return *iiwa_object_tree_.get();
}

void IiwaAndObjectStateAggregator::CalcVisualizerStateOutput(
    const Context<double> &context, BasicVector<double> *output) const {

  // Extract robot state
  const bot_core::robot_state_t& iiwa_state =
      this->EvalAbstractInput(context, input_port_iiwa_state_)
          ->GetValue<bot_core::robot_state_t>();
  // Extract Optitrack pose
  const optitrack::optitrack_frame_t& optitrack_frame =
      this->EvalAbstractInput(context, input_port_object_state_)
          ->GetValue<optitrack::optitrack_frame_t>();

   VectorX<double> visualizer_state = VectorX<double>::Zero();

  output->SetFromVector(visualizer_state);
}

void IiwaAndObjectStateAggregator::CalcContactResultsOutput(
    const Context<double> &context, ContactResults<double> *contacts) const {
  DRAKE_ASSERT(contacts != nullptr);

}

} // contact_state_estimator
} // kuka_iiwa_arm
} // examples
} // drake