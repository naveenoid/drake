#include "drake/examples/kuka_iiwa_arm/dev/contact_state_estimator/iiwa_and_object_state_aggregator.h"

#include <memory>

#include "drake/lcmt_iiwa_status.hpp"

#include "drake/lcmtypes/drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/rigid_body_plant/compliant_contact_model.h"
#include "drake/multibody/rigid_body_plant/contact_results.h"
#include "drake/systems/framework/system_common.h"

namespace drake {
using systems::Context;
using systems::BasicVector;
using systems::ContactResults;
using systems::CompliantContactModel;
namespace examples {
namespace kuka_iiwa_arm {
namespace contact_state_estimator {

IiwaAndObjectStateAggregator::IiwaAndObjectStateAggregator(
    std::unique_ptr<RigidBodyTreed> iiwa_object_tree, const double period_sec)
    : iiwa_object_tree_(std::move(iiwa_object_tree)),
      kNumPositions(iiwa_object_tree->get_num_positions()),
      kNumVelocities(iiwa_object_tree->get_num_velocities()),
      compliant_contact_model_(
          std::make_unique<CompliantContactModel<double>>()) {
  this->set_name("IiwaAndObjectStateAggregator");
  // Add some drake demands for tree num states.

  // Adds a vector output port for the visualizer state.
  output_port_visualizer_state_ =
      this->DeclareVectorOutputPort(
              systems::BasicVector<double>(kNumPositions + kNumVelocities),
              &IiwaAndObjectStateAggregator::CalcVisualizerStateOutput)
          .get_index();

  // Declares an abstract valued output port for contact information.
  output_port_contact_state_ =
      this->DeclareAbstractOutputPort(
              ContactResults<double>(),
              &IiwaAndObjectStateAggregator::CalcContactResultsOutput)
          .get_index();

  input_port_iiwa_state_ =
      this->DeclareInputPort(systems::kVectorValued, 28).get_index();
  input_port_object_state_ =
      this->DeclareInputPort(systems::kVectorValued, 13).get_index();
  this->DeclareDiscreteState(kNumPositions + 7);
  this->DeclarePeriodicDiscreteUpdate(period_sec);
}

void IiwaAndObjectStateAggregator::DoCalcDiscreteVariableUpdates(
    const systems::Context<double>& context,
    const std::vector<const systems::DiscreteUpdateEvent<double>*>& events,
    systems::DiscreteValues<double>* discrete_state) const {
  const systems::BasicVector<double>* iiwa_state =
      this->EvalVectorInput(context, input_port_iiwa_state_);
  const systems::BasicVector<double>* object_state =
      this->EvalVectorInput(context, input_port_object_state_);

  // this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(iiwa_state != nullptr);
  DRAKE_ASSERT(object_state != nullptr);

  BasicVector<double>* state = discrete_state->get_mutable_vector(0);
  auto state_value = state->get_mutable_value();

  state_value = VectorX<double>::Zero(kNumPositions + kNumVelocities);
  VectorX<double> iiwa_state_vector = iiwa_state->CopyToVector();
  state_value.segment<14>(0) = iiwa_state_vector.head(14);
  state_value.segment<7>(14) = object_state->CopyToVector();
}

void IiwaAndObjectStateAggregator::CalcVisualizerStateOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const VectorX<double> state_vector =
      context.get_discrete_state(0)->CopyToVector();
  output->get_mutable_value().head(21) = state_vector;
}

void IiwaAndObjectStateAggregator::CalcContactResultsOutput(
    const Context<double>& context, ContactResults<double>* contacts) const {
  DRAKE_ASSERT(contacts != nullptr);
  // call the geometric_contact_State_estimator
  contacts->Clear();
  // TODO(SeanCurtis-TRI): This is horribly redundant code that only exists
  // because the data is not properly accessible in the cache.  This is
  // boilerplate drawn from EvalDerivatives.  See that code for further
  // comments
  auto x = dynamic_cast<const BasicVector<double>&>(
               context.get_continuous_state_vector())
               .get_value();
  const int nq = kNumPositions;
  const int nv = kNumVelocities;
  VectorX<double> q = x.topRows(nq);
  VectorX<double> v = x.bottomRows(nv);
  auto kinsol = iiwa_object_tree_->doKinematics(q, v);

  compliant_contact_model_->ComputeContactForce(*iiwa_object_tree_.get(),
                                                kinsol, contacts);
}

}  // contact_state_estimator
}  // kuka_iiwa_arm
}  // examples
}  // drake