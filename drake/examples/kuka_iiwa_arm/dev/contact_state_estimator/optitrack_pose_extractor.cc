#include "drake/examples/kuka_iiwa_arm/dev/contact_state_estimator/optitrack_pose_extractor.h"

#include "optitrack/optitrack_frame_t.hpp"

#include "drake/systems/framework/context.h"
#include "drake/examples/kuka_iiwa_arm/dev/tools/optitrack_pose_translator.h"
#include "drake/math/quaternion.h"
#include "drake/math/rotation_matrix.h"
//#include "drake/math/roll_pitch_yaw_not_using_quaternion.h"
#include "drake/util/drakeUtil.h"

namespace drake {
using systems::Context;
using systems::DiscreteValues;
using systems::BasicVector;

namespace examples {
namespace kuka_iiwa_arm {
using tools::OptitrackPoseTranslator;

namespace contact_state_estimator {

// This value is chosen to match the value in getSendPeriodMilliSec()
// when initializing the FRI configuration on the iiwa's control
// cabinet.
const double kOptitrackLcmStatusPeriod = 0.01;

OptitrackPoseExtractor::OptitrackPoseExtractor(int object_id)
    : measured_pose_output_port_(
    this->DeclareVectorOutputPort(BasicVector<double>(13),
            &OptitrackPoseExtractor::OutputMeasuredPose)
        .get_index()),
  optitrack_pose_translator_(std::make_unique<OptitrackPoseTranslator>(
  object_id/* object ID in optitrack frame */)) {

    this->set_name("Optitrack pose extractor");
    this->DeclareAbstractInputPort();
    this->DeclareDiscreteState(13);
    this->DeclareDiscreteUpdatePeriodSec(kOptitrackLcmStatusPeriod);
}

void OptitrackPoseExtractor::DoCalcDiscreteVariableUpdates(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& pose_message = input->GetValue<optitrack::optitrack_frame_t>();
  BasicVector<double>* state = discrete_state->get_mutable_vector(0);
  auto state_value = state->get_mutable_value();
  state_value = VectorX<double>::Zero(13);
  Isometry3<double> object_pose = optitrack_pose_translator_->TranslatePose(
      &pose_message);
  state_value.segment<3>(0) = object_pose.translation();
  state_value.segment<4>(3) = drake::math::rotmat2quat(object_pose.linear());
}

void OptitrackPoseExtractor::OutputMeasuredPose(const Context<double>& context,
                                                BasicVector<double>* output) const {
  const auto state_value = context.get_discrete_state(0)->get_value();
  Eigen::VectorBlock<VectorX<double>> measured_position_output =
      output->get_mutable_value();
  measured_position_output = state_value;
}

} // contact_state_estimator
} // kuka_iiwa_arm
} // examples
} // drake