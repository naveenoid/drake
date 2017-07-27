#include "drake/manipulation/perception/pose_smoother.h"

#include <vector>

#include "drake/math/quaternion.h"
#include "drake/systems/framework/context.h"
#include "drake/manipulation/util/moving_average_filter.h"
#include "drake/common/eigen_types.h"

namespace drake {
using systems::Context;
using systems::DiscreteValues;
using systems::BasicVector;
namespace manipulation {
using util::MovingAverageFilter;
namespace perception {
using Eigen::Quaterniond;

namespace {
VectorX<double> ComputeVelocities(
    const Isometry3<double>& pose_1, const Isometry3<double>& pose_2) {

}

Isometry3<double> VectorToIsometry3(const VectorX<double>& pose_vector) {
  Isometry3<double> pose;
  pose.linear() = Quaterniond(pose_vector.segment<4>(3)).matrix();
  pose.translation() = pose_vector.segment<3>(0);
  return pose;
}

VectorX<double> Isometry3ToVector(const Isometry3<double>& pose) {
  VectorX<double> pose_vector = VectorX<double>::Zero(7);
  pose_vector.segment<3>(0) = pose.translation();
  pose_vector.segment<4>(3) = Quaterniond(pose.linear()).coeffs();
  return pose_vector;
}

Quaterniond Isometry3ToQuaternion(const Isometry3<double>& pose) {
  return Quaterniond(pose.linear());
}

void FixQuaternionForCloseness(
    const Eigen::Quaterniond& q1, Eigen::Quaterniond *q2) {

}

} // namespace

PoseSmoother::PoseSmoother(
    double max_linear_velocity, double max_angular_velocity,
    int filter_window_size, double optitrack_lcm_status_period) :
    smoothed_pose_output_port_(
        this->DeclareVectorOutputPort(
            BasicVector<double>(7),
            &PoseSmoother::OutputSmoothedPose
        ).get_index()),
    smoothed_velocity_output_port_(
        this->DeclareVectorOutputPort(
            BasicVector<double>(6),
            &PoseSmoother::OutputSmoothedVelocity
        ).get_index()),
    smoothed_state_output_port_(
        this->DeclareVectorOutputPort(
            BasicVector<double>(13),
            &PoseSmoother::OutputSmoothedState
        ).get_index()),
    kMaxLinearVelocity(max_linear_velocity),
    kMaxAngularVelocity(max_angular_velocity),
    kLCMStatusPeriod(optitrack_lcm_status_period),
    filter_(std::make_unique<MovingAverageFilter<VectorX<double>>>(
        filter_window_size)) {
  this->set_name("Pose Smoother");
  this->DeclareVectorInputPort();
  // Internal state dimensions are organised as :
  // 0-2 : Cartesian position.
  // 3-6 : Orientation in quaternions.
  // 7-9 : Linear velocity.
  // 10-12 : Angular velocity.
  this->DeclareDiscreteState(13);
  this->DeclarePeriodicDiscreteUpdate(kLCMStatusPeriod);
}

void PoseSmoother::DoCalcDiscreteVariableUpdates(
    const systems::Context<double> &context,
    const std::vector<const systems::DiscreteUpdateEvent<double> *> &events,
    systems::DiscreteValues<double>* discrete_state) const {
  VectorX<double> input_pose_vector = this->EvalEigenVectorInput(context, 0);
  BasicVector<double>* state_basic_vector =
      discrete_state->get_mutable_vector(0);
  VectorX<double> state_vector = state_basic_vector->get_mutable_value();

  Isometry3<double> input_pose = VectorToIsometry3(input_pose_vector);
  Isometry3<double> current_pose = VectorToIsometry3(state_vector.head(7));

  Quaterniond input_quaternion =  Isometry3ToQuaternion(input_pose);
  Quaterniond current_quaternion = Isometry3ToQuaternion(current_pose);

  VectorX<double> current_velocity = ComputeVelocities(
      input_pose, current_pose, kLCMStatusPeriod);

  // check if linear and angular velocities are below threshold
  // fix quaternions if not near.
  FixQuaternionForCloseness(current_pose, &input_pose);

  state.head(7) = filter_->Update(input_pose);
}

} // namespace perception
} // namespace manipulation
} // namespace drake