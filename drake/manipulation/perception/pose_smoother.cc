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
  double dot = q1.dot(*q2);
  if(dot < 0.0) {
    // Invert sign.
    q2->w() = -q2->w();
    q2->x() = -q2->x();
    q2->y() = -q2->y();
    q2->z() = -q2->z();
  }
}

VectorX<double> ComputeVelocities(
    const Isometry3<double>& pose_1, const Isometry3<double>& pose_2,
    double delta_t) {
  VectorX<double> velocities = VectorX<double>::Zero(6);

  Eigen::Array3d translation_diff = pose_1.translation().array() -
      pose_2.translation().array();
  velocities.segment<3>(0) = (translation_diff / delta_t).matrix();

  Eigen::AngleAxisd angle_axis_diff;
  // Computes q[t+1] = q_delta * q[t] first, turn q_delta into an axis, which
  // turns into an angular velocity.
  angle_axis_diff =
      Eigen::AngleAxisd(
          Isometry3ToQuaternion(pose_1) *
              Isometry3ToQuaternion(pose_2).inverse());
  velocities.segment<3>(3) =
      angle_axis_diff.axis() * angle_axis_diff.angle() / delta_t;
  return(velocities);
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
    kMaxLinearVelocity(Eigen::Array3d::Constant(max_linear_velocity)),
    kMaxAngularVelocity(Eigen::Array3d::Constant(max_angular_velocity)),
    kDiscreteUpdateInSec(optitrack_lcm_status_period / 1000.),
    filter_(std::make_unique<MovingAverageFilter<VectorX<double>>>(
        filter_window_size)) {
  this->set_name("Pose Smoother");
  //this->DeclareVectorInputPort(BasicVector<double>::);
  this->DeclareInputPort(systems::kVectorValued, 7);
  // Internal state dimensions are organised as :
  // 0-2 : Cartesian position.
  // 3-6 : Orientation in quaternions.
  // 7-9 : Linear velocity.
  // 10-12 : Angular velocity.
  this->DeclareDiscreteState(13);
  this->DeclarePeriodicDiscreteUpdate(optitrack_lcm_status_period);
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
      input_pose, current_pose, kDiscreteUpdateInSec);

  // Check if linear and angular velocities are below threshold
  if(current_velocity.segment<3>(0).array() < kMaxLinearVelocity &&
     current_velocity.segment<3>(3).array() < kMaxAngularVelocity) {
    // Fix quaternions if not near.
    FixQuaternionForCloseness(current_pose, &input_pose);
    VectorX<double> new_state = Isometry3ToVector(filter_->Update(input_pose));
    state_vector.segment<6>(7) =  ComputeVelocities(
        new_state, state_vector.head(7), kDiscreteUpdateInSec);
    state_vector.head(7) = new_state;
  }
}

void PoseSmoother::OutputSmoothedPose(
    const systems::Context<double> &context,
    systems::BasicVector<double> *output) const {
  const auto state_value = context.get_discrete_state(0)->get_value();
  Eigen::VectorBlock<VectorX<double>> measured_pose_output =
      output->get_mutable_value();
  measured_pose_output = state_value.head(7);
}

void PoseSmoother::OutputSmoothedVelocity(
    const systems::Context<double> &context,
    systems::BasicVector<double> *output) const {
  const auto state_value = context.get_discrete_state(0)->get_value();
  Eigen::VectorBlock<VectorX<double>> measured_pose_output =
      output->get_mutable_value();
  measured_pose_output = state_value.tail(6);
}

void PoseSmoother::OutputSmoothedState(
    const systems::Context<double> &context,
    systems::BasicVector<double> *output) const {
  const auto state_value = context.get_discrete_state(0)->get_value();
  Eigen::VectorBlock<VectorX<double>> measured_pose_output =
      output->get_mutable_value();
  measured_pose_output = state_value;
}
} // namespace perception
} // namespace manipulation
} // namespace drake