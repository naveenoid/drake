#include "drake/manipulation/perception/pose_smoother.h"

#include <vector>

#include "drake/math/quaternion.h"
#include "drake/systems/framework/context.h"
#include "drake/manipulation/util/moving_average_filter.h"
#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"

#define PRINT_VAR(x) drake::log()->info(#x": {}", x);

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
          pose_1.linear() *
              pose_2.linear().inverse());
  velocities.segment<3>(3) =
      angle_axis_diff.axis() * angle_axis_diff.angle() / delta_t;
  return(velocities);
}
} // namespacears

///////////////////////// debug code
void QuaternionLogger(std::string name, Eigen::Quaterniond q) {
  drake::log()->info(name);
  PRINT_VAR(q.w());
  PRINT_VAR(q.x());
  PRINT_VAR(q.y());
  PRINT_VAR(q.z());
}

void VelocityLogger(std::string name, VectorX<double> v) {
  drake::log()->info(name);
  PRINT_VAR(v(0));
  PRINT_VAR(v(2));
  PRINT_VAR(v(3));
  PRINT_VAR(v(4));
  PRINT_VAR(v(5));
  PRINT_VAR(v(6));
}
///////////////////////// debug code
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
    kDiscreteUpdateInSec(optitrack_lcm_status_period),
    filter_(std::make_unique<MovingAverageFilter<VectorX<double>>>(
        filter_window_size)), kSmoothingMode(true) {
  this->set_name("Pose Smoother");
  this->DeclareInputPort(systems::kVectorValued, 7);
  // Internal state dimensions are organised as :
  // 0-2 : Cartesian position.
  // 3-6 : Orientation in quaternions.
  // 7-9 : Linear velocity.
  // 10-12 : Angular velocity.
  // 13 : Time since last accepted sample.
  this->DeclareDiscreteState(14);
  this->DeclarePeriodicDiscreteUpdate(optitrack_lcm_status_period);
}

PoseSmoother::PoseSmoother(double optitrack_lcm_status_period) :
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
        kDiscreteUpdateInSec(optitrack_lcm_status_period),
    kSmoothingMode(false) {
  this->set_name("Pose Smoother");
  this->DeclareInputPort(systems::kVectorValued, 7);
  // Internal state dimensions are organised as :
  // 0-2 : Cartesian position.
  // 3-6 : Orientation in quaternions.
  // 7-9 : Linear velocity.
  // 10-12 : Angular velocity.
  // 13 : Time since last accepted sample.
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

  if(kSmoothingMode) {
      drake::log()->info("about to smooth");
    Isometry3<double> current_pose = VectorToIsometry3(state_vector.head(7));

    Quaterniond input_quaternion = Isometry3ToQuaternion(input_pose);
    Quaterniond current_quaternion = Isometry3ToQuaternion(current_pose);

    // If t = 0
    if (state_vector(13) == 0) {
      state_vector(13) = kDiscreteUpdateInSec;
      current_pose = input_pose;
    }

    VectorX<double> current_velocity = ComputeVelocities(
        input_pose, current_pose, state_vector(13));

    // Check if linear and angular velocities are below threshold
    bool accept_data_point = true;
    for (int i = 0; i < 3; ++i) {
      if (current_velocity(i) >= kMaxLinearVelocity ||
          current_velocity(3 + i) >= kMaxAngularVelocity) {
        accept_data_point = false;
        break;
      }
    }

    // If data is below threshold it can be added to the filter.
    if (accept_data_point) {
      FixQuaternionForCloseness(current_quaternion, &input_quaternion);
      input_pose.linear() = input_quaternion.toRotationMatrix();
      VectorX<double>
          new_state = filter_->Update(Isometry3ToVector(
          input_pose));
      state_vector.segment<6>(7) = ComputeVelocities(
          VectorToIsometry3(new_state), VectorToIsometry3(state_vector.head(7)),
          state_vector(13));
      state_vector.head(7) = new_state;
      state_vector(13) = kDiscreteUpdateInSec;
    } else {

      // Since the current sample has been rejected, the time since the last
      // sample must be incremented suitably.
      state_vector(13) += kDiscreteUpdateInSec;
    }

    state_basic_vector->get_mutable_value() = state_vector;
  } else {
    VectorX<double> padded_state = VectorX<double>::Zero(13);
    padded_state.head(7) = input_pose_vector;
    state_basic_vector->get_mutable_value() = padded_state;

    drake::log()->info("Unsmoother object state x {}, {}, {}",
                       input_pose_vector(0), input_pose_vector(1),
                       input_pose_vector(2));
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
  measured_pose_output = state_value.head(13);

}
} // namespace perception
} // namespace manipulation
} // namespace drake