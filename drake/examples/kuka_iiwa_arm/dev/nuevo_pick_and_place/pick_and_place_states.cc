#include "drake/examples/kuka_iiwa_arm/dev/nuevo_pick_and_place/pick_and_place_states.h"

#include "drake/examples/kuka_iiwa_arm/dev/nuevo_pick_and_place/pick_and_place_utils.h"
#include "drake/manipulation/state_machine/state_machine_utils.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace nuevo_pick_and_place {
namespace {

Isometry3<double> GetBodyPoseInWorld(
    const RigidBodyTreed* manipulator_tree,
    const std::string& body_name, const VectorX<double> q,
    const VectorX<double> v) {
  auto body = manipulator_tree->FindBody(body_name);
  KinematicsCache<double> cache = manipulator_tree->doKinematics(q, v, true);
  Isometry3<double> body_pose =
      manipulator_tree->CalcBodyPoseInWorldFrame(cache, *body);
  return body_pose;
}

} // namespace

using manipulation::state_machine::GripperStatus;
using manipulation::state_machine::GripperCommand;

void OpenGripper::Execute(
    const ManipulationWorldStatus &manipulation_world_status,
    double, ManipulationWorldCommand* manipulation_world_command) {
  manipulation_world_command->gripper_plan_ = GripperCommand::Open;
  has_completed_ = false;
}

bool OpenGripper::HasCompleted(
    const ManipulationWorldStatus& manipulation_world_status, double) {
  if(manipulation_world_status.gripper_status_ == GripperStatus::Opened) {
    has_completed_ = true;
  }
  return has_completed_;
}


void CloseGripper::Execute(
    const ManipulationWorldStatus& manipulation_world_status, double,
    ManipulationWorldCommand* manipulation_world_command) {

  manipulation_world_command->gripper_plan_ = GripperCommand::Close;
  has_completed_ = false;
}

bool CloseGripper::HasCompleted(
    const ManipulationWorldStatus &manipulation_world_status, double) {
  if(manipulation_world_status.gripper_status_ == GripperStatus::Closed) {
    has_completed_ = true;
  }
  return has_completed_;
}


MoveRelativeToPose::MoveRelativeToPose(
    const RigidBodyTreed &manipulator_tree,
    std::string end_effector_body_name, Isometry3<double> default_target_pose,
    int num_viapoints, double plan_duration,
    const ConstraintRelaxingIkTolerances &tolerances) :
    manipulator_tree_(manipulator_tree.Clone()),
    end_effector_body_name_(end_effector_body_name), tolerances_(tolerances) {

}

void MoveRelativeToPose::Execute(
    const ManipulationWorldStatus &manipulation_world_status,
    double current_time,
    ManipulationWorldCommand *manipulation_world_command) {

  // is has_completed_ is true, this is a fresh instance of Execute
  // being called on this state. So a new plan needs to be generated.
  if(has_completed_) {
    VectorX<double> q = manipulation_world_status.robot_status_.head(
        manipulator_tree_->get_num_positions());
    VectorX<double> v = manipulation_world_status.robot_status_.tail(
        manipulator_tree_->get_num_velocities());

    Isometry3<double> X_Wend_effector_0_ = GetBodyPoseInWorld(
        manipulator_tree_.get(), end_effector_body_name_, q, v);

    Isometry3<double> X_Wend_effector_1_ = target_pose_;

    PiecewisePolynomial default_plan();
    PiecewisePolynomialTrajectory desired_plan(default_plan());

    bool ik_result = PlanStraightLineMotion(
        q, num_viapoints_, plan_duration_,
        X_Wend_effector_0_, X_Wend_effector_1_,
        planner_.get_mutable(), tolerances_, &desired_plan);
    DRAKE_DEMAND(ik_result);

    plan_start_time_ = current_time;
    manipulation_world_command->manipulator_plan_ = desired_plan;

    has_completed_ = false;
  }
}

bool MoveRelativeToPose::HasCompleted(
    const ManipulationWorldStatus &manipulation_world_status, double current_time) {

  VectorX<double> v = manipulation_world_status.robot_status_.tail(
      manipulator_tree_->get_num_velocities());

  if(current_time >= plan_start_time_+ plan_duration_
      && v.norm() < 0.1 /* threshold to judge if stopped */) {
    has_completed_ = true;
  }
  return has_completed_;
}


MoveRelativeToTargetObject::MoveRelativeToTargetObject(
    const RigidBodyTreed &manipulator_tree,
    std::string end_effector_body_name,
    Vector3<double> xyz_offset_position,
    int num_viapoints, double plan_duration, int target_id,
    const ConstraintRelaxingIkTolerances& tolerances) :
    MoveRelativeToPose(manipulator_tree, end_effector_body_name,
    Isometry3<double>::Identity(), num_viapoints, plan_duration, tolerances),
    xyz_offset_position_(xyz_offset_position),
    target_id_(target_id) {

}

void MoveRelativeToTargetObject::Execute(
    const ManipulationWorldStatus &manipulation_world_status,
    double current_time,
    ManipulationWorldCommand *manipulation_world_command) {

  Isometry3<double> X_Wend_effector_1_ = ComputeGraspPose(
      manipulation_world_status.targets_.get_pose(target_id_));

  X_Wend_effector_1_.translation() += xyz_offset_position_;

  target_pose_ = X_Wend_effector_1_;

  MoveRelativeToPose::Execute(manipulation_world_status, current_time,
                              manipulation_world_command);

}

} // namespace nuevo_pick_and_place
} // namespace kuka_iiwa_arm
} // namespace examples
} // namespace drake

