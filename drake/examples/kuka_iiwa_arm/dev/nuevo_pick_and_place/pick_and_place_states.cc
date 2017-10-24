#include "drake/examples/kuka_iiwa_arm/dev/nuevo_pick_and_place/pick_and_place_states.h"

#include "drake/examples/kuka_iiwa_arm/dev/nuevo_pick_and_place/pick_and_place_utils.h"
#include "drake/manipulation/state_machine/state_machine_utils.h"
#include "drake/common/eigen_types.h"

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
    ManipulationWorldCommand* manipulation_world_command) {
  if(manipulation_world_status.gripper_status_ == GripperStatus::Closed) {
    is_active_ = true;
    if(manipulation_world_command->gripper_plan_ != GripperCommand::Open) {
      manipulation_world_command->gripper_plan_ = GripperCommand::Open;
    }
  } else {
    is_active_ = false;
  }
}

MoveRelativeToTargetObject::MoveRelativeToTargetObject(
    const RigidBodyTreed &manipulator_tree,
    std::string end_effector_body_name,
    Vector3<double> xyz_offset_position,
    int num_viapoints, double plan_duration, int target_id) :
    manipulator_tree_(&manipulator_tree),
    end_effector_body_name_(end_effector_body_name),
    xyz_offset_position_(xyz_offset_position),
    num_viapoints_(num_viapoints), plan_duration_(plan_duration),
    target_id_(target_id) {

}

void MoveRelativeToTargetObject::Execute(
    const ManipulationWorldStatus &manipulation_world_status,
    ManipulationWorldCommand *manipulation_world_command) {

  if(!is_active_) {
    VectorX<double> q = manipulation_world_status.robot_status_.head(
        manipulator_tree_->get_num_positions());
    VectorX<double> v = manipulation_world_status.robot_status_.tail(
        manipulator_tree_->get_num_velocities());

    Isometry3<double> X_Wend_effector_0_ = GetBodyPoseInWorld(
        manipulator_tree_, end_effector_body_name_, q, v);

    Isometry3<double> X_Wend_effector_1_ = ComputeGraspPose(
        manipulation_world_status.targets_.get_pose(target_id_));

    bool ik_result = PlanStraightLineMotion(
        q, num_viapoints_, plan_duration_,
        X_Wend_effector_0_, X_Wend_effector_1_,
        loose_pos_tol_, loose_rot_tol_, planner, &ik_res, &times);
    DRAKE_DEMAND(res);

  }



  // if(state != goal state ) make plan,

}



} // namespace nuevo_pick_and_place
} // namespace kuka_iiwa_arm
} // namespace examples
} // namespace drake

