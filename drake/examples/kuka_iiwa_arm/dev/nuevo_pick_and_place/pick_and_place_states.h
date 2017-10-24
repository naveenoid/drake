#pragma once

#include <memory>

#include "drake/manipulation/state_machine/manipulation_state.h"
#include "drake/manipulation/state_machine/state_machine_utils.h"
#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/math/rotation_matrix.h"
#include "drake/manipulation/planner/constraint_relaxing_ik.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace nuevo_pick_and_place {
using manipulation::state_machine::ManipulationState;
using manipulation::state_machine::ManipulationWorldStatus;
using manipulation::state_machine::ManipulationWorldCommand;
using manipulation::planner::ConstraintRelaxingIk;

class OpenGripper : public ManipulationState {
  std::unique_ptr<ManipulationState> Clone() {
    return std::make_unique<ManipulationState>(OpenGripper());
  }

  void Execute(
      const ManipulationWorldStatus& manipulation_world_status,
      ManipulationWorldCommand* manipulation_world_command
  );

};

class MoveRelativeToTargetObject : public ManipulationState {
  /**
   * Builds the MoveRelativeToTargetPose ManipulationState. This State
   * generates a plan to Move to a pose that is defined relative to a Target
   * Object (object whose Pose is defined in the target PoseBundle within the
   * ManipulationWorldStatus).
   *
   * @param manipulator_tree : Tree used for computing the TargetObjectPose plan.
   * @param xyz_offset_position : A 3d position offset relative to the target pose
   * specified in world frame.
   * @param num_viapoints : The number of via-points to be inserted in the planned
   * trajectory.
   * @param plan_duration : The plan duration in seconds.
   * @param target_id : The integer ID corresponding to the target object in the
   * PoseBundle.
   */
  MoveRelativeToTargetObject(
      const RigidBodyTreed& manipulator_tree,
      std::string end_effector_body_name,
      Vector3<double> xyz_offset_position,
      int num_viapoints, double plan_duration, int target_id);

  std::unique_ptr<ManipulationState> Clone() {
    std::make_unique<ManipulationState>(
        MoveRelativeToTargetObject(
            *manipulator_tree_, end_effector_body_name_,
            xyz_offset_position_, num_viapoints_,
            plan_duration_, target_id_)); }

  void Execute(
      const ManipulationWorldStatus& manipulation_world_status,
      ManipulationWorldCommand* manipulation_world_command
  );

 protected:
  Vector3<double> xyz_offset_position_{};
  const RigidBodyTreed* manipulator_tree_{};
  std::string end_effector_body_name_{};
  int num_viapoints_{0};
  double plan_duration_{0.5};
  int target_id_{0};

  Vector3<double> tight_pos_tol_;
  double tight_rot_tol_;

  Vector3<double> loose_pos_tol_;
  double loose_rot_tol_;

  std::unique_ptr<ConstraintRelaxingIk> planner_{};
};

class MoveRelativeToPose : public ManipulationState {
  /**
   * Builds the MoveRelativeToTargetPose.
   * @param xyz_offset_position
   */
  MoveRelativeToPose(
      const RigidBodyTreed& manipulator_tree,
      std::string end_effector_body_name,
      Isometry3<double> default_target_pose,
      double num_viapoints, double plan_duration);

  std::unique_ptr<ManipulationState> Clone() {
    std::make_unique<ManipulationState>(
        MoveRelativeToPose(
            *manipulator_tree_, target_pose_,
            num_viapoints_, plan_duration_)); }

  void ResetTargetPose(Isometry3<double> new_target_pose) {
    target_pose_ = new_target_pose;
  }

  void Execute(
      const ManipulationWorldStatus& manipulation_world_status,
      ManipulationWorldCommand* manipulation_world_command
  );

 protected:
  Isometry3<double> target_pose_{};
  const RigidBodyTreed* manipulator_tree_{};
  std::string end_effector_body_name_{};
  double num_viapoints_{0.0};
  double plan_duration_{0.5};

  Vector3<double> tight_pos_tol_;
  double tight_rot_tol_;

  Vector3<double> loose_pos_tol_;
  double loose_rot_tol_;
};


} // namespace nuevo_pick_and_place
} // namespace kuka_iiwa_arm
} // namespace examples
} // namespace drake
