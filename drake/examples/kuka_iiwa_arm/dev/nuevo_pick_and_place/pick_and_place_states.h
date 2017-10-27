#pragma once

#include <memory>

#include "drake/manipulation/state_machine/manipulation_state.h"
#include "drake/manipulation/state_machine/state_machine_utils.h"
#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/math/rotation_matrix.h"
#include "drake/manipulation/planner/constraint_relaxing_ik.h"
#include "drake/common/eigen_types.h"
#include "drake/common/copyable_unique_ptr.h"
#include "drake/examples/kuka_iiwa_arm/dev/nuevo_pick_and_place/pick_and_place_utils.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace nuevo_pick_and_place {
using manipulation::state_machine::ManipulationState;
using manipulation::state_machine::ManipulationWorldStatus;
using manipulation::state_machine::ManipulationWorldCommand;
using manipulation::planner::ConstraintRelaxingIk;

/**
 * This state controls opening of the gripper.
 */
class OpenGripper : public ManipulationState {
 public:
  std::unique_ptr<ManipulationState> Clone() {
    return std::make_unique<ManipulationState>(
        OpenGripper());
  }

  void Execute(
      const ManipulationWorldStatus& manipulation_world_status,
      double current_time,
      ManipulationWorldCommand* manipulation_world_command
  );

  bool HasCompleted(
      const ManipulationWorldStatus& manipulation_world_status,
      double current_time
  );
};

class CloseGripper : public ManipulationState {
 public:
  std::unique_ptr<ManipulationState> Clone() {
    return std::make_unique<ManipulationState>(
        OpenGripper());
  }
  std::unique_ptr<ManipulationState> Clone() {
    return std::make_unique<ManipulationState>(CloseGripper());
  }

  void Execute(
      const ManipulationWorldStatus& manipulation_world_status,
      double current_time,
      ManipulationWorldCommand* manipulation_world_command
  );

  bool HasCompleted(
      const ManipulationWorldStatus& manipulation_world_status,
      double current_time
  );
};

/**
 * This state generates a plan to move to a desired pose.
 */
class MoveRelativeToPose : public ManipulationState {
 public:
  /**
   * Builds the MoveRelativeToPose ManipulationState.
   * @param xyz_offset_position
   */
  MoveRelativeToPose(
      const RigidBodyTreed& manipulator_tree,
      std::string end_effector_body_name,
      Isometry3<double> default_target_pose,
      int num_viapoints, double plan_duration,
      const ConstraintRelaxingIkTolerances& tolerances);

  virtual std::unique_ptr<ManipulationState> Clone() {
    std::make_unique<ManipulationState>(
        MoveRelativeToPose(
            *manipulator_tree_.get(), end_effector_body_name_,
            target_pose_, num_viapoints_, plan_duration_, tolerances_)); }

  void ResetTargetPose(Isometry3<double> new_target_pose) {
    target_pose_ = new_target_pose;
  }

  virtual void Execute(
      const ManipulationWorldStatus& manipulation_world_status,
      double current_time,
      ManipulationWorldCommand* manipulation_world_command
  );

  bool HasCompleted(
      const ManipulationWorldStatus& manipulation_world_status,
      double current_time
  );

 protected:
  Isometry3<double> target_pose_{};
  copyable_unique_ptr<RigidBodyTreed> manipulator_tree_{};
  std::string end_effector_body_name_{};
  int num_viapoints_{0.0};
  double plan_duration_{0.5};

  ConstraintRelaxingIkTolerances tolerances_;
  copyable_unique_ptr<ConstraintRelaxingIk> planner_{};

  double plan_start_time_{0.0};
};


/**
 * This State generates a plan to Move to a pose that is defined relative
 * to a Target Object (object whose Pose is defined in the target
 * PoseBundle within the ManipulationWorldStatus).
 */
class MoveRelativeToTargetObject : public MoveRelativeToPose {
 public:
 /**
   * Builds the MoveRelativeToTargetPose ManipulationState.
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
      int num_viapoints, double plan_duration, int target_id,
      const ConstraintRelaxingIkTolerances& tolerances);

  std::unique_ptr<ManipulationState> Clone() {
    std::make_unique<ManipulationState>(
        MoveRelativeToTargetObject(
            *manipulator_tree_.get(), end_effector_body_name_,
            xyz_offset_position_, num_viapoints_,
            plan_duration_, target_id_, tolerances_)); }

  void Execute(
      const ManipulationWorldStatus& manipulation_world_status,
      double current_time,
      ManipulationWorldCommand* manipulation_world_command
  );

 protected:
  Vector3<double> xyz_offset_position_{};
  int target_id_{0};
};


class NoOp : public ManipulationState {
 public:

  NoOp() { }

  std::unique_ptr<ManipulationState> Clone() {
    std::make_unique<ManipulationState>(
        NoOp());
  }

  void Execute(const ManipulationWorldStatus& manipulation_world_status,
               double current_time,
               ManipulationWorldCommand* manipulation_world_command) { }

  bool HasCompleted(
      const ManipulationWorldStatus& manipulation_world_status,
      double current_time
  ) { return true; }

};


} // namespace nuevo_pick_and_place
} // namespace kuka_iiwa_arm
} // namespace examples
} // namespace drake
