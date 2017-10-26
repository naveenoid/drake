#pragma once

#include "drake/manipulation/planner/constraint_relaxing_ik.h"
#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace nuevo_pick_and_place {
using manipulation::planner::ConstraintRelaxingIk;

// Position the gripper 30cm above the object before grasp.
const double kPreGraspHeightOffset = 0.3;

/**
 * Computes a Grasp Pose relative to the pose of a target
 * @param X_WTarget
 * @return The Grasp pose as an Isometry3<double>
 */
Isometry3<double> ComputeGraspPose(const Isometry3<double>& X_WTarget);


/**
 * A struct to store the various tolerances for the ConstraintRelaxingIk.
 */
struct ConstraintRelaxingIkTolerances {
  Vector3<double> tight_pos_tol_{
      Vector3<double>(0.005, 0.005, 0.005) };
  double tight_rot_tol_{0.05};
  Vector3<double> loose_pos_tol_{
      Vector3<double>(0.005, 0.005, 0.005) };
  double loose_rot_tol_{0.5};
};

/**
 * Generates a Cubic polynomial plan in the joint coordinates s.t. the the
 * end effector moves in a straight line between @pX_WEndEffector0 and
 * @p X_WEndEffector1. Orientation is interpolated with slerp. Intermediate
 * waypoints' tolerance can be adjusted separately.
 * @param q_current Current state of the manipulator.
 * @param num_via_points Number of via-points in the plan.
 * @param duration Plan duration in seconds.
 * @param X_WEndEffector0 Initial End-effector Cartesian position of the plan.
 * @param X_WEndEffector1 Final End-effector Cartesian postion of the plan.
 * @param planner A pointer to the ContrainstRelaxingIk structure.
 * @param tolerances A ConstraintRelaxingIkTolerance structure with the
 * tolerance values.
 * @param trajectory The computed PiecewisePolynomialTrajectory
 * @return A bool flag indicating success (true) or failure (false).
 */
bool PlanStraightLineMotion(
    const VectorX<double>& q_current, const int num_via_points,
    double duration, const Isometry3<double>& X_WEndEffector0,
    const Isometry3<double>& X_WEndEffector1, ConstraintRelaxingIk* planner,
    ConstraintRelaxingIkTolerances tolerances,
    PiecewisePolynomialTrajectory* trajectory);


}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

