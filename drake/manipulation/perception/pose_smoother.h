#pragma once

#include <memory>
#include <vector>

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/event.h"
#include "drake/manipulation/util/moving_average_filter.h"

namespace drake {
namespace manipulation {
namespace perception {
/**
 * This class accepts the 7 dimensional pose of a rigid body (composed by a 3D
 * Cartesian position and a 4D orientation in quaternions) and returns a
 * smoothed pose by performing 2 processes :
 *  i. Rejecting outliers on the basis of user-defined linear/angular velocity
 *  thresholds on consecutive pose data values.
 *  ii. Moving average smoothing of the resulting data within a specified
 *  window size.
 *  Note on quaternion averaging :
 *  While a "correct" quaternion averaging algorithm requires averaging the
 *  corresponding attitudes, this class implements a simplification based on
 *  the version described in http://wiki.unity3d.com/index.php/Averaging_Quaternions_and_Vectors
 *  and in the introduction in [Landis et al].
 *  References:
 *  Landis and Markley
 *
 */
class PoseSmoother : public systems::LeafSystem<double> {
  public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PoseSmoother)
  /**
   * Constructs the pose smoother.
   * @param max_linear_velocity Upper threshold on linear velocity (m/sec).
   * @param max_angular_velocity Upper threshold on angular velocity (rads/sec).
   * @param filter_window_size Window size for the moving average smoothing.
   * @param optitrack_lcm_status_period The period for the internal update
   * (sec). This must be set to a value greater than 0.
   */
  PoseSmoother(
      double max_linear_velocity, double max_angular_velocity,
      int filter_window_size, double optitrack_lcm_status_period);

  const systems::OutputPort<double>& get_smoothed_pose_output_port() const {
    return this->get_output_port(smoothed_pose_output_port_);
  }

  const systems::OutputPort<double>& get_smoothed_velocity_output_port() const {
    return this->get_output_port(smoothed_velocity_output_port_);
  }

 protected:
  void DoCalcDiscreteVariableUpdates(
      const systems::Context<double>& context,
      const std::vector<const systems::DiscreteUpdateEvent<double>*>& events,
      systems::DiscreteValues<double>* discrete_state) const;

  void OutputSmoothedPose(const systems::Context<double>& context,
  systems::BasicVector<double>* output) const;

  void OutputSmoothedVelocity(const systems::Context<double>& context,
  systems::BasicVector<double>* output) const;

  void OutputSmoothedState(const systems::Context<double>& context,
  systems::BasicVector<double>* output) const;

 private:
  const int smoothed_pose_output_port_{0};
  const int smoothed_velocity_output_port_{0};
  const int smoothed_state_output_port_{0};
  const double kMaxLinearVelocity{0.0};
  const double kMaxAngularVelocity{0.0};
  const double kDiscreteUpdateInSec{0};
  const std::unique_ptr<util::MovingAverageFilter<VectorX<double>>> filter_;
};

} // namespace perception
} // namespace manipulation
} // namespace drake