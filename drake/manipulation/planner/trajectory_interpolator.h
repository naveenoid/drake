#pragma once


#include <memory>
#include <string>
#include <vector>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace manipulation {
namespace planner {

/// This enum specifies the type of interpolator to use in constructing
/// the piece-wise polynomial.
enum class InterpolatorType {
  ZeroOrderHold,
  FirstOrderHold,
  Pchip,
  Cubic
};

/// This class implements a state-less source of joint positions
/// for a robot. It has one input port, for a
/// PiecewisePolynomialTrajectory containing a plan to be followed.
///
/// The system has two output ports, one with the current desired
/// state (q,v) of the robot and another for the accelerations.
///
template <typename T>
class TrajectoryInterpolator : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrajectoryInterpolator)

  TrajectoryInterpolator(
      int output_state_dimension,
      const InterpolatorType = InterpolatorType::Cubic);
  ~TrajectoryInterpolator() override;

  const systems::InputPortDescriptor<T>& get_trajectory_input_port() const {
    return this->get_input_port(trajectory_input_port_);
  }

  const systems::OutputPort<T>&
  get_state_output_port() const {
    return this->get_output_port(state_output_port_);
  }

  const systems::OutputPort<T>&
  get_acceleration_output_port() const {
    return this->get_output_port(acceleration_output_port_);
  }

 protected:
  // Calculator method for the state output port.
  void OutputState(const systems::Context<T>& context,
                   systems::BasicVector<T>* output) const;

  // Calculator method for the accleration output port.
  void OutputAccel(const systems::Context<T>& context,
                   systems::BasicVector<T>* output) const;

  const int trajectory_input_port_{};
  int state_output_port_{-1};
  int acceleration_output_port_{-1};
  const InterpolatorType interp_type_;
  const int output_state_dimension_{-1};
};

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
