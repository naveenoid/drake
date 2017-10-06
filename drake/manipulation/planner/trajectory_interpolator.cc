#include "drake/manipulation/planner/trajectory_interpolator.h"

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"

namespace drake {
namespace manipulation {
namespace planner {

template <typename T>
TrajectoryInterpolator<T>::TrajectoryInterpolator(
    int output_state_dimension,
    const InterpolatorType interp_type)
    : output_state_dimension_(output_state_dimension),
      trajectory_input_port_(this->DeclareAbstractInputPort().get_index()),
      interp_type_(interp_type) {
  this->set_name("TrajectoryInterpolator");

  state_output_port_ =
      this->DeclareVectorOutputPort(
              systems::BasicVector<T>(2 * output_state_dimension_),
              &TrajectoryInterpolator::OutputState)
          .get_index();
  acceleration_output_port_ =
      this->DeclareVectorOutputPort(
              systems::BasicVector<T>(output_state_dimension_),
              &TrajectoryInterpolator::OutputAccel)
          .get_index();
}

template <typename T>
TrajectoryInterpolator<T>::~TrajectoryInterpolator() {}

template <typename T>
void TrajectoryInterpolator<T>::OutputState(const systems::Context<T>& context,
                                        systems::BasicVector<T>* output) const {

  const PiecewisePolynomialTrajectory* current_trajectory =
      this->EvalInputValue<PiecewisePolynomialTrajectory>(
          context, trajectory_input_port_);

  double t = context.get_time();

  Eigen::VectorBlock<VectorX<double>> output_vec =
      output->get_mutable_value();
  output_vec.head(output_state_dimension_) = current_trajectory->value(t);
  output_vec.tail(output_state_dimension_) =
      current_trajectory->derivative(1)->value(t);

}

void TrajectoryInterpolator::OutputAccel(
    const systems::Context<double>& context,
    systems::BasicVector<double>* output) const {

  const PiecewisePolynomialTrajectory* current_trajectory =
      this->EvalInputValue<PiecewisePolynomialTrajectory>(
          context, trajectory_input_port_);

  double t = context.get_time();

  Eigen::VectorBlock<VectorX<double>> output_vec =
      output->get_mutable_value();
  output_vec =
      current_trajectory->derivative(2)->value(t);
}

template class TrajectoryInterpolator<double>;

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
