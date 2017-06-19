# pragma once

#include <memory>

#include "drake/systems/framework/leaf_system.h"
#include "drake/examples/kuka_iiwa_arm/dev/tools/optitrack_pose_translator.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace contact_state_estimator {


class OptitrackPoseExtractor : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OptitrackPoseExtractor)

  explicit OptitrackPoseExtractor(int object_id = 1);

  const systems::OutputPort<double>&
  get_measured_pose_output_port() const {
    return this->get_output_port(measured_pose_output_port_);
  }

 private:
  void OutputMeasuredPose(const systems::Context<double>& context,
                              systems::BasicVector<double>* output) const;

  void DoCalcDiscreteVariableUpdates(
      const systems::Context<double>& context,
      systems::DiscreteValues<double>* discrete_state) const override;

  const int measured_pose_output_port_{};
  const std::unique_ptr<tools::OptitrackPoseTranslator> optitrack_pose_translator_;
};

} // contact_state_estimator
} // kuka_iiwa_arm
} // examples
} // drake