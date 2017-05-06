# pragma once

#include <string>
#include <vector>
#include "optitrack/optitrack_frame_t.hpp"

#include "drake/common/eigen_types.h"
#include "drake/examples/kuka_iiwa_arm/dev/tools/moving_average_filter.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace tools {

// Make this class process an optitrack frame message and return state of
// each of the objects
class OptitrackHandler {
 public:
  // TODO(naveenoid) : later introduce a constructor with a string name.
  OptitrackHandler(int object_id);

  OptitrackHandler(int object_id, int filter_window_size);

  Isometry3<double> TrackedObjectPoseInWorld(
      const optitrack::optitrack_frame_t* optitrack_msg);

  void LocateObjectIndexInFrameMessage(void);

private :
  int object_index_{-1};
  const Isometry3<double> world_X_optitrack;
  std::unique_ptr<MovingAverageFilter<Eigen::Vector3d>> filter_{nullptr};
};



}  // namespace tools
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
