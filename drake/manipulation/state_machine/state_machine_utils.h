#pragma once

#include "drake/systems/rendering/pose_bundle.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace manipulation {
namespace state_machine {
using systems::rendering::PoseBundle;

enum GripperStatus : bool {
  Closed = true,
  Opened = false
};

enum GripperCommand : bool {
  Close = true,
  Open = false
};

struct ManipulationWorldStatus {
  VectorX<double> robot_status_{};
  GripperStatus gripper_status_{
      GripperStatus::Closed };
  PoseBundle targets_{};
  int state_id_{0};
};

}
}
}