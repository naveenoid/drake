#include "drake/examples/kuka_iiwa_arm/dev/tools/optitrack_handler.h"

#include "drake/examples/kuka_iiwa_arm/dev/tools/moving_average_filter.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace tools {

namespace {
//TODO(naveenoid) : This method should ideally not hardcode the computations but
// utilise settings from some param file of some sort.
Isometry3<double> SetWorldOptitrackTransform() {
  Isometry3<double> world_X_optitrack;
  Eigen::Matrix3d rot_mat;
  rot_mat.col(0) = -Eigen::Vector3d::UnitX();
  rot_mat.col(1) = Eigen::Vector3d::UnitZ();
  rot_mat.col(2) = Eigen::Vector3d::UnitY();
  world_X_optitrack.linear() = rot_mat;
  return(world_X_optitrack);
}
} // namespace

OptitrackHandler::OptitrackHandler(int object_id) :
    object_index_(object_id), world_X_optitrack(SetWorldOptitrackTransform()),
    filter_(nullptr) {
}

OptitrackHandler::OptitrackHandler(int object_id, int filter_window_size) :
    object_index_(object_id), world_X_optitrack(SetWorldOptitrackTransform()),
    filter_(std::make_unique<MovingAverageFilter<Eigen::Vector3d>>(
    filter_window_size, 1 /* data size */)) {
}

Isometry3<double> OptitrackHandler::TrackedObjectPoseInWorld(
    const optitrack::optitrack_frame_t *optitrack_msg) {

  std::vector<optitrack::optitrack_rigid_body_t> rigid_bodies =
      optitrack_msg->rigid_bodies;

  Eigen::Quaterniond quaternion(
      rigid_bodies[object_index_].quat[3], rigid_bodies[object_index_].quat[0],
      rigid_bodies[object_index_].quat[1], rigid_bodies[object_index_].quat[2]);

  Isometry3<double> optitrack_X_object;
  optitrack_X_object.linear() = quaternion.toRotationMatrix();
  optitrack_X_object.translation() = Eigen::Vector3d(
      rigid_bodies[object_index_].xyz[0],
      rigid_bodies[object_index_].xyz[1], rigid_bodies[object_index_].xyz[2]);

  if(filter_ != nullptr) {
    optitrack_X_object.translation() = filter_->compute(optitrack_X_object.translation());
  }

  optitrack_X_object.makeAffine();
  Isometry3<double> world_X_object = world_X_optitrack * optitrack_X_object;

  return(world_X_object);
}

}  // namespace tools
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
