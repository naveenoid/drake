#include "drake/examples/kuka_iiwa_arm/dev/tools/optitrack_pose_translator.h"

#include "drake/common/drake_assert.h"
#include "math.h"

#include "drake/common/text_logging.h"
#include "drake/examples/kuka_iiwa_arm/dev/tools/moving_average_filter.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace tools {

//TODO(naveenoid) : This method should ideally not hardcode the computations
// but utilise settings from some param file of some sort.
Isometry3<double> DefaultWorldOptitrackTransform() {
  Isometry3<double> world_X_optitrack;
  Eigen::Matrix3d rot_mat;
  rot_mat.col(0) = -Eigen::Vector3d::UnitX();
  rot_mat.col(1) = Eigen::Vector3d::UnitZ();
  rot_mat.col(2) = Eigen::Vector3d::UnitY();

  auto z_tranform = Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitZ());

  auto x_transform = Eigen::AngleAxisd(0.015 * M_PI, Eigen::Vector3d::UnitX());
  world_X_optitrack.linear() = x_transform * z_tranform * rot_mat;


//
//  Eigen::Matrix3d rotate_again;
//  rot_mat.col(0) = -Eigen::Vector3d::UnitY();
//  rot_mat.col(1) = Eigen::Vector3d::UnitX();
//  rot_mat.col(2) = Eigen::Vector3d::UnitZ();
//
 // world_X_optitrack.rotate(rot_mat2);
  Eigen::Vector3d translator;
  translator<< 0.565, -0.055 , 0;
  world_X_optitrack.translate(translator);

  return(world_X_optitrack);
}

OptitrackPoseTranslator::OptitrackPoseTranslator(
    int object_id,
    const Isometry3<double>& world_X_optitrack) :
    object_index_(object_id), world_X_optitrack_(world_X_optitrack),
    filter_(nullptr) {
}

OptitrackPoseTranslator::OptitrackPoseTranslator(
    int object_id, int filter_window_size,
    const Isometry3<double>& world_X_optitrack) :
    object_index_(object_id),
    world_X_optitrack_(world_X_optitrack),
    filter_(std::make_unique<MovingAverageFilter<Eigen::Array3d>>(
    filter_window_size)) {
}

Isometry3<double> OptitrackPoseTranslator::TranslatePose(
    const optitrack::optitrack_frame_t *optitrack_msg) {

  std::vector<optitrack::optitrack_rigid_body_t> rigid_bodies =
      optitrack_msg->rigid_bodies;

//  /drake::log()->info("optitack frame msg had {}", optitrack_msg->rigid_bodies.size());

  DRAKE_DEMAND(object_index_ < rigid_bodies.size());
  // The optitrack quaternion ordering is Z-W-X-Y.
  Eigen::Quaterniond quaternion(
      rigid_bodies[object_index_].quat[3],
      rigid_bodies[object_index_].quat[0],
      rigid_bodies[object_index_].quat[1],
      rigid_bodies[object_index_].quat[2]);

  // Transform pose to world frame.
  Isometry3<double> optitrack_X_object;
  optitrack_X_object.linear() = quaternion.toRotationMatrix();
  optitrack_X_object.translation() = Eigen::Vector3d(
      rigid_bodies[object_index_].xyz[0],
      rigid_bodies[object_index_].xyz[1], rigid_bodies[object_index_].xyz[2]);

  if(filter_ != nullptr) {
    optitrack_X_object.translation() = filter_->Compute(
        optitrack_X_object.translation().array());
  }

  optitrack_X_object.makeAffine();
  Isometry3<double> world_X_object = world_X_optitrack_ * optitrack_X_object;

  return(world_X_object);
}

}  // namespace tools
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
