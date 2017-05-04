#include "drake/common/text_logging.h"

#include "optitrack/optitrack_frame_t.hpp"
#include "optitrack/optitrack_data_descriptions_t.hpp"
#include "optitrack/optitrack_rigid_body_t.hpp"
//#include <lcm/lcm-cpp.hpp>
#include <string>
#include <sstream>
#include <list>
#include <chrono>
#include <thread>
#include <mutex>

#include "drake/common/drake_path.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/examples/kuka_iiwa_arm/dev/tools/simple_tree_visualizer.h"
#include "drake/util/lcmUtil.h"
#include "drake/math/quaternion.h"
#include "drake/math/rotation_matrix.h"
#include "drake/math/roll_pitch_yaw_not_using_quaternion.h"
#include "drake/util/drakeUtil.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const std::string kModelPath =
    "/examples/kuka_iiwa_arm/models/objects/block_for_pick_and_place.urdf";

class TestOptitrackData{
 public:
  TestOptitrackData(drake::lcm::DrakeLcm* lcm, const std::string& frames_channel,
                         const std::string& descriptions_channel) :
      lcm_(lcm) {
    lcm_subscriptions_.push_back(
        lcm_->get_lcm_instance()->subscribe(frames_channel,
                        &TestOptitrackData::HandleOptitrackFrameMsg, this));
    drake::log()->info("Subscribed to {}", frames_channel);
    lcm_subscriptions_.push_back(
        lcm_->get_lcm_instance()->subscribe(descriptions_channel,
                        &TestOptitrackData::HandleOptitrackDataDescriptionMsg,
                        this));

    drake::log()->info("Subscribed to {}", descriptions_channel);
  }

  void HandleOptitrackFrameMsg(
      const ::lcm::ReceiveBuffer* rbuf,
      const std::string& chan,
      const optitrack::optitrack_frame_t* optitrack_msg) {

    Isometry3<double> world_X_optitrack;
    Eigen::Matrix3d rot_mat;
    rot_mat.col(0) = -Eigen::Vector3d::UnitX();
    rot_mat.col(1) = Eigen::Vector3d::UnitZ();
    rot_mat.col(2) = Eigen::Vector3d::UnitY();
    world_X_optitrack.linear() = rot_mat;

    Eigen::VectorXd body_state = Eigen::VectorXd::Zero(7 * num_objects_);
    if(process_frame_) {
    std::vector<optitrack::optitrack_rigid_body_t> rigid_bodies =
        optitrack_msg->rigid_bodies;

      Eigen::VectorXd world_state = Eigen::VectorXd::Zero(7 * num_objects_);
      for(int i = 0 ; i < num_objects_; ++i) {

        Isometry3<double> optitrack_X_object;

        Eigen::Quaterniond quaternion(rigid_bodies[i].quat[3], rigid_bodies[i].quat[0],
            rigid_bodies[i].quat[1], rigid_bodies[i].quat[2]);

        optitrack_X_object.linear() = quaternion.toRotationMatrix();
        optitrack_X_object.translation() = Eigen::Vector3d(rigid_bodies[i].xyz[0],
            rigid_bodies[i].xyz[1], rigid_bodies[i].xyz[2]);

        optitrack_X_object.makeAffine();
        Isometry3<double> world_X_object = world_X_optitrack * optitrack_X_object;

        body_state.segment<3>(7 * i) = world_X_object.translation();
        body_state.segment<4>(7 * i + 3) = drake::math::rotmat2quat(world_X_object.linear());
      }
      simple_tree_visualizer_->visualize(body_state);
    }
    drake::log()->info("Drawing frame");
  }

  void HandleOptitrackDataDescriptionMsg(
      const ::lcm::ReceiveBuffer* rbuf, const std::string& chan,
      const optitrack::optitrack_data_descriptions_t* optitrack_msg) {
    if(!process_frame_) {
      tree_ = std::make_unique<RigidBodyTree<double>>();
      num_objects_ = optitrack_msg->num_rigid_bodies;
      drake::log()->info("Optitrack had : {} bodies",
                         num_objects_);
      std::vector<optitrack::optitrack_rigid_body_description_t>
          rigid_bodies = optitrack_msg->rigid_bodies;
      for (int i = 0; i < optitrack_msg->num_rigid_bodies; ++i) {
        drake::log()->info("Body {}", i);
        drake::log()->info("Body name {}", rigid_bodies[i].name);
      }

      auto weld_to_frame = std::allocate_shared<RigidBodyFrame<double>>(
          Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr,
          Eigen::Vector3d::Zero() /* base position */,
          Eigen::Vector3d::Zero() /* base orientation */);

      for(int i = 0; i < num_objects_; ++i) {
        parsers::ModelInstanceIdTable table;
        table = drake::parsers::urdf::AddModelInstanceFromUrdfFile(
            drake::GetDrakePath() + kModelPath, drake::multibody::joints::kQuaternion,
            weld_to_frame, tree_.get());
        drake::log()->info("Added a new model to the tree");
      }

      drake::log()->info("Added object has {} positions", tree_->get_num_positions());
      simple_tree_visualizer_ = std::make_unique<tools::SimpleTreeVisualizer>(*tree_.get(), lcm_);
      process_frame_ = true;
    }
  }
 private:
  drake::lcm::DrakeLcm* lcm_{nullptr};
  std::unique_ptr<RigidBodyTreed> tree_{nullptr};
  std::list<::lcm::Subscription*> lcm_subscriptions_;
  Isometry3 world_X_optitrack;
  bool process_frame_{false};
  int num_objects_{0};
  std::unique_ptr<tools::SimpleTreeVisualizer> simple_tree_visualizer_{nullptr};
};

int do_main() {
  drake::lcm::DrakeLcm lcm;

  TestOptitrackData(&lcm, "OPTITRACK_FRAMES", "OPTITRACK_DATA_DESCRIPTIONS");
//
  while(true) {
    // Handles all messages.
    while (lcm.get_lcm_instance()->handleTimeout(10) == 0) {
    }
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
//
  }
  return 0;
}

} // namespace
} // namespace kuka_iiwa_arm
} // namesapce examples
} // namespace drake

int main(int argc, char *argv[]) {
  return drake::examples::kuka_iiwa_arm::do_main();
}



