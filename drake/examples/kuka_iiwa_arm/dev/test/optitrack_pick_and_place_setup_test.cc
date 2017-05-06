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
#include <gflags/gflags.h>

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

DEFINE_uint64(robot_id, 2, "ID of base of the kuka robot.");
DEFINE_uint64(box_id, 1, "ID of the box to pick.");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
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

const std::string kRobotPath =
    "/examples/kuka_iiwa_arm/models/objects/block_for_pick_and_place_large_size.urdf";

const std::string kModelPath =
    "/examples/kuka_iiwa_arm/models/objects/block_for_pick_and_place.urdf";

class TestOptitrackData{
 public:
  TestOptitrackData(drake::lcm::DrakeLcm* lcm, std::vector<unsigned int> tracked_object_id) :
      lcm_(lcm), world_X_optitrack(SetWorldOptitrackTransform()),
      tracked_object_id_(tracked_object_id) {
    drake::log()->info("About to subscribe to shit");
    lcm_subscriptions_.push_back(
        lcm_->get_lcm_instance()->subscribe("OPTITRACK_FRAMES",
                                            &TestOptitrackData::HandleOptitrackFrameMsg, this));
    drake::log()->info("Subscribed to {}", "OPTITRACK_FRAMES");
    lcm_subscriptions_.push_back(
        lcm_->get_lcm_instance()->subscribe("OPTITRACK_DATA_DESCRIPTIONS",
                                            &TestOptitrackData::HandleOptitrackDataDescriptionMsg,
                                            this));

    for (unsigned int i =0; i<tracked_object_id_.size(); ++i) {
      drake::log()->info("Tracked object {}", tracked_object_id_.at(i));
    }
    drake::log()->info("Subscribed to {}","OPTITRACK_DATA_DESCRIPTIONS");
  }

  void HandleOptitrackFrameMsg(
      const ::lcm::ReceiveBuffer* rbuf,
      const std::string& chan,
      const optitrack::optitrack_frame_t* optitrack_msg) {

    Eigen::VectorXd body_state = Eigen::VectorXd::Zero(7 * 2);
    if(process_frame_) {


      std::vector<unsigned int> tracked_object_id;

      tracked_object_id.push_back(FLAGS_box_id);
      tracked_object_id.push_back(FLAGS_robot_id);

      std::vector<optitrack::optitrack_rigid_body_t> rigid_bodies =
          optitrack_msg->rigid_bodies;
//      drake::log()->info("Got rigid bodies, tracked object size {}", tracked_object_id_.size());

//      Eigen::VectorXd world_state = Eigen::VectorXd::Zero(7 * 2);
      for(int i = 0 ; i < 2; ++i) {

        int id = tracked_object_id.at(i);
        Isometry3<double> optitrack_X_object;

        drake::log()->info("So far ok at {}", id);
        Eigen::Quaterniond quaternion(rigid_bodies[id].quat[3], rigid_bodies[id].quat[0],
                                      rigid_bodies[id].quat[1], rigid_bodies[id].quat[2]);

        optitrack_X_object.linear() = quaternion.toRotationMatrix();
        optitrack_X_object.translation() = Eigen::Vector3d(rigid_bodies[id].xyz[0],
                                                           rigid_bodies[id].xyz[1], rigid_bodies[id].xyz[2]);

        optitrack_X_object.makeAffine();
        Isometry3<double> world_X_object = world_X_optitrack * optitrack_X_object;

        body_state.segment<3>(7 * i) = world_X_object.translation();
        body_state.segment<4>(7 * i + 3) = drake::math::rotmat2quat(world_X_object.linear());
      }
      simple_tree_visualizer_->visualize(body_state);
      drake::log()->info("Drawing frame");
    }
  }

  void HandleOptitrackDataDescriptionMsg(
      const ::lcm::ReceiveBuffer* rbuf, const std::string& chan,
      const optitrack::optitrack_data_descriptions_t* optitrack_msg) {
    if(!process_frame_) {
      tree_ = std::make_unique<RigidBodyTree<double>>();
      num_objects_in_frame_ = optitrack_msg->num_rigid_bodies;
      drake::log()->info("Optitrack had : {} bodies",
                         num_objects_in_frame_);
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

      parsers::ModelInstanceIdTable table;

      // Add robot
      table = drake::parsers::urdf::AddModelInstanceFromUrdfFile(
          drake::GetDrakePath() + kRobotPath, drake::multibody::joints::kQuaternion,
          weld_to_frame, tree_.get());
      drake::log()->info("Added a new model to the tree");

      // Add object
      table = drake::parsers::urdf::AddModelInstanceFromUrdfFile(
          drake::GetDrakePath() + kModelPath, drake::multibody::joints::kQuaternion,
          weld_to_frame, tree_.get());
      drake::log()->info("Added a new model to the tree");

      drake::log()->info("Added object has {} positions", tree_->get_num_positions());
      simple_tree_visualizer_ = std::make_unique<tools::SimpleTreeVisualizer>(*tree_.get(), lcm_);
      process_frame_ = true;
    }
  }
 private:
  drake::lcm::DrakeLcm* lcm_{nullptr};
  std::unique_ptr<RigidBodyTreed> tree_{nullptr};
  std::list<::lcm::Subscription*> lcm_subscriptions_;
  const Isometry3<double> world_X_optitrack;
  bool process_frame_{false};
  int num_objects_in_frame_{0};
  std::unique_ptr<tools::SimpleTreeVisualizer> simple_tree_visualizer_{nullptr};
  std::vector<unsigned int> tracked_object_id_;
};

int do_main() {
  drake::lcm::DrakeLcm lcm;

  std::vector<unsigned int> tracked_object_id;

  tracked_object_id.push_back(FLAGS_box_id);
  tracked_object_id.push_back(FLAGS_robot_id);

  TestOptitrackData(&lcm, tracked_object_id);

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


/////////// OLD SHIT //////////////////////

//#include "drake/common/text_logging.h"
//
//#include "optitrack/optitrack_frame_t.hpp"
//#include "optitrack/optitrack_data_descriptions_t.hpp"
//#include "optitrack/optitrack_rigid_body_t.hpp"
////#include <lcm/lcm-cpp.hpp>
//#include <string>
//#include <sstream>
//#include <list>
//#include <chrono>
//#include <thread>
//#include <mutex>
//
//#include "drake/common/drake_path.h"
//#include "drake/lcm/drake_lcm.h"
//#include "drake/common/eigen_types.h"
//#include "drake/multibody/parsers/urdf_parser.h"
//#include "drake/multibody/rigid_body_tree.h"
//#include "drake/examples/kuka_iiwa_arm/dev/tools/simple_tree_visualizer.h"
//#include "drake/util/lcmUtil.h"
//#include "drake/math/quaternion.h"
//#include "drake/math/rotation_matrix.h"
//#include "drake/math/roll_pitch_yaw_not_using_quaternion.h"
//#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h
//#include "drake/util/drakeUtil.h"
//
//namespace drake {
//namespace examples {
//namespace kuka_iiwa_arm {
//namespace {
//
////TODO(naveenoid) : This method should ideally not hardcode the computations but
//// utilise settings from some param file of some sort.
//Isometry3<double> SetWorldOptitrackTransform() {
//  Isometry3<double> world_X_optitrack;
//  Eigen::Matrix3d rot_mat;
//  rot_mat.col(0) = -Eigen::Vector3d::UnitX();
//  rot_mat.col(1) = Eigen::Vector3d::UnitZ();
//  rot_mat.col(2) = Eigen::Vector3d::UnitY();
//  world_X_optitrack.linear() = rot_mat;
//  return(world_X_optitrack);
//}
//
//const std::string kRobotPath =
//    "/manipulation/models/iiwa_description/urdf/iiwa14_primitive_collision.urdf";
//
//const std::string kModelPath =
//    "/examples/kuka_iiwa_arm/models/objects/block_for_pick_and_place.urdf";
//
//class TestOptitrackData{
// public:
//  TestOptitrackData(drake::lcm::DrakeLcm* lcm) :
//      lcm_(lcm) {
//    lcm_subscriptions_.push_back(
//        lcm_->get_lcm_instance()->subscribe(
//            "OPTITRACK_FRAMES",
//            &TestOptitrackData::HandleOptitrackFrameMsg, this));
//    drake::log()->info("Subscribed to {}", "OPTITRACK_FRAMES");
//    lcm_subscriptions_.push_back(
//        lcm_->get_lcm_instance()->subscribe(
//            "OPTITRACK_DATA_DESCRIPTIONS",
//            &TestOptitrackData::HandleOptitrackDataDescriptionMsg, this));
//    drake::log()->info("Subscribed to {}", "OPTITRACK_DATA_DESCRIPTIONS");
//
//    lcm_subscriptions_.push_back(
//        lcm_->get_lcm_instance()->subscribe(
//            "IIWA_STATE_EST",
//            &TestOptitrackData::HandleOptitrackDataDescriptionMsg,
//            this));
//  }
//
//  void HandleOptitrackFrameMsg(
//      const ::lcm::ReceiveBuffer* rbuf,
//      const std::string& chan,
//      const optitrack::optitrack_frame_t* optitrack_msg) {
//
//    Isometry3<double> world_X_optitrack;
//    Eigen::Matrix3d rot_mat;
//    rot_mat.col(0) = -Eigen::Vector3d::UnitX();
//    rot_mat.col(1) = Eigen::Vector3d::UnitZ();
//    rot_mat.col(2) = Eigen::Vector3d::UnitY();
//    world_X_optitrack.linear() = rot_mat;
//
//    Eigen::VectorXd body_state = Eigen::VectorXd::Zero(7 * num_objects_in_frame_);
//    if(process_frame_) {
//      std::vector<optitrack::optitrack_rigid_body_t> rigid_bodies =
//          optitrack_msg->rigid_bodies;
//
//      Eigen::VectorXd world_state = Eigen::VectorXd::Zero(7 * num_objects_in_frame_);
//      for(int i = 0 ; i < num_objects_in_frame_; ++i) {
//
//        Isometry3<double> optitrack_X_object;
//
//        Eigen::Quaterniond quaternion(rigid_bodies[i].quat[3], rigid_bodies[i].quat[0],
//                                      rigid_bodies[i].quat[1], rigid_bodies[i].quat[2]);
//
//        optitrack_X_object.linear() = quaternion.toRotationMatrix();
//        optitrack_X_object.translation() = Eigen::Vector3d(rigid_bodies[i].xyz[0],
//                                                           rigid_bodies[i].xyz[1], rigid_bodies[i].xyz[2]);
//
//        optitrack_X_object.makeAffine();
//        Isometry3<double> world_X_object = world_X_optitrack * optitrack_X_object;
//
//        body_state.segment<3>(7 * i) = world_X_object.translation();
//        body_state.segment<4>(7 * i + 3) = drake::math::rotmat2quat(world_X_object.linear());
//      }
//
//    }
//    drake::log()->info("Drawing frame");
//  }
//
//  void HandleOptitrackDataDescriptionMsg(
//      const ::lcm::ReceiveBuffer* rbuf, const std::string& chan,
//      const optitrack::optitrack_data_descriptions_t* optitrack_msg) {
//    if(!process_frame_) {
//      num_objects_in_frame_ = optitrack_msg->num_rigid_bodies;
//      drake::log()->info("Optitrack had : {} bodies",
//                         num_objects_in_frame_);
//      std::vector<optitrack::optitrack_rigid_body_description_t>
//          rigid_bodies = optitrack_msg->rigid_bodies;
//      for (int i = 0; i < optitrack_msg->num_rigid_bodies; ++i) {
//        drake::log()->info("Body {}", i);
//        drake::log()->info("Body name {}", rigid_bodies[i].name);
//      }
//
//      drake::log()->info("Added object has {} positions", tree_->get_num_positions());
//      process_frame_ = true;
//    }
//  }
//  void HandleIIWAStatusMsg(const lcm::ReceiveBuffer* rbuf,
//                           const std::string& chan,
//                           const bot_core::robot_state_t* iiwa_msg) {
//    Isometry3 iiwa_base_ = DecodePose(iiwa_msg->pose);
//
//    Eigen::VectorXd iiwa_q;
//    for (int i = 0; i < iiwa_msg->num_joints; ++i) {
////      iiwa_v[i] = iiwa_msg->joint_velocity[i];
//      iiwa_q[i] = iiwa_msg->joint_position[i];
//    }
//  }
//
//  void DrawTree(void) {
//
//    mtx_.lock();
//      // build state vector
//    mtx_.unlock();
//
//    simple_tree_visualizer_->visualize(body_state);
//  }
//
//  void InitializeTree(void) {
//    if(process_frame_) {
//      // Draw Robot on table.
//      auto tree_builder = std::make_unique<WorldSimTreeBuilder <double>>();
//
//      // Adds models to the simulation builder. Instances of these models can be
//      // subsequently added to the world.
//      tree_builder->StoreModel("iiwa", kRobotPath);
//      tree_builder->StoreModel("table",
//                               "/examples/kuka_iiwa_arm/models/table/"
//                                   "extra_heavy_duty_table_surface_only_collision.sdf");
//      tree_builder->StoreModel(
//          "box",
//          "/examples/kuka_iiwa_arm/models/objects/block_for_pick_and_place.urdf");
//      tree_builder->StoreModel("wsg",
//                               "/examples/schunk_wsg/models/schunk_wsg_50.sdf");
//
//      // Build a world with two fixed tables.  A box is placed one on
//      // table, and the iiwa arm is fixed to the other.
//      tree_builder->AddFixedModelInstance("table",
//                                          Eigen::Vector3d::Zero() /* xyz */,
//                                          Eigen::Vector3d::Zero() /* rpy */);
//      tree_builder->AddGround();
//
//      // The `z` coordinate of the top of the table in the world frame.
//      // The quantity 0.736 is the `z` coordinate of the frame associated with the
//      // 'surface' collision element in the SDF. This element uses a box of height
//      // 0.057m thus giving the surface height (`z`) in world coordinates as
//      // 0.736 + 0.057 / 2.
//      const double kTableTopZInWorld = 0.736 + 0.057 / 2;
//
//      // Coordinates for kRobotBase originally from iiwa_world_demo.cc.
//      // The intention is to center the robot on the table.
//      const Eigen::Vector3d kRobotBase(-0.243716, -0.625087, kTableTopZInWorld);
//      // Start the box slightly above the table.  If we place it at
//      // the table top exactly, it may start colliding the table (which is
//      // not good, as it will likely shoot off into space).
////      const Eigen::Vector3d kBoxBase(1 + -0.43, -0.65, kTableTopZInWorld + 0.1);
//      int id = tree_builder->AddFixedModelInstance("iiwa", kRobotBase);
//
//
//      parsers::ModelInstanceIdTable table;
//      for (int i = 0; i < num_objects_in_frame_; ++i) {
//        id = tree_builder->AddFloatingModelInstance(
//            "box", Vector3<double>(0, 0, 0), Vector3<double>(0, 0, 1));
//        drake::log()->info("Added a new model to the tree");
//      }
//
//      tree_(std::move(tree_builder->Build()));
//      num_positions_ = tree_->get_num_positions();
//      simple_tree_visualizer_ = std::make_unique<tools::SimpleTreeVisualizer>(
//          *tree_.get(), lcm_);
//
//      tree_ready_ = true;
//    }
//  }
//
// private:
//  drake::lcm::DrakeLcm* lcm_{nullptr};
//  std::unique_ptr<RigidBodyTreed> tree_{nullptr};
//  std::list<::lcm::Subscription*> lcm_subscriptions_;
//  Isometry3<double> world_X_optitrack;
//  bool process_frame_{false};
//  bool tree_ready_{false};
//  int num_objects_in_frame_{0};
//  int num_positions_{0};
//  std::mutex mtx_;
//  std::unique_ptr<tools::SimpleTreeVisualizer> simple_tree_visualizer_{nullptr};
//  std::map<int, Eigen::VectorXd> world_state_by_id_;
//};
//
//int do_main() {
//  drake::lcm::DrakeLcm lcm;
//
//  TestOptitrackData(&lcm);
////
//  while(true) {
//    // Handles all messages.
//    while (lcm.get_lcm_instance()->handleTimeout(10) == 0) {
//    }
////    std::this_thread::sleep_for(std::chrono::milliseconds(100));
////
//  }
//  return 0;
//}
//
//} // namespace
//} // namespace kuka_iiwa_arm
//} // namesapce examples
//} // namespace drake
//
//int main(int argc, char *argv[]) {
//  return drake::examples::kuka_iiwa_arm::do_main();
//}
//
//
//
