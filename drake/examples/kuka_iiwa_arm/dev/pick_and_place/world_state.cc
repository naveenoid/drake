#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/world_state.h"

#include "bot_core/robot_state_t.hpp"
#include <lcm/lcm-cpp.hpp>
#include "optitrack/optitrack_frame_t.hpp"

#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/util/lcmUtil.h"
#include "drake/examples/kuka_iiwa_arm/dev/tools/optitrack_handler.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

WorldState::WorldState(const std::string& iiwa_model_path,
                       const std::string& end_effector_name, lcm::LCM* lcm)
    : iiwa_model_path_(iiwa_model_path),
      ee_name_(end_effector_name),
      track_object_from_optitrack_(false), lcm_(lcm) {
  ConstructorCommon();
}

WorldState::WorldState(const std::string &iiwa_model_path,
                       const std::string &end_effector_name,
                       lcm::LCM *lcm,
                       int optitrack_object_index,
                       int optitrack_filter_window_size) :
    iiwa_model_path_(iiwa_model_path),
    ee_name_(end_effector_name),
    track_object_from_optitrack_(true), lcm_(lcm) {
  if(optitrack_filter_window_size == 1) {
    optitrack_handler_ = std::make_unique<tools::OptitrackHandler>(
        optitrack_object_index);
  }
  ConstructorCommon();
}



void WorldState::ConstructorCommon() {
  iiwa_time_ = -1;
  iiwa_base_ = iiwa_ee_pose_ = Isometry3<double>::Identity();
  iiwa_q_ = VectorX<double>::Zero(7);
  iiwa_v_ = VectorX<double>::Zero(7);
  iiwa_ee_vel_.setZero();

  wsg_time_ = -1;
  wsg_q_ = 0;
  wsg_v_ = 0;
  wsg_force_ = 0;

  obj_time_ = -1;
  obj_pose_ = Isometry3<double>::Identity();
  obj_vel_.setZero();

}

WorldState::~WorldState() {
  for (lcm::Subscription* sub : lcm_subscriptions_) {
    int status = lcm_->unsubscribe(sub);
    DRAKE_DEMAND(status == 0);
  }
  lcm_subscriptions_.clear();
}

void WorldState::SubscribeToIiwaStatus(const std::string& channel) {
  lcm_subscriptions_.push_back(
      lcm_->subscribe(channel, &WorldState::HandleIiwaStatus, this));
}

void WorldState::SubscribeToWsgStatus(const std::string& channel) {
  lcm_subscriptions_.push_back(
      lcm_->subscribe(channel, &WorldState::HandleWsgStatus, this));
}

void WorldState::SubscribeToObjectStatus(const std::string& channel) {
  if(!track_object_from_optitrack_) {
    lcm_subscriptions_.push_back(
        lcm_->subscribe(channel, &WorldState::HandleObjectStatus, this));
  } else {
    lcm_subscriptions_.push_back(
        lcm_->subscribe(channel, &WorldState::HandleOptitrackObjectStatus, this));
  }
}

void WorldState::HandleIiwaStatus(const lcm::ReceiveBuffer* rbuf,
                                  const std::string& chan,
                                  const bot_core::robot_state_t* iiwa_msg) {
  iiwa_base_ = DecodePose(iiwa_msg->pose);

  if (iiwa_time_ == -1) {
    auto base_frame = std::allocate_shared<RigidBodyFrame<double>>(
        Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr,
        iiwa_base_);

    iiwa_ = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFile(
        iiwa_model_path_, multibody::joints::kFixed, base_frame, iiwa_.get());
    end_effector_ = iiwa_->FindBody("iiwa_link_ee");
  }

  iiwa_time_ = iiwa_msg->utime / 1e6;

  for (int i = 0; i < iiwa_msg->num_joints; ++i) {
    iiwa_v_[i] = iiwa_msg->joint_velocity[i];
    iiwa_q_[i] = iiwa_msg->joint_position[i];
  }

  KinematicsCache<double> cache = iiwa_->doKinematics(iiwa_q_, iiwa_v_, true);

  iiwa_ee_pose_ = iiwa_->CalcBodyPoseInWorldFrame(cache, *end_effector_);
  iiwa_ee_vel_ =
      iiwa_->CalcBodySpatialVelocityInWorldFrame(cache, *end_effector_);
}

void WorldState::HandleWsgStatus(const lcm::ReceiveBuffer* rbuf,
                                 const std::string& chan,
                                 const lcmt_schunk_wsg_status* wsg_msg) {
  bool is_first_msg = wsg_time_ == -1;
  double cur_time = wsg_msg->utime / 1e6;
  double dt = cur_time - wsg_time_;

  wsg_time_ = cur_time;

  if (is_first_msg) {
    wsg_q_ = wsg_msg->actual_position_mm / 1000.;
    wsg_v_ = 0;
    wsg_force_ = wsg_msg->actual_force;
    return;
  }

  if (!is_first_msg && dt == 0) return;

  // TODO(siyuanfeng): Need to filter
  wsg_v_ = (wsg_msg->actual_position_mm / 1000. - wsg_q_) / dt;
  wsg_q_ = wsg_msg->actual_position_mm / 1000.;
  wsg_force_ = wsg_msg->actual_force;
}

void WorldState::HandleObjectStatus(const lcm::ReceiveBuffer* rbuf,
                                    const std::string& chan,
                                    const bot_core::robot_state_t* obj_msg) {
  obj_time_ = obj_msg->utime / 1e6;
  obj_pose_ = DecodePose(obj_msg->pose);
  obj_vel_ = DecodeTwist(obj_msg->twist);
}

void WorldState::HandleOptitrackObjectStatus(
    const lcm::ReceiveBuffer *rbuf, const std::string &chan,
    const optitrack::optitrack_frame_t *optitrack_msg) {
  obj_time_ = optitrack_msg->utime / 1e6;

  obj_pose_ =
      optitrack_handler_->TrackedObjectPoseInWorld(optitrack_msg);

  // TODO(naveenoid) : No way of computing velocities for now.
  obj_vel_ = Eigen::VectorXd::Zero(6);

}

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
