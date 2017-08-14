/// @file
///
/// Implements state tracker contact visualization from optitrack / real-robot
/// LCM
/// sources.

#include <memory>

#include <gflags/gflags.h>
#include "bot_core/robot_state_t.hpp"
#include "optitrack/optitrack_frame_t.hpp"
#include "drake/lcmt_contact_results_for_viz.hpp"

#include "drake/examples/kuka_iiwa_arm/dev/contact_state_estimator/iiwa_and_object_state_aggregator.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/perception/optitrack_pose_extractor.h"
#include "drake/manipulation/perception/pose_smoother.h"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_driven_loop.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

using bot_core::robot_state_t;
using optitrack::optitrack_frame_t;
using std::unique_ptr;

DEFINE_string(urdf, "", "Name of urdf to load");
DEFINE_bool(contacts, false, "turn on the contacts computation");

namespace drake {
using manipulation::perception::PoseSmoother;
using manipulation::perception::OptitrackPoseExtractor;
namespace examples {
namespace kuka_iiwa_arm {
namespace contact_state_estimator {
namespace {
const char* const kIiwaUrdf =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_primitive_collision.urdf";
const char* const kObjectUrdf =
        "drake/examples/kuka_iiwa_arm/models/objects/big_box.urdf";
const int kNumRobotJoints = 7;
const char* const kLcmIiwaStatusChannel = "IIWA_STATUS";
const char* const kLcmOptitrackChannel = "OPTITRACK_FRAMES";

std::unique_ptr<RigidBodyTreed> BuildDemoTree(
    const Eigen::Vector3d& box_position = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d& box_orientation = Eigen::Vector3d::Zero()) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  tree_builder->StoreModel("iiwa", kIiwaUrdf);
  tree_builder->StoreModel("target", kObjectUrdf);

  tree_builder->AddGround();

  tree_builder->AddFixedModelInstance("iiwa", Eigen::Vector3d::Zero());
  tree_builder->AddFloatingModelInstance("target", box_position,
                                         box_orientation);
  return tree_builder->Build();
}

int DoMain() {
  lcm::DrakeLcm lcm;
  drake::log()->info("Starting new demo");
  systems::DiagramBuilder<double> builder;

  auto iiwa_status_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_iiwa_status>(
          kLcmIiwaStatusChannel, &lcm));
  iiwa_status_sub->set_name("status_sub");

  auto iiwa_status_receiver =
      builder.AddSystem<IiwaStatusReceiver>(kNumRobotJoints);
  iiwa_status_receiver->set_name("status_receiver");

  auto optitrack_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<optitrack_frame_t>(
          kLcmOptitrackChannel, &lcm));
  optitrack_sub->set_name("optitrack_sub");

  // from experimentation
  Eigen::Isometry3d X_WO = Eigen::Isometry3d::Identity();
  Eigen::Matrix3d rot_mat;
  rot_mat.col(0) = -Eigen::Vector3d::UnitX();
  rot_mat.col(1) = Eigen::Vector3d::UnitZ();
  rot_mat.col(2) = Eigen::Vector3d::UnitY();

  //auto z_transform = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ());
  //auto x_transform = Eigen::AngleAxisd(0.015 * M_PI, Eigen::Vector3d::UnitX());
//
//    auto z_transform = Eigen::AngleAxisd(0.0*M_PI, Eigen::Vector3d::UnitZ());
//    auto x_transform = Eigen::AngleAxisd(0.0*M_PI, Eigen::Vector3d::UnitX());

//  X_WO.linear() = x_transform * z_transform * rot_mat;
    X_WO.linear() = rot_mat;
  Eigen::Vector3d translator;
  translator = Eigen::Vector3d::Zero();
  // translator<< 0.0, 0.0, 0;
  translator << -0.343, -0.1627, 0.005;
  X_WO.translate(translator);

  drake::log()->info("About to add pose extractor");
  // 0, 1 seem to be robot bases
  // Update to thje new version of the pose extractor
  auto optitrack_pose_extractor = builder.AddSystem<OptitrackPoseExtractor>(
      2, X_WO, 0.005 /* pose extractor period */);
  optitrack_pose_extractor->set_name("optitrack pose extractor");

    auto pose_smoother = builder.AddSystem<PoseSmoother>(
        1.0 /* max_linear_velocity */, 0.5*M_PI /* max_radial_velocity */,
        5 /* window_size */, 0.005 /* lcm status period */);
//
//  auto pose_smoother =
//      builder.AddSystem<PoseSmoother>(0.01 /*lcm status period */);

  /// Create a custom method for the tree builder stuff.
  auto iiwa_object_tree =
      BuildDemoTree(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  drake::log()->info("Demo tree contains numposition {}",
                     iiwa_object_tree->get_num_positions());
  drake::log()->info("Demo tree contains numvelocity {}",
                     iiwa_object_tree->get_num_velocities());

  drake::log()->info("About to add StateAggregator");
  auto iiwa_object_state_aggregator =
      builder.AddSystem<IiwaAndObjectStateAggregator>(
          std::move(iiwa_object_tree), 0.005 /* period_sec */);
  iiwa_object_state_aggregator->set_name("iiwa object state aggregator");

  drake::log()->info("About to add visuaLizer");
  auto drake_visualizer = builder.AddSystem<systems::DrakeVisualizer>(
      iiwa_object_state_aggregator->get_rigid_body_tree(), &lcm);
  drake_visualizer->set_name("drake visualizer");

  drake::log()->info("Conecting iiwa sub to iiwa status receiver");
  builder.Connect(iiwa_status_sub->get_output_port(0),
                  iiwa_status_receiver->get_input_port(0));
  drake::log()->info("Conecting iiwa status receiver to iiwa state aggregator");
  builder.Connect(iiwa_status_receiver->get_measured_position_output_port(),
                  iiwa_object_state_aggregator->get_input_port_iiwa_state());
  drake::log()->info("Conecting optitrack sub to pose extractor");
  builder.Connect(optitrack_sub->get_output_port(0),
                  optitrack_pose_extractor->get_input_port(0));

  builder.Connect(optitrack_pose_extractor->get_measured_pose_output_port(),
                  pose_smoother->get_input_port(0));
  builder.Connect(pose_smoother->get_smoothed_state_output_port(),
                  iiwa_object_state_aggregator->get_input_port_object_state());
  // insert a pose smoother here.

  drake::log()->info("pose_smoother->get_smoothed_state_output_port() {}",
                     pose_smoother->get_smoothed_state_output_port().size());

  drake::log()->info(
      "stateaggregator dim {}",
      iiwa_object_state_aggregator->get_input_port_object_state().size());

  drake::log()->info("Conecting state aggregator to visualizer");

  drake::log()->info("drake visualizer {}",
                     drake_visualizer->get_input_port(0).size());
  builder.Connect(
      iiwa_object_state_aggregator->get_output_port_visualizer_state(),
      drake_visualizer->get_input_port(0));

  if (FLAGS_contacts) {
    // contact state visualization.
    // Add contact viz.
    drake::log()->info("Contact visualization is on");
    auto contact_viz =
        builder.AddSystem<systems::ContactResultsToLcmSystem<double>>(
            iiwa_object_state_aggregator->get_rigid_body_tree());
    contact_viz->set_name("contact_viz");

    auto contact_results_publisher = builder.AddSystem(
        systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
            "CONTACT_RESULTS", &lcm));
    contact_results_publisher->set_name("contact_results_publisher");

    builder.Connect(
        iiwa_object_state_aggregator->get_output_port_contact_state(),
        contact_viz->get_input_port(0));
    builder.Connect(contact_viz->get_output_port(0),
                    contact_results_publisher->get_input_port(0));
  }
  auto diagram = builder.Build();

  drake::log()->info("lcm driven loop started");
  systems::lcm::LcmDrivenLoop loop(
      *diagram, *optitrack_sub, nullptr, &lcm,
      std::make_unique<
          systems::lcm::UtimeMessageToSeconds<optitrack_frame_t>>());

  // Waits for the first message.
  const systems::AbstractValue& first_msg = loop.WaitForMessage();
  double msg_time =
      loop.get_message_to_time_converter().GetTimeInSeconds(first_msg);
  //  const optitrack_frame_t &first_status =
  //  first_msg.GetValue<optitrack_frame_t>();
  //  VectorX<double> q0(kNumRobotJoints);
  //
  //  drake::log()->info("First.status : {}", first_status.num_joints);
  //  drake::log()->info("kNumRobotJoints : {}", kNumRobotJoints);

  //  DRAKE_DEMAND(kNumRobotJoints == first_status.num_joints);
  //  for (int i = 0; i < kNumRobotJoints; i++)
  //    q0[i] = first_status.joint_position_measured[i];
  //
  systems::Context<double>* diagram_context = loop.get_mutable_context();
  systems::Context<double>& status_sub_context =
      diagram->GetMutableSubsystemContext(*optitrack_sub, diagram_context);
  optitrack_sub->SetDefaults(&status_sub_context);

  //   Explicit initialization.
  diagram_context->set_time(msg_time);
  //  auto plan_source_context =
  //      diagram->GetMutableSubsystemContext(diagram_context, plan_source);
  //  plan_source->Initialize(msg_time, q0,
  //                          plan_source_context->get_mutable_state());

  loop.RunToSecondsAssumingInitialized();
  drake::log()->info("lcm driven loop about to be stopped for some reason");
  return 0;
}

}  // namespace
}  // namespace contact_state_estimator
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::contact_state_estimator::DoMain();
}
