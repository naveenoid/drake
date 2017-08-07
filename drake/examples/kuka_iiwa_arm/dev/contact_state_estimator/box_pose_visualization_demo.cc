
/// @file
///
/// Implements state tracker from optitrack
/// sources.

#include <memory>

#include <gflags/gflags.h>
#include "bot_core/robot_state_t.hpp"
#include "optitrack/optitrack_frame_t.hpp"

#include "drake/lcm/drake_lcm.h"
#include "drake/common/drake_path.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_driven_loop.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h"
#include "drake/examples/kuka_iiwa_arm/dev/contact_state_estimator/optitrack_pose_extractor.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"

using optitrack::optitrack_frame_t;
using std::unique_ptr;

DEFINE_string(urdf, "", "Name of urdf to load");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace contact_state_estimator {
namespace {

const char *const kObjectUrdf = "/examples/kuka_iiwa_arm/models/objects/big_box.urdf";
const char *const kLcmOptitrackChannel = "OPTITRACK_FRAMES";

std::unique_ptr<RigidBodyTreed> BuildDemoTree(
    const Eigen::Vector3d& box_position = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d& box_orientation= Eigen::Vector3d::Zero()) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  tree_builder->StoreModel(
      "target", kObjectUrdf);

  tree_builder->AddGround();
  tree_builder->AddFloatingModelInstance("target", box_position,
                                         box_orientation);
  return tree_builder->Build();
}

int DoMain() {
  lcm::DrakeLcm lcm;

  drake::log()->info("Starting new demo");

  systems::DiagramBuilder<double> builder;

  auto optitrack_sub =
      builder.AddSystem(
          systems::lcm::LcmSubscriberSystem::Make<optitrack_frame_t>(
              kLcmOptitrackChannel, &lcm));
  optitrack_sub->set_name("optitrack_sub");

  auto optitrack_pose_extractor = builder.AddSystem<OptitrackPoseExtractor>();
  optitrack_pose_extractor->set_name("optitrack pose extractor");

  /// Create a custom method for the tree builder stuff.
  auto iiwa_object_tree = BuildDemoTree(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  auto drake_visualizer = builder.AddSystem<systems::DrakeVisualizer>(
      *iiwa_object_tree.get(),&lcm);
  drake_visualizer->set_name("drake visualizer");

  drake::log()->info("Conecting optitrack sub to pose extractor");
  builder.Connect(optitrack_sub->get_output_port(0),
                  optitrack_pose_extractor->get_input_port(0));
  drake::log()->info("Conecting state aggregator to visualizer");
  builder.Connect(optitrack_pose_extractor->get_measured_pose_output_port(),
                  drake_visualizer->get_input_port(0));

  auto diagram = builder.Build();

  drake::log()->info("lcm driven loop started");

  systems::lcm::LcmDrivenLoop loop(
      *diagram, *optitrack_sub, nullptr, &lcm,
      std::make_unique<
          systems::lcm::UtimeMessageToSeconds<optitrack_frame_t>>());

  // Waits for the first message.
  const systems::AbstractValue &first_msg = loop.WaitForMessage();
  double msg_time =
      loop.get_message_to_time_converter().GetTimeInSeconds(first_msg);
//  const optitrack_frame_t &first_status = first_msg.GetValue<optitrack_frame_t>();
//  VectorX<double> q0(kNumRobotJoints);
//
//  drake::log()->info("First.status : {}", first_status.num_joints);
//  drake::log()->info("kNumRobotJoints : {}", kNumRobotJoints);

//  DRAKE_DEMAND(kNumRobotJoints == first_status.num_joints);
//  for (int i = 0; i < kNumRobotJoints; i++)
//    q0[i] = first_status.joint_position_measured[i];
//
  systems::Context<double> *diagram_context = loop.get_mutable_context();
  systems::Context<double> *status_sub_context =
      diagram->GetMutableSubsystemContext(diagram_context, optitrack_sub);
  optitrack_sub->SetDefaults(status_sub_context);

//   Explicit initialization.
  diagram_context->set_time(msg_time);
//  auto plan_source_context =
//      diagram->GetMutableSubsystemContext(diagram_context, plan_source);
//  plan_source->Initialize(msg_time, q0,
//                          plan_source_context->get_mutable_state());

  loop.RunToSecondsAssumingInitialized();
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
