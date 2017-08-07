/// @file
///
/// Implements state tracker from real-robot LCM
/// sources.

#include <memory>

#include <gflags/gflags.h>
#include "bot_core/robot_state_t.hpp"

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
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"

using bot_core::robot_state_t;
using std::unique_ptr;

DEFINE_string(urdf, "", "Name of urdf to load");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace contact_state_estimator {
namespace {

const char *const kIiwaUrdf = "/manipulation/models/iiwa_description/urdf/"
    "dual_iiwa14_polytope_collision.urdf";
const int kNumRobotJoints = 14;
const char *const kLcmIiwaStatusChannel = "IIWA_STATUS";

std::unique_ptr<RigidBodyTreed> BuildDemoTree(
    const Eigen::Vector3d& box_position = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d& box_orientation= Eigen::Vector3d::Zero()) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  tree_builder->StoreModel("iiwa", kIiwaUrdf);
  tree_builder->AddGround();

  tree_builder->AddFixedModelInstance("iiwa", Eigen::Vector3d::Zero());

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

  auto iiwa_status_receiver = builder.AddSystem<IiwaStatusReceiver>(kNumRobotJoints);
  iiwa_status_receiver->set_name("status_receiver");

  /// Create a custom method for the tree builder stuff.
  auto iiwa_tree = BuildDemoTree(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  auto drake_visualizer = builder.AddSystem<systems::DrakeVisualizer>(
      *iiwa_tree.get(),&lcm);
  drake_visualizer->set_name("drake visualizer");

  drake::log()->info("Conecting iiwa sub to iiwa status receiver");
  builder.Connect(iiwa_status_sub->get_output_port(0),
                  iiwa_status_receiver->get_input_port(0));
  drake::log()->info("Conecting iiwa status receiver to iiwa state aggregator");

  drake::log()->info("drake visualizer {}",
                     drake_visualizer->get_input_port(0).size());
  builder.Connect(iiwa_status_receiver->get_measured_position_output_port(),
                  drake_visualizer->get_input_port(0));

  auto diagram = builder.Build();

  drake::log()->info("lcm driven loop started");

  systems::lcm::LcmDrivenLoop loop(
      *diagram, *iiwa_status_sub, nullptr, &lcm,
      std::make_unique<
          systems::lcm::UtimeMessageToSeconds<lcmt_iiwa_status>>());

  // Waits for the first message.
  const systems::AbstractValue &first_msg = loop.WaitForMessage();
  double msg_time =
      loop.get_message_to_time_converter().GetTimeInSeconds(first_msg);
  const lcmt_iiwa_status &first_status = first_msg.GetValue<lcmt_iiwa_status>();
  VectorX<double> q0(kNumRobotJoints);

  drake::log()->info("First.status : {}", first_status.num_joints);
  drake::log()->info("kNumRobotJoints : {}", kNumRobotJoints);

  DRAKE_DEMAND(kNumRobotJoints == first_status.num_joints);
  for (int i = 0; i < kNumRobotJoints; i++)
    q0[i] = first_status.joint_position_measured[i];

  systems::Context<double> *diagram_context = loop.get_mutable_context();
  systems::Context<double> *status_sub_context =
      diagram->GetMutableSubsystemContext(diagram_context, iiwa_status_sub);
  iiwa_status_sub->SetDefaults(status_sub_context);

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
