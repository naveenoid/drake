/**
 * @file test demo to visualize a floating object tree in a random set of configurations.
 */
#include "drake/examples/kuka_iiwa_arm/dev/tools/simple_tree_visualizer.h"

#include <chrono>
#include <thread>

#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

DEFINE_int32(num_configurations, 10,
"Number of random test configurations to display in the demo");

DEFINE_int32(num_objects, 3,
"Number of Objects to display in the demo");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace tools {
namespace {

int DoMain() {
  drake::lcm::DrakeLcm lcm;

  // Adds a demo tree.
  const std::string kModelPath =
      "/examples/kuka_iiwa_arm/models/objects/block_for_pick_and_place.urdf";

  auto tree = std::make_unique<RigidBodyTree<double>>();

  auto weld_to_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr,
      Eigen::Vector3d::Zero() /* base position */,
      Eigen::Vector3d::Zero() /* base orientation */);

  for(int i = 0; i < FLAGS_num_objects; ++i) {
    parsers::ModelInstanceIdTable table;
    table = drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        drake::GetDrakePath() + kModelPath, drake::multibody::joints::kQuaternion,
        weld_to_frame, tree.get());
    drake::log()->info("Added a new model to the tree");
  }

  drake::log()->info("Added object has {} positions", tree->get_num_positions());
  SimpleTreeVisualizer simple_tree_visualizer(*tree.get(), &lcm);

  drake::log()->info("Starting position perturbation tests");

  // Simple demo that iterates through a bunch of position configurations.
  for (int i = 0; i < FLAGS_num_configurations; ++i) {
    Eigen::VectorXd body_state = Eigen::VectorXd::Zero(7 * FLAGS_num_objects);

    for(int j = 0; j < FLAGS_num_objects; ++j) {
      body_state.segment<3>(3 * j) = Eigen::Vector3d::Random();
      simple_tree_visualizer.visualize(body_state);

      // Sleep for a second just so that the new configuration can be seen
      // on the visualizer.
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }

  drake::log()->info("Starting orientation perturbation tests");

  // Simple demo that iterates through a bunch of orientation configurations.
  for (int i = 0; i < FLAGS_num_configurations; ++i) {
    Eigen::VectorXd body_state = Eigen::VectorXd::Zero(7 * FLAGS_num_objects);
    for(int j = 0; j < FLAGS_num_objects; ++j) {
      body_state.segment<4>(3 * FLAGS_num_objects + 4 * j) = Eigen::Vector4d::Random();
      simple_tree_visualizer.visualize(body_state);

      // Sleep for a second just so that the new configuration can be seen
      // on the visualizer.
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }
  return 0;
}

}  // namespace
}  // namespace tools
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::tools::DoMain();
}
