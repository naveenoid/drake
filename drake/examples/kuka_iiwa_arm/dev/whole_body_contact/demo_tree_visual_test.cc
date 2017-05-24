/**
 * @file test demo to visualize a given tree in a random set of configurations.
 */

#include <chrono>
#include <thread>

#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/examples/kuka_iiwa_arm/dev/whole_body_contact/demo_diagram_builder.h"
#include "drake/examples/kuka_iiwa_arm/dev/tools/simple_tree_visualizer.h"
//
//DEFINE_int32(num_configurations, 10,
//"Number of random test configurations to display in the demo");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace whole_body_contact {
namespace {

using tools::SimpleTreeVisualizer;

int DoMain() {
  drake::lcm::DrakeLcm lcm;

  ModelInstanceInfo<double> iiwa_instance;
  ModelInstanceInfo<double> box_instance;
  Eigen::Vector3d box_position(1 + -0.43, -0.65, kTableTopZInWorld + 0.1);
  Eigen::Vector3d box_orientation = Vector3<double>(0, 0, 1);

  auto tree =
      BuildDemoTree<double>(&iiwa_instance, &box_instance, box_position,
                            box_orientation);

  SimpleTreeVisualizer simple_tree_visualizer(*tree.get(), &lcm);


   Eigen::VectorXd robot_config = Eigen::VectorXd::Zero(7);

  robot_config << 0.75 , 1.5, 1.5, 1.75, 0.0, -1.5, 0.5;

   Eigen::VectorXd box_config = Eigen::VectorXd::Zero(7);
   box_config.head(3) = (Eigen::Vector3d() << -0.30, 0.5, 0.15).finished();

  Eigen::AngleAxisd aa(0.0, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond quat(aa);
  quat.normalize();
   box_config.tail(4) = quat.vec();


   std::cout<<"Tree positions : "<<tree->get_num_positions()<<", "<<
               tree->get_num_positions()<<"\n";

    std::cout<<"Box position : "<<box_config<<"\n";
   std::cout<<"Box orientation : "<<quat.vec()<<"\n";
   Eigen::VectorXd state = Eigen::VectorXd::Zero(
        tree->get_num_positions());
    state.head(7) = robot_config;
    state.tail(7) = box_config;
   // state.tail(3) = box_position;
    simple_tree_visualizer.visualize(state);

    std::cout<<"State : "<<state<<"\n";

    // Sleep for a second just so that the new configuration can be seen
    // on the visualizer.
    std::this_thread::sleep_for(std::chrono::milliseconds(4000));


  return 0;
}

}  // namespace
}  // namespace whole_body_contact
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::whole_body_contact::DoMain();
}
