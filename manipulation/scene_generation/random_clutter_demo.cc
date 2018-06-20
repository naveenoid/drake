#include <ctime>
#include <map>
#include <random>
#include <sstream>
#include <string>

#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/scene_generation/random_clutter_generator.h"
#include "drake/manipulation/scene_generation/simulate_plant_to_rest.h"
#include "drake/manipulation/util/simple_tree_visualizer.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace scene_generation {
namespace {
using std::string;
using std::stringstream;
using std::map;

DEFINE_int32(repetitions, 1, "each of 4 models is repeated this many times.");
DEFINE_int32(max_iterations, 100, "Max iterations for this demo.");
DEFINE_bool(visualize_only_terminal_state, true,
            "turn on visualization of the "
            "demo only at terminal states : i.e. when ik is available and when "
            "the simulate "
            "to rest terminates.");
DEFINE_double(max_settling_time, 1.5,
              "maximum simulation time for settling the"
              "object.");
DEFINE_double(v_threshold, 0.1, "velocity threshold to terminate sim early.");
DEFINE_double(min_distance, 0.005, "minimum distance between objects");
DEFINE_bool(fall_sim, false,
            "Compute a fall simulation to 'settle' the objects.");
DEFINE_bool(add_z_height_cost, true,
            "Add a cost on the z height of the objects");
DEFINE_bool(use_quaternions, false, "use floating rpy joints (optimise"
  " orientation as well)");
DEFINE_bool(return_infeasible_as_well, true, "return constraint accuracy tolerance "
  "not met solutions as well.");

const char kPath[] = "examples/kuka_iiwa_arm/models/objects/";

std::unique_ptr<RigidBodyTreed> GenerateSceneTree(
    std::vector<int>* clutter_instances, int num_repetitions) {
  map<string, int> target_names = {
      {string(kPath) + "block_for_pick_and_place.urdf", num_repetitions},
      {string(kPath) + "block_for_pick_and_place_large_size.urdf",
       num_repetitions},
      {string(kPath) + "big_robot_toy.urdf", num_repetitions},
      {string(kPath) + "block_for_pick_and_place_mid_size.urdf",
       num_repetitions}};

  std::unique_ptr<util::WorldSimTreeBuilder<double>> tree_builder =
      std::make_unique<util::WorldSimTreeBuilder<double>>();
  tree_builder->StoreModel("open_top_box", string(kPath) + "open_top_box.urdf");
  tree_builder->AddFixedModelInstance("open_top_box",
                                      Eigen::Vector3d(0, 0, 0.01));

  std::vector<int> clutter_instance_list;
  int model_ctr = 0;
  for (map<string, int>::iterator it = target_names.begin();
       it != target_names.end(); ++it) {
    stringstream model_name;
    model_name << "model_" << model_ctr++;
    tree_builder->StoreModel(model_name.str(), it->first);

    for (int i = 0; i < it->second; ++i) {
      // All floating objects are added to origin.
      Isometry3<double> model_pose = Isometry3<double>::Identity();
      int tree_instances = tree_builder->AddFloatingModelInstance(
          model_name.str(), model_pose.translation(),
          model_pose.linear().eulerAngles(0, 1, 2), FLAGS_use_quaternions);
      clutter_instance_list.push_back(tree_instances);
    }
  }

  *clutter_instances = clutter_instance_list;

  tree_builder->AddGround();
  return tree_builder->Build();
}

int DoMain() {
  time_t tstart, tend;

  lcm::DrakeLcm lcm;
  Isometry3<double> clutter_base = Isometry3<double>::Identity();
  clutter_base.translation() = (VectorX<double>(3) << 0.0, 0.0, 3.0).finished();

  std::vector<int> clutter_instances;

  auto scene_tree = GenerateSceneTree(&clutter_instances, FLAGS_repetitions);

  std::stringstream clutter_instances_string;
  for (auto& it : clutter_instances) {
    clutter_instances_string << it << ",";
  }

  VectorX<double> q_nominal =
      VectorX<double>::Random(scene_tree->get_num_positions());
  std::unique_ptr<RandomClutterGenerator> clutter;

  clutter = std::make_unique<RandomClutterGenerator>(
      scene_tree.get(), clutter_instances, Vector3<double>(0.0, 0.0, 0.3),
      Vector3<double>(0.17, 0.34, 0.3 * FLAGS_repetitions), FLAGS_min_distance);
  //Vector3<double>(0.2, 0.4, 0.3 * FLAGS_repetitions));

  auto scene_plant =
      std::make_unique<systems::RigidBodyPlant<double>>(std::move(scene_tree));

  std::unique_ptr<SimpleTreeVisualizer> simple_tree_visualizer;

  std::unique_ptr<SimulatePlantToRest> dropper;

  if (FLAGS_visualize_only_terminal_state) {
    simple_tree_visualizer = std::make_unique<SimpleTreeVisualizer>(
        scene_plant->get_rigid_body_tree(), &lcm);
  }

  if (FLAGS_fall_sim) {
    if (!FLAGS_visualize_only_terminal_state) {
      auto drake_visualizer = std::make_unique<systems::DrakeVisualizer>(
          scene_plant->get_rigid_body_tree(), &lcm);
      dropper = std::make_unique<SimulatePlantToRest>(
          std::move(scene_plant), std::move(drake_visualizer));
    } else {
      dropper = std::make_unique<SimulatePlantToRest>(std::move(scene_plant));
    }
  }

  std::default_random_engine generator(123);

  VectorX<double> q_final, v_final;
  double sum_ik = 0, mean_ik = 0, sum_total = 0, mean_total = 0;
  for (int i = 0; i < FLAGS_max_iterations; ++i) {
    tstart = time(0);
    VectorX<double> q_ik = clutter->GenerateFloatingClutter(
        q_nominal, &generator, FLAGS_add_z_height_cost, FLAGS_return_infeasible_as_well);
    if (FLAGS_visualize_only_terminal_state) {
      simple_tree_visualizer->visualize(q_ik);
    }

    tend = time(0);
    double ik_time = std::difftime(tend, tstart);

    sum_ik += ik_time;
    mean_ik = sum_ik / (i + 1.0);
    drake::log()->debug(
        "Successfully computed clutter. About to simulate dropping");
    if (FLAGS_fall_sim) {
      q_final = dropper->Run(q_ik, &v_final, FLAGS_v_threshold,
                             FLAGS_max_settling_time);
      if (FLAGS_visualize_only_terminal_state) {
        simple_tree_visualizer->visualize(q_final);
      }
    }
    tend = time(0);

    double process_time = std::difftime(tend, tstart);
    sum_total += process_time;
    mean_total = sum_total / (i + 1.0);

    drake::log()->info("IK time : {} sec", ik_time);
    drake::log()->info("Computation time : {} sec", process_time);
    drake::log()->info("Trials : {}, Mean computation time : {}", i,
                       mean_total);
    drake::log()->info("Trials : {}, Mean ik time : {}", i, mean_ik);
  }

  return 0;
}

}  // namespace
}  // namespace scene_generation
}  // namespace manipulation
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  drake::manipulation::scene_generation::DoMain();
}
