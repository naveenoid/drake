#include <map>
#include <random>
#include <sstream>
#include <string>
#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/scene_generation/random_clutter_generator.h"
#include "drake/manipulation/util/simple_tree_visualizer.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/multibody/rigid_body_tree.h"

#include <ctime>

namespace drake {
namespace manipulation {
namespace scene_generation {
namespace {
using std::string;
using std::stringstream;
using std::map;

DEFINE_int32(model_reps, 1, "each of 4 models is repeated this many times");
DEFINE_int32(max_iterations, 5, "Max iterations for this demo.");
DEFINE_bool(visualize, true, "turn on visualization of the demo");

const string kPath = "examples/kuka_iiwa_arm/models/objects/";

std::vector<int> PopulateWithFloatingObjectRepetitions(
    util::WorldSimTreeBuilder<double>* tree_builder,
    map<string, int> model_map) {
  std::vector<int> clutter_instance_list;
  int model_ctr = 0;
  for (map<string, int>::iterator it = model_map.begin(); it != model_map.end();
       ++it) {
    stringstream model_name;
    model_name << "model_" << model_ctr++;
    tree_builder->StoreModel(model_name.str(), it->first);

    for (int i = 0; i < it->second; ++i) {
      // All floating objects are added to origin.
      Isometry3<double> model_pose = Isometry3<double>::Identity();
      int tree_instances = tree_builder->AddFloatingModelInstance(
          model_name.str(), model_pose.translation(),
          model_pose.linear().eulerAngles(0, 1, 2));
      clutter_instance_list.push_back(tree_instances);
    }
  }
  return clutter_instance_list;
}

std::unique_ptr<RigidBodyTreed> GenerateSceneTree(
    std::vector<int>* clutter_instances, int num_repetitions) {
  map<string, int> target_names = {
      {kPath + "block_for_pick_and_place.urdf", num_repetitions},
      {kPath + "block_for_pick_and_place_large_size.urdf", num_repetitions},
      {kPath + "big_robot_toy.urdf", num_repetitions},
      {kPath + "block_for_pick_and_place_mid_size.urdf", num_repetitions}};

  std::unique_ptr<util::WorldSimTreeBuilder<double>> tree_builder =
      std::make_unique<util::WorldSimTreeBuilder<double>>();
  tree_builder->StoreModel("open_top_box", kPath + "open_top_box.urdf");
  tree_builder->AddFixedModelInstance("open_top_box",
                                      Eigen::Vector3d(0, 0, 0.01));
  *clutter_instances =
      PopulateWithFloatingObjectRepetitions(tree_builder.get(), target_names);
  tree_builder->AddGround();
  return tree_builder->Build();
}

int DoMain() {
  time_t t_overall_start, t_overall_end;
  time_t tstart, tend;
  t_overall_start = time(0);

  lcm::DrakeLcm lcm;
  Isometry3<double> clutter_base = Isometry3<double>::Identity();
  clutter_base.translation() = (VectorX<double>(3) << 0.0, 0.0, 3.0).finished();

  std::vector<int> clutter_instances;
  std::vector<double> mean_computation_time;

  for ( int repetitions = 1; repetitions < 5; ++repetitions) {
    drake::log()->info("Repetitions : {}", repetitions);
    auto scene_tree = GenerateSceneTree(&clutter_instances, repetitions);

    std::stringstream clutter_instances_string;
    for (auto& it : clutter_instances) {
      clutter_instances_string << it << ",";
    }

    VectorX<double> q_nominal =
        VectorX<double>::Random(scene_tree->get_num_positions());
    std::unique_ptr<RandomClutterGenerator> clutter; 

    if(FLAGS_visualize) {
      clutter =  std::make_unique<RandomClutterGenerator>(
        std::move(scene_tree), clutter_instances, Vector3<double>(0.0, 0.0, 0.65),
        Vector3<double>(0.2, 0.4, 0.8*repetitions), BindVisualizeSimulationLcm(&lcm));
    } else {
      clutter =  std::make_unique<RandomClutterGenerator>(
        std::move(scene_tree), clutter_instances, Vector3<double>(0.0, 0.0, 0.65),
        Vector3<double>(0.2, 0.4, 0.8*repetitions));
    }
    
    std::default_random_engine generator(123);

    double sum = 0, mean = 0;
    for (int i = 0; i < FLAGS_max_iterations; ++i) {
      tstart = time(0);
      VectorX<double> q_ik =
          clutter->GenerateFloatingClutter(q_nominal, generator);

      clutter->DropObjectsToGround(q_ik);

      drake::log()->debug("About to simulate dropping");
      tend = time(0);

      double process_time = std::difftime(tend, tstart);
      sum += process_time;
      mean = sum / (i + 1.0);
      drake::log()->info("Computation time : {} sec", process_time);
      drake::log()->info("Mean computation time : {}", mean);
    }
    mean_computation_time.push_back(mean);

    drake::log()->info("Bounded Clutter generated\n");
 
  }

  for(int repetitions = 1; repetitions < 5; ++repetitions) {
    drake::log()->info("Mean computation time for {} repetions : {} sec", repetitions, 
      mean_computation_time[repetitions-1]);
  }

  t_overall_end = time(0);
  
  drake::log()->info("Overall processing time : {} sec", 
    std::difftime(t_overall_end, t_overall_start));
  
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