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

const string kPath = "examples/kuka_iiwa_arm/models/objects/";
const string jacoPath = "manipulation/models/jaco_description/urdf/";

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
    std::vector<int>* clutter_instances) {
  map<string, int> target_names = {
      {kPath + "block_for_pick_and_place.urdf", FLAGS_model_reps},
      {kPath + "block_for_pick_and_place_large_size.urdf", FLAGS_model_reps},
      {kPath + "big_robot_toy.urdf", FLAGS_model_reps},
      {kPath + "block_for_pick_and_place_mid_size.urdf", FLAGS_model_reps}, 
      {jacoPath + "j2s7s300.urdf", 0}};

  std::unique_ptr<util::WorldSimTreeBuilder<double>> tree_builder =
      std::make_unique<util::WorldSimTreeBuilder<double>>();
      tree_builder->StoreModel("open_top_box", kPath + "open_top_box.urdf");
  tree_builder->AddFixedModelInstance(
    "open_top_box", Eigen::Vector3d(0, 0, 0.01));
  *clutter_instances =
      PopulateWithFloatingObjectRepetitions(tree_builder.get(), target_names);
  tree_builder->AddGround();
  return tree_builder->Build();
}

int DoMain() {
  time_t tstart, tend;

  lcm::DrakeLcm lcm;
  Isometry3<double> clutter_base = Isometry3<double>::Identity();
  clutter_base.translation() = (VectorX<double>(3) << 0.0, 0.0, 3.0).finished();

  std::vector<int> clutter_instances;
  auto scene_tree = GenerateSceneTree(&clutter_instances);

  std::stringstream clutter_instances_string;
  for(auto& it: clutter_instances) {
    clutter_instances_string<<it<<",";
  }
  
  drake::log()->info("ClutterInstancesString {}", 
    clutter_instances_string.str());

  VectorX<double> q_nominal =
      VectorX<double>::Random(scene_tree->get_num_positions());

  RandomClutterGenerator clutter(std::move(scene_tree), clutter_instances,
                                 Vector3<double>(0.0, 0.0, 0.65),
                                 Vector3<double>(0.25, 0.45, 2.0), true);

  std::default_random_engine generator(123);

  for (int i = 0; i < 15; ++i) {
    tstart = time(0);
    clutter.Generate(q_nominal, generator);
    tend = time(0);

    drake::log()->info("---------------------------------------");
    drake::log()->info("\tIt took {} sec", std::difftime(tend, tstart));
    drake::log()->info("---------------------------------------");
  }

  drake::log()->info("Bounded Clutter generated\n");

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