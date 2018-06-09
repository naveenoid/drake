#include "drake/manipulation/scene_generation/random_clutter_generator.h"

#include <map>
#include <random>
#include <sstream>
#include <string>
#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/manipulation/scene_generation/random_clutter_generator.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace scene_generation {

const std::string kPath = "examples/kuka_iiwa_arm/models/objects/";

class ClutterGeneratorTest  : public ::testing::Test{
 protected:
  void SetUp() {
     tree_builder_ =
        std::make_unique<util::WorldSimTreeBuilder<double>>();
    tree_builder_->StoreModel("open_top_box", kPath + "open_top_box.urdf");
    tree_builder_->StoreModel("toy", kPath + "big_robot_toy.urdf");

    tree_builder_->AddFixedModelInstance("open_top_box",
                                        Eigen::Vector3d(0, 0, 0.01));
    
  }

  void PopulateTreeAndSetupClutterGenerator(int num_elements_in_clutter) {
  	std::vector<int> clutter_instances;
	Isometry3<double> origin = Isometry3<double>::Identity();
  	for(int i = 0; i < num_elements_in_clutter; ++i) {

  		int model_instance = tree_builder_->AddFloatingModelInstance(
  			"toy", origin.translation(), origin.linear().eulerAngles(0,1,2));
  		clutter_instances.push_back(model_instance);
  	}

  	auto scene_tree = tree_builder_->Build();

  	num_positions_ = scene_tree->get_num_positions();
  	clutter_generator_ = std::make_unique<RandomClutterGenerator>(
  		std::move(scene_tree), clutter_instances,
        Vector3<double>(0.0, 0.0, 0.65),
        Vector3<double>(0.2, 0.4, 1.6));
  }

	std::unique_ptr<util::WorldSimTreeBuilder<double>> tree_builder_;
	std::unique_ptr<RandomClutterGenerator> clutter_generator_;
	int num_positions_;
};

GTEST_TEST(ClutterGeneratorTest, TestClutterGeneration) {
	std::default_random_engine generator(42);

	// Check construction of a ClutterGenerator.
	// EXPECT_NO_THROW(PopulateTreeAndSetupClutterGenerator(4));

}

}  // namespace scene_generation
}  // namespace manipulation
}  // namespace drake
