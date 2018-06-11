#include "drake/manipulation/scene_generation/random_clutter_generator.h"

#include <cmath>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/manipulation/scene_generation/random_clutter_generator.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace scene_generation {
const Vector3<double> kClutterCenter(0.0, 0.0, 0.0);
const Vector3<double> kClutterSize(0.2, 0.4, 1.6);
const char kPath[] = "examples/kuka_iiwa_arm/models/objects/";
const char kContainer[] = "open_top_box.urdf";
const char kObject1[] = "big_robot_toy.urdf";
const char kObject2[] = "block_for_pick_and_place.urdf";
// const double kMaxSettlingTime = 2.5;
const int kNumRepetitions = 4;

namespace {
void VerifyClutterIk(const VectorX<double>& q, int num_elements_in_clutter) {
  Vector3<double> clutter_min = kClutterCenter - 0.5 * kClutterSize;
  Vector3<double> clutter_max = kClutterCenter + 0.5 * kClutterSize;
  std::cout<<"size :"<<q.size()<<"\n";
  for (int i = 0; i < num_elements_in_clutter; ++i) {
    VectorX<double> pose = q.segment(7 * i, 7);
    for (int j = 0; j < 3; ++j) {
      EXPECT_TRUE(pose[j] >= clutter_min[j] && pose[j] <= clutter_max[j]);
    }
    // Check for valid orientation
    VectorX<double> orientation = pose.tail(4);
    EXPECT_TRUE(std::abs(1 - orientation.norm()) < 1e-5);
  }
}
}  // namespace

class ClutterGeneratorTest : public ::testing::Test {
 public:
  void SetUp() {
    auto tree_builder = std::make_unique<util::WorldSimTreeBuilder<double>>();
    tree_builder->StoreModel("container",
                             std::string(kPath) + std::string(kContainer));
    tree_builder->StoreModel("object1",
                             std::string(kPath) + std::string(kObject1));
    tree_builder->StoreModel("object2",
                             std::string(kPath) + std::string(kObject2));

    tree_builder->AddFixedModelInstance("container",
                                        Eigen::Vector3d(0, 0, 0.01));
    std::vector<int> clutter_instances;

    Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
    for (int i = 0; i < kNumRepetitions; ++i) {
      for (int j = 0; j < 2; ++j) {
        std::stringstream object_name;
        object_name << "object" << j + 1;
        int model_instance = tree_builder->AddFloatingModelInstance(
            object_name.str(), origin.translation(),
            origin.linear().eulerAngles(0, 1, 2));
        clutter_instances.push_back(model_instance);
      }
    }

    auto scene_tree = tree_builder->Build();
    num_positions_ = scene_tree->get_num_positions();
    clutter_generator_ = std::make_unique<RandomClutterGenerator>(
        std::move(scene_tree), clutter_instances, kClutterCenter, kClutterSize);
  }

 protected:
  std::unique_ptr<RandomClutterGenerator> clutter_generator_;
  int num_positions_;
};

// class ParametricClutter : public ::testing::TestWithParam

TEST_F(ClutterGeneratorTest, TestClutterGeneration) {
  std::default_random_engine generator(42);

  // Check construction of a ClutterGenerator.
  EXPECT_NO_THROW(SetUp());

  EXPECT_EQ(num_positions_, 7 * 4 * 2);
  VectorX<double> q_initial = VectorX<double>::Random(num_positions_);
  VectorX<double> q_ik, q_out;

  EXPECT_NO_THROW(
      q_ik = clutter_generator_->GenerateFloatingClutter(q_initial, generator));

  EXPECT_NO_THROW(VerifyClutterIk(q_ik, 8));
  EXPECT_EQ(q_ik.size(), num_positions_);

  VectorX<double> expected_q_ik =
      (VectorX<double>() << 0.00491742, -0.0946778, -0.485943, 0.999251,
       0.0156874, -0.012078, -0.0332393, -0.0353352, 0.12295, 0.551126,
       0.243613, 0.0897512, 0.935425, 0.239952, 0.0866337, 0.114335, 0.430785,
       0.882623, -0.232526, 0.38421, 0.138895, 0.00921746, 0.197321, 0.322502,
       0.99059, -0.0366267, -0.112861, 0.0682066, -0.0805078, -0.146412,
       -0.191468, 0.999777, -0.00682559, -0.0178732, 0.00895582, -0.0118353,
       -0.0130784, -0.287947, 0.410034, 0.423442, 0.779683, -0.211338,
       0.0305059, -0.00409457, -0.784797, 0.939013, 0.0205317, -0.124883,
       0.319747, 0.0871127, -0.008822, 0.124902, 0.951364, 0.13906, 0.200061,
       0.188533).finished();

  EXPECT_TRUE(CompareMatrices(q_ik, expected_q_ik, 1e-5));
/*
  drake::log()->info("1. q_ik : {}", q_ik.transpose());
  EXPECT_NO_THROW(
      q_out = clutter_generator_->DropObjectsToGround(q_ik, kMaxSettlingTime));

  VectorX<double> expected_q_out =
      (VectorX<double>()
       << -0.0172616, 0.118082, - 0.117374, 0.924228, -0.155999, 0.203126, 
       -0.283207, 0.0246995, 0.0416224, -0.599842, 0.939458, -0.0435517, 
       -0.333233, 0.0669141, -0.0225794, -0.0910767, -0.481556, 0.807161, 
       0.0548224, -0.447842, -0.380689, -0.0599304, 0.036369, -0.142254, 0.949539,
        -0.0198993, 0.271831, 0.155201, -0.0126265, -0.0733827, -0.0954601, 
        0.131801, 0.806357, 0.164171, -0.552689, 0.0855031, 0.0767592, -0.257896, 
        0.974651, 0.104849, -0.181912, -0.0772687, 0.0877299, -0.0557903, 
        -0.373838, 0.646831, 0.595514, -0.351101, 0.322026, 0.0336604, 
        -0.0562056, 0.139158, 0.847193, 0.0555979, -0.513381, -0.12495).finished();

  //EXPECT_TRUE(CompareMatrices(q_out, expected_q_out, 1e-5));
  drake::log()->info("1. q_out : {}", q_out.transpose());

  EXPECT_NO_THROW(
      q_ik = clutter_generator_->GenerateFloatingClutter(q_initial, generator));

  EXPECT_NO_THROW(VerifyClutterIk(q_ik, 4));

  EXPECT_EQ(q_ik.size(), num_positions_);

  drake::log()->info("2. q_ik : {}", q_ik.transpose());
*/
}

}  // namespace scene_generation
}  // namespace manipulation
}  // namespace drake
