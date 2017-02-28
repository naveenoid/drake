#include "gtest/gtest.h"

#include <vector>

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/dev/iiwa_ik_planner.h"
#include "drake/multibody/parsers/urdf_parser.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

std::vector<double> linspace(const double lower_limit, const double upper_limit,
                     const int number_points ) {
  std::vector<double> spaced_vector;

  for ( int itr = 0; itr <number_points; itr++) {
    spaced_vector.push_back(lower_limit + (upper_limit - lower_limit)
        * (itr / number_points));
  }
  return spaced_vector;
}

// This test method does a grid search on IK failures in the
// Ranges corresponding to the pick_and_place_demo
void IKGridTest(
    std::vector<double> x_range,
    std::vector<double> y_range,
    std::vector<double> z_range) {

  const double kTableTopZInWorld = 0.7756;

// Coordinates for kRobotBase originally from iiwa_world_demo.cc.
// The intention is to center the robot on the table.
  const Eigen::Vector3d kRobotBase(-0.243716, -0.625087, kTableTopZInWorld);

  const std::string kModelPath =
      GetDrakePath() +
          "/examples/kuka_iiwa_arm/urdf/iiwa14_simplified_collision.urdf";

  auto weld_to_frame = std::allocate_shared < RigidBodyFrame < double >> (
      Eigen::aligned_allocator < RigidBodyFrame < double >> (), "world", nullptr, kRobotBase, Eigen::Vector3d());

  std::unique_ptr <RigidBodyTree<double>> iiwa =
      std::make_unique < RigidBodyTree < double >> ();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      kModelPath, multibody::joints::kFixed, nullptr, iiwa.get());
  const std::string kEndEffectorLinkName = "iiwa_link_ee";


//  int id = tree_builder->AddFixedModelInstance("iiwa", kRobotBase);
//  RigidBodyTree<double> *iiwa = tree_builder->get_model_info_for_instance(id);
//
  IiwaIkPlanner ik_planner(kModelPath, kEndEffectorLinkName, nullptr);
  IKResults ik_res;
  IiwaIkPlanner::IkCartesianWaypoint wp;
  wp.pos_tol = Vector3<double>(0.001, 0.001, 0.001);
  wp.rot_tol = 0.005;
  wp.constrain_orientation = true;
  std::vector <IiwaIkPlanner::IkCartesianWaypoint> waypoints(1, wp);

  const VectorX<double> kQcurrent = iiwa->getZeroConfiguration();
//  VectorX<double> q_fk;
//
//  const double kEpsilon = 1e-8;
//  const Vector3<double> kUpperBound =
//      wp.pos_tol + kEpsilon * Vector3<double>::Ones();
//  const Vector3<double> kLowerBound =
//      -wp.pos_tol - kEpsilon * Vector3<double>::Ones();

  for (std::vector<double>::iterator it_x = x_range.begin();
       it_x != x_range.end(); ++it_x) {
    for (std::vector<double>::iterator it_y = y_range.begin();
         it_y != y_range.end(); ++it_y) {
      for (std::vector<double>::iterator it_z = z_range.begin();
           it_z != z_range.end(); ++it_z)
        waypoints[0].
            pose.translation() = Vector3<double>(*it_x, *it_y, *it_z);
//      std::cout << *it_x << ", " << *it_y << " : ";
      bool ret =
          ik_planner.PlanSequentialTrajectory(waypoints, kQcurrent, &ik_res);
//      if (ret) {
//        std::cout << "TRUE\n";
//      } else {
//        std::cout << "FALSE\n";
//      }
      EXPECT_TRUE(ret);
//      test_result[x_ctr][y_ctr] = ret;

    }
  }
//
//  for (int i = 0; i <num_points; ++i) {
//    for (int j = 0; j<num_points; ++j) {
//      std::cout<<"("<<i<<","<<j<<"):";
//      if(test_result[x_ctr][y_ctr])
//        std::cout<<"TR; ";
//      else
//        std::cout<<"FL; ";
//    }
//    std::cout<<"\n";
//  }
}

GTEST_TEST(testInverseKinematicsRange, range_test) {

// Extracted from the SDF
// Table surface dimensions.
const double kTablex_Length = 0.762;
const double kTabley_Length = 0.7112;

const int kNumXPoints = 3;
const int kNumYPoints = 3;
const int kNumZPoints = 2;

const double kTable0x_Origin = 0.8, kTable0y_Origin = 0;
const double kTable1x_Origin = 0.0, kTable1y_Origin = 0.85;

std::vector<double> z_range = linspace(
    0.926 /* grasp position */,
    1.236 /* pre-grasp position */,
    kNumZPoints
);

// The x and y range corresponding to table 0.
std::vector<double> x_range_0 = linspace(
    kTable0x_Origin - kTablex_Length / 2, kTable0x_Origin + kTablex_Length / 2,
    kNumXPoints);
std::vector<double> y_range_0 = linspace(
    kTable0y_Origin - kTabley_Length / 2, kTable0y_Origin + kTabley_Length / 2,
    kNumYPoints);

IKGridTest(x_range_0, y_range_0, z_range);

// The x and y range corresponding to table 1.
std::vector<double> x_range_1 = linspace(
    kTable1x_Origin - kTablex_Length / 2, kTable1x_Origin + kTablex_Length / 2,
    kNumXPoints);
std::vector<double> y_range_1 = linspace(
    kTable1y_Origin - kTabley_Length / 2, kTable1y_Origin + kTabley_Length / 2,
    kNumYPoints);

IKGridTest(x_range_1, y_range_1, z_range);
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
