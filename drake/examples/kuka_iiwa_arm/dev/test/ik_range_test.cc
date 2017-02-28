#include "gtest/gtest.h"

#include <vector>

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/dev/iiwa_ik_planner.h"
//#include "drake/ex
#include "drake/multibody/parsers/urdf_parser.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

inline double get_orientation_difference(const Matrix3<double>& rot0,
                                         const Matrix3<double>& rot1) {
  AngleAxis<double> err(rot0.transpose() * rot1);
  return err.angle();
}


std::vector<double> linspace(const double lower_limit, const double upper_limit,
                     const int number_points ) {
  std::vector<double> spaced_vector;

  for ( int itr = 0; itr <number_points; itr++) {
    spaced_vector.push_back(lower_limit + (upper_limit - lower_limit)
        * (itr / number_points));
  }

  return spaced_vector;
}

// With the current setup, this code simply searches for IK failures in the
// Ranges corresponding to the IK demo.
void IKGridTest(
    const double x_min, const double x_max, const int x_num_points,
    const double y_min, const double y_max, const int y_num_points,
    const double z) {

  // TODO(naveenoid): Extract this tree from a common factory builder once
  // its extracted from iiwa_wsg_sim.
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  tree_builder->StoreModel("iiwa",
                           "/examples/kuka_iiwa_arm/urdf/"
                               "iiwa14_simplified_collision.urdf");
  tree_builder->StoreModel("table",
                           "/examples/kuka_iiwa_arm/models/table/"
                               "extra_heavy_duty_table_surface_only_collision.sdf");
  tree_builder->StoreModel(
      "wsg", "/examples/schunk_wsg/models/schunk_wsg_50.sdf");

  // Build a world with two fixed tables.  A box is placed one on
  // table, and the iiwa arm is fixed to the other.
  // IIWA table
  tree_builder->AddFixedModelInstance("table",
                                      Eigen::Vector3d::Zero() /* xyz */,
                                      Eigen::Vector3d::Zero() /* rpy */);
  // Table 0 for pick and place demo.
  tree_builder->AddFixedModelInstance("table",
                                      Eigen::Vector3d(0.8, 0, 0) /* xyz */,
                                      Eigen::Vector3d::Zero() /* rpy */);
  // Table 1 for pick and place demo.
  tree_builder->AddFixedModelInstance("table",
                                      Eigen::Vector3d(0, 0.85, 0) /* xyz */,
                                      Eigen::Vector3d::Zero() /* rpy */);

  // The `z` coordinate of the top of the table in the world frame.
  const double kTableTopZInWorld = 0.7756;

  // Coordinates for kRobotBase originally from iiwa_world_demo.cc.
  // The intention is to center the robot on the table.
  const Eigen::Vector3d kRobotBase(-0.243716, -0.625087, kTableTopZInWorld);

  int id = tree_builder->AddFixedModelInstance("iiwa", kRobotBase);
  *iiwa_instance = tree_builder->get_model_info_for_instance(id);


  const std::string kEndEffectorLinkName = "iiwa_link_ee";

  std::vector<double> x_range = linspace(x_min, x_max, x_num_points);
  std::vector<double> y_range = linspace(y_min, y_max, y_num_points);

  IiwaIkPlanner ik_planner(kModelPath, kEndEffectorLinkName, nullptr);
  IiwaIkPlanner::IkResult ik_res;
  IiwaIkPlanner::IkCartesianWaypoint wp;
  wp.time = 0;
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

  int x_ctr = 0, y_ctr = 0;
  for (std::vector<double>::iterator it_x = x_range.begin();
       it_x != x_range.end(); ++it_x, ++x_ctr) {
    for (std::vector<double>::iterator it_y = y_range.begin();
         it_y != y_range.end(); ++it_y, ++y_ctr) {

      waypoints[0].
          pose.translation() = Vector3<double>(*it_x, *it_y, z);

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


GTEST_TEST(testInverseKinematicsRange, Table1PreGraspTest) {
IKGridTest(
-0.3556 /* x_min */, 7.3556 /* x_max */, 10 /* num_x */,
0.381 /* y_min */, 1.231 /* y_max */, 10, 0.736+ 0.057/2

);


GTEST_TEST(testInverseKinematicsRange, Table2PreGraspTest) {
IKGridTest(
-0.3556 /* x_min */, 7.3556 /* x_max */, 10 /* num_x */,
0.381 /* y_min */, 1.231 /* y_max */, 10, 0.736+ 0.057/2

);
}
}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
//
//int main(int argc, char* argv[]) {
//  gflags::ParseCommandLineFlags(&argc, &argv, true);
//  return drake::examples::kuka_iiwa_arm::DoMain();
//}
