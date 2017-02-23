#include "gtest/gtest.h"

#include <vector>

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/dev/iiwa_ik_planner.h"
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
}

std::vector<double> linspace(const double lower_limit, const double upper_limit,
                     const int number_points ) {
  std::vector<double> spaced_vector;

  for ( int itr = 0; itr <number_points; its++) {
    spaced_vector.push_back(lower_limit + (upper_limit - lower_limit)
        * (its / number_points));
  }

  return spaced_vector;
}



// With the current setup, this code simply searches for IK failures in the
// Ranges corresponding to the IK demo.


void rangeTest() {
const std::string kModelPath =
    GetDrakePath() +
        "/examples/kuka_iiwa_arm/urdf/iiwa14_simplified_collision.urdf";
const std::string kEndEffectorLinkName = "iiwa_link_ee";
//std::default_random_engine rand_generator(1234);

std::unique_ptr<RigidBodyTree<double>> iiwa =
    std::make_unique<RigidBodyTree<double>>();
drake::parsers::urdf::AddModelInstanceFromUrdfFile(
    kModelPath, multibody::joints::kFixed, nullptr, iiwa.get());

KinematicsCache<double> cache = iiwa->CreateKinematicsCache();
const RigidBody<double>* end_effector = iiwa->FindBody(kEndEffectorLinkName);

// Setup ranges
const double x_min = -0.3556, x_max = 0.3556;
const double y_min = 0.381, y_max = 1.231;
const int num_points = 10;

const vector<double> x_range = linspace(x_min, x_max, num_points);
const vector<double> y_range = linspace(y_min, y_max, num_points);
const double z = 0.736 + 0.057 / 2;

IiwaIkPlanner ik_planner(kModelPath, kEndEffectorLinkName, nullptr);
IiwaIkPlanner::IkResult ik_res;
IiwaIkPlanner::IkCartesianWaypoint wp;
wp.time = 0;
wp.pos_tol = Vector3<double>(0.001, 0.001, 0.001);
wp.rot_tol = 0.005;
wp.constrain_orientation = true;
std::vector<IiwaIkPlanner::IkCartesianWaypoint> waypoints(1, wp);

const VectorX<double> kQcurrent = iiwa->getZeroConfiguration();
VectorX<double> q_fk;

const double kEpsilon = 1e-8;
const Vector3<double> kUpperBound =
    wp.pos_tol + kEpsilon * Vector3<double>::Ones();
const Vector3<double> kLowerBound =
    -wp.pos_tol - kEpsilon * Vector3<double>::Ones();

for (std::vector<double>::iterator it_x = x_range.begin();
it_x!= x_range.end(); ++it_x) {
for (std::vector<double>::iterator it_y = y_range.begin();
it_y!= y_range.end(); ++it_y) {

waypoints[0].
pose = Isometry3<double>(*it_x, *it_y, z);

std::cout<<*it_x<<", "<<*it_y<<" : ";
bool ret =
    ik_planner.PlanSequentialTrajectory(waypoints, kQcurrent, &ik_res);
if(ret) {
std::cout<<"TRUE\n";
} else {
std::cout<<"FALSE\n";
}
//EXPECT_TRUE(ret);

}
}
//
//
///////////////////
//      for (int i = 0; i < 100; ++i) {
//      q_fk = iiwa->getRandomConfiguration(rand_generator);
//      cache.initialize(q_fk);
//      iiwa->doKinematics(cache);
//
//      Isometry3<double> fk_pose =
//          iiwa->CalcBodyPoseInWorldFrame(cache, *end_effector);
//      waypoints[0].pose = fk_pose;
//
//      bool ret =
//          ik_planner.PlanSequentialTrajectory(waypoints, kQcurrent, &ik_res);
//      EXPECT_TRUE(ret);
//
//      cache.initialize(ik_res.q.col(1));
//      iiwa->doKinematics(cache);
//      Isometry3<double> ik_pose =
//          iiwa->CalcBodyPoseInWorldFrame(cache, *end_effector);
////      Vector3<double> pos_diff = ik_pose.translation() - fk_pose.translation();
////      double rot_diff =
////          get_orientation_difference(ik_pose.linear(), fk_pose.linear());
//
////      EXPECT_TRUE((pos_diff.array() <= kUpperBound.array()).all());
////      EXPECT_TRUE((pos_diff.array() >= kLowerBound.array()).all());
//
//      // cos(ang_diff) >= cos(tol) is the actual constraint in the IK.
//      EXPECT_TRUE(std::cos(rot_diff) + kEpsilon >= std::cos(wp.rot_tol));
//      }
}
//
//int DoMain() {
////  DRAKE_DEMAND(FLAGS_simulation_sec > 0);
////
////  KukaDemo<double> model(MakePlan());
////  Simulator<double> simulator(model);
////  Context<double>* context = simulator.get_mutable_context();
////
////  VectorX<double> desired_state = VectorX<double>::Zero(14);
////  model.get_kuka_plant().set_state_vector(model.get_kuka_context(context),
////                                          desired_state);
////
////  simulator.Initialize();
////  simulator.set_target_realtime_rate(1.0);
////
////  simulator.StepTo(FLAGS_simulation_sec);
//
//  rangeTest();
//  return 0;
//}

GTEST_TEST(testInverseKinematicsRange, RangeTest) {
rangeTest();
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
