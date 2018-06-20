#include "drake/manipulation/scene_generation/random_clutter_generator.h"

#include <algorithm>
#include <set>
#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/math/random_rotation.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/rigid_body_constraint.h"
#include "drake/multibody/rigid_body_ik.h"

namespace drake {
namespace manipulation {
namespace scene_generation {
using Eigen::Isometry3d;
using Eigen::Quaterniond;
using Eigen::VectorXd;
using Eigen::Vector3d;
using std::map;
using std::string;
using std::stringstream;
using std::vector;
using std::default_random_engine;
using std::uniform_real_distribution;

namespace {
Eigen::VectorXi SimpleCumulativeSum(int min_val, int num_elements) {
  Eigen::VectorXi cumsum = Eigen::VectorXi::Constant(num_elements, 0);
  int i = min_val;
  for (int it = 0; it < cumsum.size(); ++it) {
    // drake::log()->info("i: {}",it);
    cumsum(it) = i++;
  }
  return cumsum;
}

VectorX<double> GenerateBoundedRandomSample(
    std::default_random_engine *generator, VectorX<double> min,
    VectorX<double> max) {
  DRAKE_DEMAND(min.size() == max.size());
  VectorX<double> return_vector = VectorX<double>::Zero(min.size());
  for (int i = 0; i < min.size(); ++i) {
    uniform_real_distribution<double> distribution(min(i), max(i));
    return_vector(i) = distribution(*generator);
  }
  return return_vector;
}

Eigen::VectorXd Resize(const Eigen::VectorXd& x, int size) {
  int current_size = x.size();
  Eigen::VectorXd return_vector = x;
  return_vector.resize(current_size + size);
  return_vector.head(current_size) = x;
  return_vector.tail(size) = VectorXd::Zero(size);
  return return_vector;
}

Eigen::VectorXi Resize(const Eigen::VectorXi& x, int size) {
  // drake::log()->info("x : {}", x.transpose());
  int current_size = x.size();
  // drake::log()->info("curret size : {}, requested_size {}", current_size, size);
  Eigen::VectorXi return_vector = x;
  return_vector.resize(current_size + size);
  return_vector.head(current_size) = x;
  return_vector.tail(size) = Eigen::VectorXi::Zero(size);

  // drake::log()->info("return_vector: {}", return_vector.transpose());
  // drake::log()->info(
  //   "original {}, new {}", x.transpose(), return_vector.transpose());
  return return_vector;
}


}  // namespace

RandomClutterGenerator::RandomClutterGenerator(
    RigidBodyTree<double>* scene_tree,
    const std::vector<int>& clutter_model_instances,
    const Vector3<double>& clutter_center, const Vector3<double>& clutter_size,
    double min_inter_object_distance)
    : scene_tree_ptr_(scene_tree),
      clutter_model_instances_(clutter_model_instances),
      clutter_center_(clutter_center),
      clutter_lb_(-0.5 * clutter_size),
      clutter_ub_(0.5 * clutter_size),
      inter_object_distance_(min_inter_object_distance) {
  // Checks that the number of requested instances > 0 & < the total number
  // of instances in the tree.
  DRAKE_DEMAND(clutter_model_instances.size() >= 1);
  for (auto& it : clutter_model_instances) {
    // check that the tree contains the model instance in question. i.e
    // atleast one body exists for each model instance listed in
    // clutter_model_instance.
    DRAKE_DEMAND(scene_tree_ptr_->FindModelInstanceBodies(it).size() > 0);
  }
}

VectorX<double> RandomClutterGenerator::GenerateFloatingClutter(
    const VectorX<double>& q_nominal, std::default_random_engine *generator,
    bool add_z_height_cost, bool return_infeasible_as_well) {
  DRAKE_DEMAND(scene_tree_ptr_->get_num_positions() == q_nominal.size());

  VectorX<double> q_nominal_candidate = q_nominal;
  VectorX<double> q_ik_result = q_nominal;

  int ik_result_code = 100;

  int max_result_code = return_infeasible_as_well? 10 : 1;

  // Keep running the IK until a feasible solution is found.
  while (ik_result_code > max_result_code) {
    drake::log()->debug("IK new run initiated on tree of size {}.",
                        scene_tree_ptr_->get_num_positions());

    // Setup constraint array.
    std::vector<RigidBodyConstraint*> constraint_array;

    // set MinDistanceConstraint
    drake::log()->debug("Adding MinDistanceConstraint.");
    std::vector<int> active_bodies_idx;
    std::set<std::string> active_group_names;
    MinDistanceConstraint min_distance_constraint(
        scene_tree_ptr_, inter_object_distance_ /* min_distance */,
        active_bodies_idx, active_group_names);
    constraint_array.push_back(&min_distance_constraint);

    Eigen::VectorXi linear_posture_iAfun, linear_posture_jAvar;
    VectorX<double> linear_posture_A, linear_posture_lb, linear_posture_ub;
    VectorX<double> q_initial = q_nominal;
    std::vector<int> z_indices;
   // int skip = 0;
    Vector3<double> bounded_position;
    int clutter_body_ctr = 0;
    // Iterate through each of the model instances of the clutter and add
    // elements to the linear posture_constraint.
    for (auto& instance : clutter_model_instances_) {
      auto model_instance_bodies =
          scene_tree_ptr_->FindModelInstanceBodies(instance);
      for (size_t i = 0; i < model_instance_bodies.size(); ++i) {
        auto body = model_instance_bodies[i];
        if (body->has_joint()) {
          const DrakeJoint* joint = &body->getJoint();
          if (!joint->is_fixed()) {
            int joint_dofs = joint->get_num_positions();
            DRAKE_DEMAND(joint_dofs == 1 || joint_dofs == 7 || joint_dofs == 6);
            VectorX<double> joint_lb, joint_ub, joint_initial, joint_nominal;

//            int current_size = linear_posture_iAfun.size();

            switch (joint_dofs) {
              case 7 :
                {/*
                  joint_lb = VectorX<double>::Zero(7);
                  joint_ub = joint_lb;
                  joint_initial = joint_lb;
                  joint_nominal = joint_lb;

                  // Add 7 ints to the iAfun, jAvar
                  // linear_posture_A has 7 ones added.
                  linear_posture_iAfun = Resize(linear_posture_iAfun, 7);
                  linear_posture_iAfun.tail(7) = SimpleCumulativeSum(current_size, 7);
                  linear_posture_jAvar = Resize(linear_posture_jAvar, 7);
                  linear_posture_A = Resize(linear_posture_A, 7);
                  linear_posture_A.tail(7) = VectorX<double>::Ones(7);

                  linear_posture_lb = Resize(linear_posture_lb, 7);
                  linear_posture_ub = Resize(linear_posture_ub, 7);
                  joint_lb.head(3) = clutter_lb_;
                  joint_ub.head(3) = clutter_ub_;

                  auto temp_out = GenerateBoundedRandomSample(
                      generator, clutter_lb_, clutter_ub_);
                  joint_initial.head(3) = temp_out;
                  z_indices.push_back(body->get_position_start_index() + 2);
                  joint_nominal.head(3) = Vector3<double>::Zero(3);

                  // Orientation
                  Eigen::Quaterniond quat =
                      drake::math::UniformlyRandomQuaternion(generator);
                  joint_lb[3] = quat.w();
                  joint_lb[4] = quat.x();
                  joint_lb[5] = quat.y();
                  joint_lb[6] = quat.z();
                  joint_ub.tail(4) = joint_lb.tail(4);
                  linear_posture_lb.tail(7) = joint_lb;
                  linear_posture_ub.tail(7) = joint_ub;

                  joint_initial.tail(4) = joint_ub.tail(4);
                  joint_nominal.tail(4) = joint_ub.tail(4);

                  linear_posture_lb.segment(body->get_position_start_index(),
                                        joint_dofs) = joint_lb;
                  linear_posture_ub.segment(body->get_position_start_index(),
                                        joint_dofs) = joint_ub;*/
                }
                break;

              case 6 :
                {
                  joint_lb = VectorX<double>::Zero(6);
                  joint_ub = joint_lb;
                  joint_initial = VectorX<double>::Zero(6);
                  joint_nominal = VectorX<double>::Zero(6);
                  
                  // drake::log()->info("A. linear_posture_iAfun : {}", linear_posture_iAfun.transpose());
                  linear_posture_iAfun = Resize(linear_posture_iAfun, 3);
                  // drake::log()->info("B. linear_posture_iAfun : {}", linear_posture_iAfun.transpose());
                  linear_posture_iAfun.tail(3) = SimpleCumulativeSum(3 * clutter_body_ctr++, 3);
                  // drake::log()->info("C. linear_posture_iAfun : {}", linear_posture_iAfun.transpose());

                  linear_posture_jAvar = Resize(linear_posture_jAvar, 3);
                  linear_posture_jAvar.tail(3) = SimpleCumulativeSum(body->get_position_start_index(), 3);
                  linear_posture_A = Resize(linear_posture_A, 3);
                  linear_posture_A.tail(3) = VectorX<double>::Ones(3);

                  linear_posture_lb = Resize(linear_posture_lb, 3);
                  linear_posture_ub = Resize(linear_posture_ub, 3);
                  joint_lb = clutter_lb_;
                  joint_ub = clutter_ub_;

                  auto temp_out = GenerateBoundedRandomSample(
                      generator, clutter_lb_, clutter_ub_);
                  joint_initial.head(3) = temp_out;
                  z_indices.push_back(body->get_position_start_index() + 2);
                  joint_nominal.head(3) = Vector3<double>::Zero(3);

                  // Orientation
                  Vector3d rpy =
                      drake::math::UniformlyRandomRPY(generator);
                  joint_initial[3] = rpy[0];
                  joint_initial[4] = rpy[1];
                  joint_initial[5] = rpy[2];
                  
                  linear_posture_lb.tail(3) = joint_lb;
                  linear_posture_ub.tail(3) = joint_ub;

                  // drake::log()->info("joint ini :{}", joint_initial.transpose());
                  // drake::log()->info("joint nom :{}", joint_nominal.transpose());

                  // drake::log()->info("iAfun {}", linear_posture_iAfun.transpose() );
                  // drake::log()->info("jAvar {}", linear_posture_jAvar.transpose() );
                  // drake::log()->info("lb {}", linear_posture_lb.transpose() );
                  // drake::log()->info("ub {}", linear_posture_ub.transpose() );

                }
                break;

              case 1 : 
                {/*
                  joint_lb = VectorX<double>::Zero(1);
                  joint_ub = joint_lb;
                  joint_initial = VectorX<double>::Zero(1);
                  joint_nominal = VectorX<double>::Zero(1);

                  linear_posture_iAfun = Resize(linear_posture_iAfun, 1);
                  linear_posture_iAfun.tail(1) = SimpleCumulativeSum(
                    current_size, 1);
                  linear_posture_jAvar = Resize(linear_posture_jAvar, 1);
                  linear_posture_A = Resize(linear_posture_A, 1);
                  linear_posture_A.tail(1) = VectorX<double>::Ones(1);

                  linear_posture_lb = Resize(linear_posture_lb, 1);
                  linear_posture_ub = Resize(linear_posture_ub, 1);
                  
                  joint_lb[0] = joint->getJointLimitMin()[0];
                  joint_ub[0] = joint->getJointLimitMin()[0];
                  joint_initial = GenerateBoundedRandomSample(
                    generator, joint_lb, joint_ub);
                  joint_nominal = joint_initial;

                  linear_posture_lb.segment(body->get_position_start_index(),
                                        joint_dofs) = joint_lb;
                  linear_posture_ub.segment(body->get_position_start_index(),
                                        joint_dofs) = joint_ub;
                  skip = 0;*/
                }
                break;

            }
            q_initial.segment(body->get_position_start_index(), joint_dofs) =
                joint_initial;
            q_nominal_candidate.segment(body->get_position_start_index(),
                                        joint_dofs) = joint_nominal;
          }
        }
      }
    }



    linear_posture_A = VectorX<double>::Ones(linear_posture_iAfun.size());

    // drake::log()->debug("lb num {}", linear_posture_lb.size());
    // drake::log()->debug("ub num {}", linear_posture_ub.size());
    // drake::log()->debug("A num {}", linear_posture_A.size());
    // drake::log()->debug("iAfun {}", linear_posture_iAfun.size());
    // drake::log()->debug("jAvar {}", linear_posture_jAvar.size());


    // drake::log()->info("lb num {} :{}", linear_posture_lb.size(), linear_posture_lb.transpose());
    // drake::log()->info("ub num {} :{}", linear_posture_ub.size(), linear_posture_ub.transpose());
    // drake::log()->info("A num {} :{}", linear_posture_A.size(), linear_posture_A.transpose());
    // drake::log()->info("iAfun {} :{}", linear_posture_iAfun.size(), linear_posture_iAfun.transpose());
    // drake::log()->info("jAvar {} :{}", linear_posture_jAvar.size(), linear_posture_jAvar.transpose());

       drake::log()->debug("Adding SingleTimeLinearPostureConstraint");

    SingleTimeLinearPostureConstraint linear_posture_constraint(
        scene_tree_ptr_, linear_posture_iAfun, linear_posture_jAvar,
        linear_posture_A, linear_posture_lb, linear_posture_ub);

    constraint_array.push_back(&linear_posture_constraint);
    drake::log()->debug("Constraint array size {}", constraint_array.size());

    Eigen::MatrixXd Q_candidate =
        Eigen::MatrixXd::Zero(q_initial.size(), q_initial.size());

    if (add_z_height_cost) {
      for (auto& it : z_indices) {
        Q_candidate(it, it) = 100;
      }
    }

    IKoptions ikoptions(scene_tree_ptr_);
    ikoptions.setQ(Q_candidate);
    ikoptions.setDebug(true);

    // setup IK problem and run.
    IKResults ik_results =
        inverseKinSimple(scene_tree_ptr_, q_initial, q_nominal_candidate,
                         constraint_array, ikoptions);

    for (auto it : ik_results.info) {
      drake::log()->info("IK Result code : {}", it);
      ik_result_code = it;
      if (ik_result_code > max_result_code) {
        drake::log()->debug("IK failure, recomputing IK");
      }
    }

    if (!ik_results.q_sol.empty()) {
      q_ik_result = ik_results.q_sol.back();
    }
  }
  return q_ik_result;
}

}  // namespace scene_generation
}  // namespace manipulation
}  // namespace drake
