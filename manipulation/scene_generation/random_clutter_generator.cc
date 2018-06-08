#include "drake/manipulation/scene_generation/random_clutter_generator.h"

#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/manipulation/util/simple_tree_visualizer.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/math/random_rotation.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/rigid_body_constraint.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

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
using util::WorldSimTreeBuilder;
using std::default_random_engine;
using std::uniform_real_distribution;
using systems::RigidBodyPlant;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::Simulator;
using systems::RungeKutta2Integrator;
using systems::ContinuousState;
using util::ModelInstanceInfo;

namespace {
// VectorX<double> Isometry3dToVector(const Isometry3d& pose) {
//   VectorXd return_vector = VectorXd::Zero(7);
//   return_vector.head(3) = pose.translation();
//   Quaterniond return_quat = Quaterniond(pose.linear());
//   return_vector.tail(4) = (VectorXd(4) << return_quat.w(), return_quat.x(),
//                            return_quat.y(), return_quat.z())
//                               .finished();
//   return return_vector;
// }

// Isometry3<double> VectorToIsometry3d(const VectorX<double>& q) {
//   Isometry3<double> return_pose = Isometry3<double>::Identity();
//   return_pose.translation() = q.head(3);
//   Quaterniond return_quat = Quaterniond(q(3), q(4), q(5), q(6));
//   return_pose.linear() = return_quat.toRotationMatrix();
//   return return_pose;
// }

// template <typename T>
// void AppendTo(T* t, const T& t_to_append) {
//   int append_size = t_to_append.size();
//   T t_copy = *t;
//   int old_size = t->size();
//   t->resize(old_size + append_size);
//   t->head(old_size) = t_copy;
//   t->tail(append_size) = t_to_append;
// }

Eigen::VectorXi SimpleCumulativeSum(int num_elements) {
  Eigen::VectorXi cumsum = Eigen::VectorXi::Constant(num_elements, 0);
  int i = 0;
  for (int it = 0; it < cumsum.size(); ++it) {
    cumsum(it) = i++;
  }
  return cumsum;
}

}  // namespace

// const double kRandomInitializationDimensionRatio = 0.7;

RandomClutterGenerator::RandomClutterGenerator(
    std::unique_ptr<RigidBodyTreed> scene_tree,
    std::vector<int> clutter_model_instances, Vector3<double> clutter_center,
    Vector3<double> clutter_size, bool visualize_steps,
    double min_inter_object_distance)
    : scene_tree_ptr_(scene_tree.get()),
      clutter_model_instances_(clutter_model_instances),
      clutter_center_(clutter_center),
      clutter_lb_(-0.5 * clutter_size),
      clutter_ub_(0.5 * clutter_size),
      visualize_steps_(visualize_steps),
      inter_object_distance_(min_inter_object_distance) {
  // Checks that the number of requested instances > 0 & < the total number
  // of instances in the tree.

  DRAKE_DEMAND(clutter_model_instances.size() >= 1);
  for (auto& it : clutter_model_instances) {
    // check that the tree contains the model instance in question. i.e
    // atleast one body exists for each model instance listed in
    // clutter_model_instance.
    DRAKE_DEMAND(scene_tree->FindModelInstanceBodies(it).size() > 0);
  }

  fall_sim_diagram_ = GenerateFallSimDiagram(std::move(scene_tree));

  // distribution_map_["length"] =
  //     normal_distribution<double>(-clutter_size_[0] / 2, +clutter_size_[0] /
  //     2);
  // distribution_map_["breadth"] =
  //     normal_distribution<double>(-clutter_size_[1] / 2, +clutter_size_[1] /
  //     2);
  // distribution_map_["height"] =
  //     normal_distribution<double>(-clutter_size_[2] / 2, +clutter_size_[2] /
  //     2);
  // distribution_map_["orientation"] = normal_distribution<double>();
}

// Isometry3<double> RandomClutterGenerator::GenerateRandomBoundedPose() {
//   Quaterniond random_quat = Quaterniond::UnitRandom();
//   Isometry3d return_pose = Isometry3d::Identity();
//   return_pose.linear() = random_quat.toRotationMatrix();

//   uniform_real_distribution<double> x_distribution(
//       -clutter_size_(0) * (kRandomInitializationDimensionRatio / 2),
//       clutter_size_(0) * (kRandomInitializationDimensionRatio / 2));

//   uniform_real_distribution<double> y_distribution(
//       -clutter_size_(1) * (kRandomInitializationDimensionRatio / 2),
//       clutter_size_(1) * (kRandomInitializationDimensionRatio / 2));

//   uniform_real_distribution<double> z_distribution(
//       -clutter_size_(2) * (kRandomInitializationDimensionRatio / 2),
//       clutter_size_(2) * (kRandomInitializationDimensionRatio / 2));

//   return_pose.translation() =
//       (VectorXd(3) << x_distribution(generator_), y_distribution(generator_),
//        z_distribution(generator_))
//           .finished();
//   return_pose.makeAffine();

//   Eigen::Isometry3d X_WB = Eigen::Isometry3d::Identity();
//   X_WB.translation() = clutter_center_;
//   // return pose is computed in the Bounding box frame B. Transform and
//   return
//   // pose in World frame (W).
//   return X_WB * return_pose;
// }

// VectorX<double> RandomClutterGenerator::GetRandomBoundedConfiguration(
//     const RigidBodyTree<double>* scene_tree,
//     std::vector<int> clutter_model_instances, const VectorX<double> q_inital)
//     {
//   VectorX<double> q_random = q_inital;

//   VectorX<double> x = VectorX<double>::Zero(scene_tree->get_num_positions());

//   // iterate through clutter elements in the tree. Set random poses for each
//   // element.
//   for (auto it : clutter_model_instances) {
//     auto base_body_index = scene_tree->FindBaseBodies(it);

//     // TODO(naveenoid) : resolve multi-link model initialization.
//     std::vector<const RigidBody<double>*> model_instance_bodies =
//         scene_tree->FindModelInstanceBodies(it);
//     Isometry3d segment_pose = GenerateRandomBoundedPose();
//     q_random.segment<7>(model_instance_bodies[0]->get_position_start_index())
//     =
//         Isometry3dToVector(segment_pose);
//   }
//   return q_random;
// }

// VectorX<double> RandomClutterGenerator::GetNominalConfiguration(
//     const RigidBodyTreed& model_tree) {
//   VectorX<double> q_zero =
//       VectorX<double>::Zero(model_tree.get_num_positions());

//   int num_model_instances = model_tree.get_num_model_instances();
//   VectorX<double> x = VectorX<double>::Zero(model_tree.get_num_positions());

//   // iterate through tree. Set random poses for each floating element.
//   for (int i = 0; i < num_model_instances; ++i) {
//     auto base_body_index = model_tree.FindBaseBodies(i);
//     // TODO(naveenoid) : figure out if the model instance has more than 1
//     link
//     std::vector<const RigidBody<double>*> model_instance_bodies =
//         model_tree.FindModelInstanceBodies(i);\
//     Isometry3d segment_pose = Isometry3d::Identity();
//     q_zero.segment<7>(model_instance_bodies[0]->get_position_start_index()) =
//         Isometry3dToVector(segment_pose);
//   }
//   return q_zero;
// }

VectorX<double> RandomClutterGenerator::Generate(
    const VectorX<double> q_nominal, std::default_random_engine& generator) {
  DRAKE_DEMAND(scene_tree_ptr_->get_num_positions() == q_nominal.size());
  /*
    if (visualize_steps_) {
      SimpleTreeVisualizer visualizer(*scene_tree.get(), lcm_);
    }*/
  VectorX<double> q_ik_result = q_nominal;

  int ik_result_code = 100;

  // Keep running the IK until a feasible solution is found.
  while (ik_result_code != 1) {
    drake::log()->info("IK new run initiated on tree of size {}.", 
      scene_tree_ptr_->get_num_positions());

    // VectorX<double> q_nominal = GetRandomBoundedConfiguration(
    //     scene_tree.get(), clutter_model_instances, q_initial);

    // Setup constraint array.
    std::vector<RigidBodyConstraint*> constraint_array;

    // set MinDistanceConstraint
    drake::log()->info("MinDistanceConstraint added.");
    std::vector<int> active_bodies_idx;
    std::set<std::string> active_group_names;
    MinDistanceConstraint min_distance_constraint(
        scene_tree_ptr_, inter_object_distance_ /* min_distance */,
        active_bodies_idx, active_group_names);
    constraint_array.push_back(&min_distance_constraint);

    Eigen::VectorXi linear_posture_iAfun =
                        SimpleCumulativeSum(q_nominal.size()),
                    linear_posture_jAvar = linear_posture_iAfun;
    VectorX<double> linear_posture_A = VectorX<double>::Ones(q_nominal.size()),
                    linear_posture_lb = q_nominal,
                    linear_posture_ub = q_nominal;

    drake::log()->info(
        "Looping through clutter instances to add LinearConstraint.");
    // Iterate through each of the model instances of the clutter and add
    // elements
    // to the linear posture_constraint.
    for (auto& instance : clutter_model_instances_) {
      auto model_instance_bodies =
          scene_tree_ptr_->FindModelInstanceBodies(instance);
      // drake::log()->info("Checking instance {}, num bodies : {}", 
        // instance, model_instance_bodies.size());
      for (size_t i = 0; i < model_instance_bodies.size(); ++i) {
        auto body = model_instance_bodies[i];
        // drake::log()->info("Checking instance {}, body id {}", instance,
                           // body->get_body_index());
        if (body->has_joint()) {
          // drake::log()->info("body id {} has a joint", body->get_body_index());
          const DrakeJoint* joint = &body->getJoint();
          if (!joint->is_fixed()) {
            int joint_dofs = joint->get_num_positions();
            // drake::log()->info("body id {} has a floating joint of dim {}", 
            //   body->get_body_index(), joint_dofs);

            // Enforces checks only for revolute and floating quaternion joints.
            DRAKE_DEMAND(joint_dofs == 1 || joint_dofs == 7);
            VectorX<double> joint_lb = VectorX<double>::Zero(joint_dofs);
            VectorX<double> joint_ub = joint_lb;

            if (joint_dofs == 7) {
              // If the num dofs of the joint is 7 its a floating quaternion
              // joint.
              // Need to set the position part to be bounded in position.
              // Need to set the orientation part to be a random quaternion.

              // Position
              joint_lb.head(3) = clutter_lb_;
              joint_ub.head(3) = clutter_ub_;

              // Orientation
              Eigen::Quaterniond quat =
                  drake::math::UniformlyRandomQuaternion(&generator);
              joint_lb[3] = quat.w();
              joint_lb[4] = quat.x();
              joint_lb[5] = quat.y();
              joint_lb[6] = quat.z();

              // drake::log()->info("Random quat: [{},{}, {}, {}]", 
              //   quat.w(), quat.x(), quat.y(), quat.z());

              joint_ub.tail(4) = joint_lb.tail(4);

            } else if (joint_dofs == 1) {
              // If the num dofs of the joint is 1, set lb, ub to joint_lim_min,
              // joint_lim_max.

              joint_lb[0] = joint->getJointLimitMin()[0];
              joint_ub[0] = joint->getJointLimitMin()[0];

              drake::log()->info("Adding world position constraint");
              // set WorldPositionConstraint (bounds every object to the
              // bounding box)
              WorldPositionConstraint world_position_constraint(
                  scene_tree_ptr_, body->get_body_index(),
                  Eigen::Vector3d::Zero(), clutter_lb_, clutter_ub_);
              constraint_array.push_back(&world_position_constraint);
            }
            // drake::log()->info("Joint lb:[{}], ub:[{}]", 
            //   joint_lb.transpose(), joint_ub.transpose());

            linear_posture_lb.segment(body->get_position_start_index(),
                                      joint_dofs) = joint_lb;
            linear_posture_ub.segment(body->get_position_start_index(),
                                      joint_dofs) = joint_ub;
          }
        }
        // drake::log()->info("Finished checking body {}", body->get_body_index());
      }
      // drake::log()->info("Finished checking model instance {}", instance);
    }

    linear_posture_A = VectorX<double>::Ones(linear_posture_iAfun.size());

    drake::log()->info("About to add SingleTimeLinearPostureConstraint");

    // drake::log()->info("linear_posture_ub : {}", linear_posture_ub.transpose());
    // drake::log()->info("linear_posture_lb : {}", linear_posture_lb.transpose());
    // drake::log()->info("linear_posture_iAfun : {}", linear_posture_iAfun.transpose());
    // drake::log()->info("linear_posture_A : {}", linear_posture_A.transpose());
    // Adding a single linear constraint for setting all orientations to
    // their
    // (initial) random orientations. This is needed to ensure that
    // feasible
    // orientations are generated as a result of the IK.
    SingleTimeLinearPostureConstraint linear_posture_constraint(
        scene_tree_ptr_, linear_posture_iAfun, linear_posture_jAvar,
        linear_posture_A, linear_posture_lb, linear_posture_ub);

    
    VectorX<double> q_initial = 0.5 * (linear_posture_lb + linear_posture_ub);

    // drake::log()->info("q_nominal : {}", q_nominal.transpose());
    // drake::log()->info("q_initial : {}", q_nominal.transpose());
    constraint_array.push_back(&linear_posture_constraint);

    drake::log()->info("Constraint array size {}", constraint_array.size());

    IKoptions ikoptions(scene_tree_ptr_);
    ikoptions.setDebug(true);

    // setup IK problem and run.
    IKResults ik_results = inverseKinSimple(
        scene_tree_ptr_, q_initial, q_nominal, constraint_array, ikoptions);

    int indx = 0;
    for (auto it : ik_results.info) {
      drake::log()->info(" IK Result code {} : {}", indx++, it);
      ik_result_code = it;
      if (ik_result_code != 1) {
        drake::log()->info("IK failure, recomputing IK");
      }
    }

    if (!ik_results.q_sol.empty()) {
      q_ik_result = ik_results.q_sol.back();
    }
  }

  // drake::log()->info("q_ik_result : {}", q_ik_result);
  drake::log()->info("About to simulate dropping");

  // Simulate Fall and return the state.
  return DropObjectsToGround(q_ik_result);
}

VectorX<double> RandomClutterGenerator::DropObjectsToGround(
    const VectorX<double>& q_ik) {
  Simulator<double> simulator(*fall_sim_diagram_);

  // setting initial condition
  // auto diagram_context = sys->CreateDefaultContext();
  VectorX<double> x_initial =
      VectorX<double>::Zero(scene_tree_ptr_->get_num_positions() +
                            scene_tree_ptr_->get_num_velocities());

  x_initial.head(scene_tree_ptr_->get_num_positions()) = q_ik;
  simulator.get_mutable_context()
      .get_mutable_continuous_state_vector()
      .SetFromVector(x_initial);

  simulator.Initialize();

  if (visualize_steps_) {
    // simulator.set_target_realtime_rate(1.0);
  }
  simulator.reset_integrator<RungeKutta2Integrator<double>>(
      *fall_sim_diagram_, 0.0001, &simulator.get_mutable_context());
  simulator.get_mutable_integrator()->set_maximum_step_size(0.001);
  simulator.get_mutable_integrator()->set_fixed_step_mode(true);

  drake::log()->info("Starting Simulation");
  simulator.StepTo(1.1);

  drake::log()->info("Copying the return vector");
  VectorX<double> q_return =
      simulator.get_context().get_continuous_state_vector().CopyToVector();

  return q_return;
}

std::unique_ptr<systems::Diagram<double>>
RandomClutterGenerator::GenerateFallSimDiagram(
    std::unique_ptr<RigidBodyTreed> scene_tree) {
  DiagramBuilder<double> builder;

  // Transferring ownership of tree to the RigidBodyPlant.
  auto plant = builder.template AddSystem<systems::RigidBodyPlant<double>>(
      std::move(scene_tree));
  plant->set_name("RBP");

  systems::CompliantMaterial default_material;
  default_material
      .set_youngs_modulus(1e7)  // Pa
      .set_dissipation(2)       // s/m
      .set_friction(0.9, 0.5);
  plant->set_default_compliant_material(default_material);

  systems::CompliantContactModelParameters model_parameters;
  model_parameters.characteristic_radius = 2e-4;  // m
  model_parameters.v_stiction_tolerance = 0.01;   // m/s
  plant->set_contact_model_parameters(model_parameters);

  if (visualize_steps_) {
    auto drake_visualizer =
        builder.template AddSystem<systems::DrakeVisualizer>(
            plant->get_rigid_body_tree(), &lcm_);
    drake_visualizer->set_name("DV");

    builder.Connect(plant->get_output_port(0),
                    drake_visualizer->get_input_port(0));
  }

  if (plant->get_num_actuators() > 0) {
    auto zero_input = builder.template AddSystem<systems::ConstantVectorSource>(
        Eigen::VectorXd::Zero(plant->get_num_actuators()));
    builder.Connect(zero_input->get_output_port(), plant->get_input_port(0));
  }

  return builder.Build();
}

}  // namespace scene_generation
}  // namespace manipulation
}  // namespace drake