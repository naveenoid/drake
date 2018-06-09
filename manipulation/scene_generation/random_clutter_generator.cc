#include "drake/manipulation/scene_generation/random_clutter_generator.h"

#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
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

Eigen::VectorXi SimpleCumulativeSum(int num_elements) {
  Eigen::VectorXi cumsum = Eigen::VectorXi::Constant(num_elements, 0);
  int i = 0;
  for (int it = 0; it < cumsum.size(); ++it) {
    cumsum(it) = i++;
  }
  return cumsum;
}

VectorX<double> GenerateBoundedRandomSample(
    std::default_random_engine& generator, VectorX<double> min,
    VectorX<double> max) {
  DRAKE_DEMAND(min.size() == max.size());
  VectorX<double> return_vector = VectorX<double>::Zero(min.size());
  for (int i = 0; i < min.size(); ++i) {
    uniform_real_distribution<double> distribution(min(i), max(i));
    return_vector(i) = distribution(generator);
  }
  return return_vector;
}

}  // namespace

void VisualizeSimulationLcm(lcm::DrakeLcm* lcm,
                            systems::DiagramBuilder<double>* builder,
                            systems::RigidBodyPlant<double>* plant) {
  auto drake_visualizer = builder->template AddSystem<systems::DrakeVisualizer>(
      plant->get_rigid_body_tree(), lcm);
  drake_visualizer->set_name("DrakeViz");

  builder->Connect(plant->get_output_port(0),
                   drake_visualizer->get_input_port(0));
}

RandomClutterGenerator::RandomClutterGenerator(
    std::unique_ptr<RigidBodyTreed> scene_tree,
    std::vector<int> clutter_model_instances, Vector3<double> clutter_center,
    Vector3<double> clutter_size, const VisualizeSimulationCallback& visualize,
    double min_inter_object_distance)
    : scene_tree_ptr_(scene_tree.get()),
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
    DRAKE_DEMAND(scene_tree->FindModelInstanceBodies(it).size() > 0);
  }
  fall_sim_diagram_ = GenerateFallSimDiagram(std::move(scene_tree), visualize);
}

VectorX<double> RandomClutterGenerator::GenerateFloatingClutter(
    const VectorX<double> q_nominal, std::default_random_engine& generator) {
  DRAKE_DEMAND(scene_tree_ptr_->get_num_positions() == q_nominal.size());

  VectorX<double> q_ik_result = q_nominal;

  int ik_result_code = 100;
  // Keep running the IK until a feasible solution is found.
  while (ik_result_code != 1) {
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

    Eigen::VectorXi linear_posture_iAfun =
                        SimpleCumulativeSum(q_nominal.size()),
                    linear_posture_jAvar = linear_posture_iAfun;
    VectorX<double> linear_posture_A = VectorX<double>::Ones(q_nominal.size()),
                    linear_posture_lb = q_nominal,
                    linear_posture_ub = q_nominal;
    VectorX<double> q_initial = q_nominal;

    Vector3<double> bounded_position;
    // Iterate through each of the model instances of the clutter and add
    // elements
    // to the linear posture_constraint.
    for (auto& instance : clutter_model_instances_) {
      auto model_instance_bodies =
          scene_tree_ptr_->FindModelInstanceBodies(instance);
      for (size_t i = 0; i < model_instance_bodies.size(); ++i) {
        auto body = model_instance_bodies[i];
        if (body->has_joint()) {
          const DrakeJoint* joint = &body->getJoint();
          if (!joint->is_fixed()) {
            int joint_dofs = joint->get_num_positions();
            // Enforces checks only for Floating quaternion joints.
            DRAKE_DEMAND(joint_dofs == 1 || joint_dofs == 7);
            VectorX<double> joint_lb = VectorX<double>::Zero(joint_dofs);
            VectorX<double> joint_ub = joint_lb;
            VectorX<double> joint_initial = joint_ub;
            if (joint_dofs == 7) {
              // If the num dofs of the joint is 7 its a floating quaternion
              // joint.The position part to be bounded by the clutter bounding
              // box, the orientation part to be a random quaternion.
              // Position
              joint_lb.head(3) = clutter_lb_;
              joint_ub.head(3) = clutter_ub_;
              auto temp_out = GenerateBoundedRandomSample(
                  generator, clutter_lb_, clutter_ub_);
              joint_initial.head(3) = temp_out;

              // Orientation
              Eigen::Quaterniond quat =
                  drake::math::UniformlyRandomQuaternion(&generator);
              joint_lb[3] = quat.w();
              joint_lb[4] = quat.x();
              joint_lb[5] = quat.y();
              joint_lb[6] = quat.z();
              joint_ub.tail(4) = joint_lb.tail(4);
              joint_initial.tail(4) = joint_ub.tail(4);
            } else if (joint_dofs == 1) {
              // If the num dofs of the joint is 1, set lb, ub to joint_lim_min,
              // joint_lim_max.
              joint_lb[0] = joint->getJointLimitMin()[0];
              joint_ub[0] = joint->getJointLimitMin()[0];
              joint_initial =
                  GenerateBoundedRandomSample(generator, joint_lb, joint_ub);
            }
            linear_posture_lb.segment(body->get_position_start_index(),
                                      joint_dofs) = joint_lb;
            linear_posture_ub.segment(body->get_position_start_index(),
                                      joint_dofs) = joint_ub;
            q_initial.segment(body->get_position_start_index(), joint_dofs) =
                joint_initial;
          }
        }
      }
    }

    linear_posture_A = VectorX<double>::Ones(linear_posture_iAfun.size());

    drake::log()->debug("Adding SingleTimeLinearPostureConstraint");

    SingleTimeLinearPostureConstraint linear_posture_constraint(
        scene_tree_ptr_, linear_posture_iAfun, linear_posture_jAvar,
        linear_posture_A, linear_posture_lb, linear_posture_ub);

    constraint_array.push_back(&linear_posture_constraint);
    drake::log()->debug("Constraint array size {}", constraint_array.size());

    IKoptions ikoptions(scene_tree_ptr_);
    ikoptions.setQ(Eigen::MatrixXd::Zero(q_initial.size(), q_initial.size()));
    ikoptions.setDebug(true);

    // setup IK problem and run.
    IKResults ik_results = inverseKinSimple(
        scene_tree_ptr_, q_initial, q_nominal, constraint_array, ikoptions);

    for (auto it : ik_results.info) {
      drake::log()->info("IK Result code : {}", it);
      ik_result_code = it;
      if (ik_result_code != 1) {
        drake::log()->debug("IK failure, recomputing IK");
      }
    }

    if (!ik_results.q_sol.empty()) {
      q_ik_result = ik_results.q_sol.back();
    }
  }
  return q_ik_result;
}

VectorX<double> RandomClutterGenerator::DropObjectsToGround(
    const VectorX<double>& q_ik, double max_settling_time) {
  Simulator<double> simulator(*fall_sim_diagram_);
  int num_positions = scene_tree_ptr_->get_num_positions();
  int num_velocities = scene_tree_ptr_->get_num_velocities();

  // Setting initial condition
  VectorX<double> x_initial =
      VectorX<double>::Zero(num_positions + num_velocities);

  x_initial.head(scene_tree_ptr_->get_num_positions()) = q_ik;

  simulator.get_mutable_context()
      .get_mutable_continuous_state_vector()
      .SetFromVector(x_initial);
  simulator.Initialize();

  simulator.reset_integrator<RungeKutta2Integrator<double>>(
      *fall_sim_diagram_, 0.0001, &simulator.get_mutable_context());
  simulator.get_mutable_integrator()->set_maximum_step_size(0.001);
  simulator.get_mutable_integrator()->set_fixed_step_mode(true);

  VectorX<double> v = VectorX<double>::Zero(num_velocities);

  double step_time = 0.5, step_delta = 0.1;
  double v_threshold = 1e-1;
  VectorX<double> x = x_initial;

  do {
    drake::log()->debug("Starting Simulation");

    simulator.StepTo(step_time);
    step_time += step_delta;
    x = simulator.get_context().get_continuous_state_vector().CopyToVector();
    v = x.tail(num_velocities);
  } while ((v.array() > v_threshold).any() && step_time <= max_settling_time);

  drake::log()->info("In-Simulation time : {} sec", step_time);
  return x.head(num_positions);
}

std::unique_ptr<systems::Diagram<double>>
RandomClutterGenerator::GenerateFallSimDiagram(
    std::unique_ptr<RigidBodyTreed> scene_tree,
    const VisualizeSimulationCallback& visualize) {
  DiagramBuilder<double> builder;

  // Transferring ownership of tree to the RigidBodyPlant.
  auto plant = builder.template AddSystem<systems::RigidBodyPlant<double>>(
      std::move(scene_tree));
  plant->set_name("RBP");

  systems::CompliantMaterial default_material;
  default_material
      .set_youngs_modulus(1e6)  // Pa
      .set_dissipation(9)       // s/m
      .set_friction(1.2, 0.5);
  plant->set_default_compliant_material(default_material);

  systems::CompliantContactModelParameters model_parameters;
  model_parameters.characteristic_radius = 2e-4;  // m
  model_parameters.v_stiction_tolerance = 0.1;    // m/s
  plant->set_contact_model_parameters(model_parameters);

  if (visualize) {
    visualize(&builder, plant);
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