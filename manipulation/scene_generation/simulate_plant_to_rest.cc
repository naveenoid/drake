#include "drake/manipulation/scene_generation/simulate_plant_to_rest.h"

#include "drake/common/eigen_types.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"

namespace drake {
namespace manipulation {
namespace scene_generation {


std::unique_ptr<systems::LeafSystem<double>> DrakeVisualizerCallback(lcm::DrakeLcm* lcm, systems::RigidBodyPlant<double>* plant) {
  return std::make_unique<systems::DrakeVisualizer>(plant->get_rigid_body_tree(), lcm);
}

SimulatePlantToRest::SimulatePlantToRest(
	std::unique_ptr<systems::RigidBodyPlant<double>> scene_plant, const VisualizerSystemCallback& visualizer) :
	 plant_ptr_(scene_plant.get()), diagram_(GenerateDiagram(std::move(scene_plant), visualizer)) {
	 }

std::unique_ptr<systems::Diagram<double>> SimulatePlantToRest::GenerateDiagram(
	std::move(scene_plant), visualizer) {
  systems::DiagramBuilder<double> builder;

  // Transferring ownership of tree to the RigidBodyPlant.
  auto plant = builder.template AddSystem(
      std::move(scene_plant));
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
   	auto visualizer_system = builder.template AddSystem(
   		std::move(visualizer(plant)));
	visualizer_system->set_name("visualizer");

  builder.Connect(plant->get_output_port(0),
                   visualizer_system->get_input_port(0));

  }

  if (plant->get_num_actuators() > 0) {
    auto zero_input = builder.template AddSystem<systems::ConstantVectorSource>(
        Eigen::VectorXd::Zero(plant->get_num_actuators()));
    builder.Connect(zero_input->get_output_port(), plant->get_input_port(0));
  }

  return builder.Build();
}


VectorX<double> SimulatePlantToRest::Run(const VectorX<double>& q_ik, 
	double max_settling_time) {
	systems::Simulator<double> simulator(*diagram_);

  int num_positions = plant_ptr_->get_num_positions();
  int num_velocities = plant_ptr_->get_num_velocities();

  // Setting initial condition
  VectorX<double> x_initial =
      VectorX<double>::Zero(num_positions + num_velocities);

  x_initial.head(num_positions) = q_ik;

  simulator.get_mutable_context()
      .get_mutable_continuous_state_vector()
      .SetFromVector(x_initial);
  simulator.Initialize();

  simulator.reset_integrator<systems::RungeKutta2Integrator<double>>(
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

}  // namespace scene_generation
}  // namespace manipulation
}  // namespace drake