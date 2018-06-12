#pragma once

#include "drake/common/eigen_types.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"

namespace drake {
namespace manipulation {
namespace scene_generation {

/**
 * Given a RigidBodyPlant, this class allows the construction and excution of a 
 * Simulation which enables the state of the plant to come to a rest from a 
 * specified initial condition through the application of 0 magnitude of torques
 * at the input. .
 */

class SimulatePlantToRest {
 public:
  /**
   * Constructs the SimulatePlantToRest
   * @param scene_plant A unique_ptr to a RigidBodyPlant of the scene.
   * @param visualize A unique_ptr to a DrakeVisualizer that can be 
   * specified to enable rendering the simulation.
   */
  SimulatePlantToRest(
      std::unique_ptr<systems::RigidBodyPlant<double>> scene_plant,
      std::unique_ptr<systems::DrakeVisualizer> visualizer = {});

  /**
   * Simulates a drop of the objects to the ground.
   * @param max_settling_time is the max time to wait for settling the
   * clutter scene.
   */
  VectorX<double> Run(const VectorX<double>& q_ik,
                      double max_settling_time = 1.5);

  /**
   * Returns a pointer to the Sim diagram.
   */
  systems::Diagram<double>* GetSimDiagram();

 private:
  // Builds a diagram of the clutter scene.
  std::unique_ptr<systems::Diagram<double>> GenerateDiagram(
      std::unique_ptr<systems::RigidBodyPlant<double>> scene_plant,
      std::unique_ptr<systems::DrakeVisualizer> = {});

  systems::RigidBodyPlant<double>* plant_ptr_{nullptr};
  std::unique_ptr<systems::Diagram<double>> diagram_;
};

}  // namespace scene_generation
}  // namespace manipulation
}  // namespace drake