#pragma once

#include <random>
#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace manipulation {
namespace scene_generation {

/**
 * Visualization for debugging purposes.
 */
typedef std::function<void(systems::DiagramBuilder<double>* diagram,
                           systems::RigidBodyPlant<double>* plant)>
    VisualizeSimulationCallback;

/**
 * Visualizes a simulation of a RigidBodyPlant containing a given RigidBodyTree.
 */
void VisualizeSimulationLcm(lcm::DrakeLcm* lcm,
                            systems::DiagramBuilder<double>* diagram,
                            systems::RigidBodyPlant<double>* plant);

/**
 * Binds an LCM object to `VisualizeSimulationLcm`.
 */
inline VisualizeSimulationCallback BindVisualizeSimulationLcm(
    lcm::DrakeLcm* lcm) {
  // NOLINTNEXTLINE(build/namespaces): For `bind`.
  return std::bind(VisualizeSimulationLcm, lcm, std::placeholders::_1,
                   std::placeholders::_2);
}

/**
 * Given a RigidBodyTree containing a given scene the RandomClutterGenerator
 * can repeatedly generate bounded random poses/configurations on selected
 * model instances within the tree.
 * Each of these objects are seperated from each other by (settable) minimum
 * distance and their object frames are located within a (settable) bounding
 * box volume.
 * This class provides 2 methods that (i) solves the IK problem to find feasible
 * poses on the clutter bodies and (ii) executes a short-duration simulation
 * to drop and settle all of the floating bodies onto the scene.
 *
 * NOTES :
 * 1. Current version does not 'strongly' enforce the poses of the non-clutter
 * elements, i.e. the state of model instances that do not comprise the
 * clutter bodies and that are not fixed to the world may be perturbed during
 * the simulation phase.
 * 2. Current version only ensures bounded clutter for the case of all model
 * instances on the tree containing the QuaternionFloatingJoint.
 * 3. The current version has only been tested with SNOPT.
 */

class RandomClutterGenerator {
 public:
  /**
   * Constructs the RandomClutterGenerator
   * @param scene_tree A unique_ptr to the tree containing the scene.
   * @param clutter_model_instances A set of model instance indices
   * corresponding to the bodies on the tree that should comprise the clutter
   * @param clutter_centroid Centroid of the clutter bounding box in
   * world coordinates.
   * @param clutter_size The size of the clutter bounding box along the
   * length, breadth, and height.
   * @param visualize The callback to a visualization object (for the
   * dropping object simulation phase).
   * @param min_inter_object_distance Minimum distance between objects.
   */
  RandomClutterGenerator(std::unique_ptr<RigidBodyTreed> scene_tree,
                         std::vector<int> clutter_model_instances,
                         Vector3<double> clutter_centroid,
                         Vector3<double> clutter_size,
                         const VisualizeSimulationCallback& visualize = {},
                         double min_inter_object_distance = 0.001);

  /**
   * Generates the "Floating" clutter scene by solving an IK problem.
   * @return a ModelPosePair of the object model names and their poses.
   * @param q_nominal : nominal configuration for the scene_tree. Poses of
   * the model_instances not specified in `clutter_model_instances' are set
   * to this value.
   * @param generator : used to pass a seed.
   */
  VectorX<double> GenerateFloatingClutter(
      VectorX<double> q_nominal, std::default_random_engine& generator);

  /**
   * Simulates a drop of the objects to the ground.
   * @param max_settling_time is the max time to wait for settling the
   * clutter scene.
   */
  VectorX<double> DropObjectsToGround(const VectorX<double>& q_ik,
                                      double max_settling_time = 2.5);

  /**
   * Returns a pointer to the Sim diagram.
   */
  systems::Diagram<double>* GetSimDiagram();

 private:
  // Builds a diagram of the clutter scene.
  std::unique_ptr<systems::Diagram<double>> GenerateFallSimDiagram(
      std::unique_ptr<RigidBodyTreed> scene_tree,
      const VisualizeSimulationCallback& visualize = {});

  RigidBodyTreed* scene_tree_ptr_{nullptr};
  std::vector<int> clutter_model_instances_;

  Vector3<double> clutter_center_{Vector3<double>::Zero()};
  Vector3<double> clutter_lb_{Vector3<double>::Zero()};
  Vector3<double> clutter_ub_{Vector3<double>::Zero()};

  double inter_object_distance_{0.1};

  std::unique_ptr<systems::Diagram<double>> fall_sim_diagram_{nullptr};

  // std::map<std::string, normal_distribution<double>> distribution_map_;
};

}  // namespace scene_generation
}  // namespace manipulation
}  // namespace drake