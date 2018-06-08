#pragma once

#include <random>
#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/multibody/rigid_body_constraint.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace manipulation {
namespace scene_generation {

/**
 * Given a RigidBodyTree containing a given scene the RandomClutterGenerator
 * can repeatedly generate bounded random poses/configurations on selected
 * model instances within the tree.
 * Each of these objects are seperated from each other by (settable) minimum
 * distance and their object frames are located within a (settable) bounding
 * box volume.
 * This tool solves an IK problem to find feasible poses on the clutter
 * bodies and then executes a short-duration simulation to settle all of the
 * elements.
 * NOTES :
 * 1. Current version does not 'strongly' enforce the poses of the non-clutter
 * elements, i.e. the state of model instances that do not comprise the
 * clutter bodies and that are not fixed to the world may be perturbed during
 * the simulation phase.
 * 2. Current version assumes that all model instances on the tree contain only
 * the following joints : RevoluteJoint, and QuaternionFloatingJoint.
 */

class RandomClutterGenerator {
 public:
  /**
   * Constructs the RandomClutterGenerator
   * @param scene_tree A unique_ptr to the tree containing the scene.
   * @param clutter_model_instances A set of model instance indices
   * corresponding to the bodies on the tree that should comprise the clutter
   * @param bounding_box_centroid Centroid of the clutter bounding box in
   * world coordinates.
   * @param bounding_box_size The size of the clutter bounding box along the
   * length, breadth, and height.
   * @param min_inter_object_distance Minimum distance between objects.
   */
  RandomClutterGenerator(std::unique_ptr<RigidBodyTreed> scene_tree,
                         std::vector<int> clutter_model_instances,
                         Vector3<double> bounding_box_centroid,
                         Vector3<double> bounding_box_size,
                         bool visualize_steps,
                         double min_inter_object_distance = 0.001);

  /**
   * Generates the cluttered scene.
   * @return a ModelPosePair of the object model names and their poses.
   */
  VectorX<double> Generate(VectorX<double> q_nominal,
                           std::default_random_engine& generator);

 private:
  // // Generates a random pose within the bounds of the problem.
  // Isometry3<double> GenerateRandomBoundedPose();

  // // Generates a random bounded tree configuration.
  // VectorX<double> GetRandomBoundedConfiguration(
  //   const RigidBodyTreed* scene_tree,
  //   std::vector<int> clutter_model_instances,
  //   VectorX<double> q_initial);

  // Builds a diagram of the clutter scene.
  std::unique_ptr<systems::Diagram<double>> GenerateFallSimDiagram(
      std::unique_ptr<RigidBodyTreed> scene_tree);

  // Simulates a drop of the objects to the ground.
  VectorX<double> DropObjectsToGround(const VectorX<double>& q_ik);

  RigidBodyTreed* scene_tree_ptr_{nullptr};
  std::vector<int> clutter_model_instances_;

  Vector3<double> clutter_center_{Vector3<double>::Zero()};
  Vector3<double> clutter_lb_{Vector3<double>::Zero()};
  Vector3<double> clutter_ub_{Vector3<double>::Zero()};

  bool visualize_steps_{false};

  double inter_object_distance_{0.1};

  lcm::DrakeLcm lcm_;

  std::unique_ptr<systems::Diagram<double>> fall_sim_diagram_{nullptr};

  // std::map<std::string, normal_distribution<double>> distribution_map_;
};

}  // namespace scene_generation
}  // namespace manipulation
}  // namespace drake