# pragma once
#include<memory>

#include "drake/multibody/rigid_body_plant/contact_results.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace tools {

template <typename T>
class GeometricContactStateEstimator {
 public:
  GeometricContactStateEstimator(
      std::unique_ptr<RigidBodyTree<T>> tree);

  systems::ContactResults<T> ComputeContactResults(const VectorX<T>& x )

 private:
  //systems::ContactResults<T>* contact_results_;
  std::unique_ptr<RigidBodyTree<T>> rigid_body_tree_;
};

} // namespace tools
} // namespace kuka_iiwa_arm
} // namespace examples
} // namespace drake