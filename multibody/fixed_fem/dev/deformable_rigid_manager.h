#pragma once

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/contact_solver.h"
#include "drake/multibody/fixed_fem/dev/deformable_model.h"
#include "drake/multibody/fixed_fem/dev/fem_solver.h"
#include "drake/multibody/plant/contact_jacobians.h"
#include "drake/multibody/plant/discrete_update_manager.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** %DeformableRigidManager implements the interface in DiscreteUpdateManager
 and performs discrete update for deformable and rigid bodies with a two-way
 coupling scheme.
 @tparam_nonsymbolic_scalar. */
template <typename T>
class DeformableRigidManager final
    : public multibody::internal::DiscreteUpdateManager<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DeformableRigidManager)

  DeformableRigidManager() = default;

  /** Sets the given `contact_solver` as the solver that `this`
    %DeformableRigidManager uses to solve contact. */
  void set_contact_solver(
      std::unique_ptr<multibody::contact_solvers::internal::ContactSolver<T>>
          contact_solver) {
    contact_solver_ = std::move(contact_solver);
  }

 private:
  /* Implements DiscreteUpdateManager::ExtractModelInfo(). Verifies that
   exactly one DeformableModel is registered in the owning plant and
   sets up FEM solvers for deformable bodies. */
  void ExtractModelInfo() final;

  /* Make the FEM solvers that solve the deformable FEM models. */
  void MakeFemSolvers();

  void DeclareCacheEntries(MultibodyPlant<T>* plant) final;

  void DoCalcContactSolverResults(
      const systems::Context<T>& context,
      contact_solvers::internal::ContactSolverResults<T>* results) const final;

  /* Calculates all contact quantities needed by the contact solver and the
   TAMSI solver from the given `context` and `rigid_contact_pairs`.
   @pre All pointer parameters are non-null.
   @pre The size of `v` and `minus_tau` are equal to the number of rigid
        generalized velocities.
   @pre `M` is square and has rows and columns equal to the number of rigid
        generalized velocities.
   @pre `mu`, `phi`, `fn`, `stiffness`, `damping`, and `rigid_contact_pairs`
        have the same size. */
  void CalcContactQuantities(
      const systems::Context<T>& context,
      const std::vector<multibody::internal::DiscreteContactPair<T>>&
          rigid_contact_pairs,
      multibody::internal::ContactJacobians<T>* rigid_contact_jacobians,
      EigenPtr<VectorX<T>> v, EigenPtr<MatrixX<T>> M,
      EigenPtr<VectorX<T>> minus_tau, EigenPtr<VectorX<T>> mu,
      EigenPtr<VectorX<T>> phi, EigenPtr<VectorX<T>> fn,
      EigenPtr<VectorX<T>> stiffness, EigenPtr<VectorX<T>> damping) const;

  // TODO(xuchenhan-tri): Implement this once AccelerationKinematicsCache
  //  also caches acceleration for deformable dofs.
  void DoCalcAccelerationKinematicsCache(
      const systems::Context<T>&,
      multibody::internal::AccelerationKinematicsCache<T>*) const final {
    throw std::logic_error(
        "DoCalcAcclerationKinematicsCache() hasn't be implemented for "
        "DeformableRigidManager yet.");
  }

  void DoCalcDiscreteValues(const systems::Context<T>& context,
                            systems::DiscreteValues<T>* updates) const final;

  /* Evaluates the FEM state of the deformable body with index `id`. */
  const FemStateBase<T>& EvalFemStateBase(const systems::Context<T>& context,
                                          SoftBodyIndex id) const {
    return this->plant()
        .get_cache_entry(fem_state_cache_indexes_[id])
        .template Eval<FemStateBase<T>>(context);
  }

  /* Evaluates the free motion FEM state of the deformable body with index `id`.
   */
  const FemStateBase<T>& EvalFreeMotionFemStateBase(
      const systems::Context<T>& context, SoftBodyIndex id) const {
    return this->plant()
        .get_cache_entry(free_motion_cache_indexes_[id])
        .template Eval<FemStateBase<T>>(context);
  }

  /* The deformable models being solved by `this` manager. */
  const DeformableModel<T>* deformable_model_{nullptr};
  /* Cached FEM state quantities. */
  std::vector<systems::CacheIndex> fem_state_cache_indexes_;
  std::vector<systems::CacheIndex> free_motion_cache_indexes_;
  /* Solvers for all deformable bodies. */
  std::vector<std::unique_ptr<FemSolver<T>>> fem_solvers_{};
  std::unique_ptr<multibody::contact_solvers::internal::ContactSolver<T>>
      contact_solver_{nullptr};
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fixed_fem::DeformableRigidManager)
