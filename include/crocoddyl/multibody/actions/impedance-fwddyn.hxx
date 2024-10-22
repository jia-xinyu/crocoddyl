///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2022, LAAS-CNRS, University of Edinburgh,
//                          Heriot-Watt University
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <pinocchio/algorithm/aba-derivatives.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/cholesky.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include "crocoddyl/core/utils/exception.hpp"
#include "crocoddyl/multibody/actions/impedance-fwddyn.hpp"

namespace crocoddyl {

template <typename Scalar>
DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::
    DifferentialActionModelImpedanceFwdDynamicsTpl(
        const pinocchio::SE3Tpl<Scalar>& pref,
        boost::shared_ptr<StateMultibody> state,
        boost::shared_ptr<ActuationModelAbstract> actuation,
        boost::shared_ptr<CostModelSum> costs,
        boost::shared_ptr<ConstraintModelManager> constraints)
    : Base(state, actuation->get_nu(), costs->get_nr()),
      actuation_(actuation),
      costs_(costs),
      constraints_(constraints),
      pinocchio_(*state->get_pinocchio().get()),
      without_armature_(true),
      armature_(VectorXs::Zero(state->get_nv())),
      pref_(pref),   // ---- Impedance ---
      Kmat_(Matrix6s::Zero()),
      Dmat_(Matrix6s::Zero()) {
  if (costs_->get_nu() != nu_) {
    throw_pretty(
        "Invalid argument: "
        << "Costs doesn't have the same control dimension (it should be " +
               std::to_string(nu_) + ")");
  }
  Base::set_u_lb(Scalar(-1.) * pinocchio_.effortLimit.tail(nu_));
  Base::set_u_ub(Scalar(+1.) * pinocchio_.effortLimit.tail(nu_));
}

template <typename Scalar>
DifferentialActionModelImpedanceFwdDynamicsTpl<
    Scalar>::~DifferentialActionModelImpedanceFwdDynamicsTpl() {}

template <typename Scalar>
void DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::calc(
    const boost::shared_ptr<DifferentialActionDataAbstract>& data,
    const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>& u) {
  if (static_cast<std::size_t>(x.size()) != state_->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " +
                        std::to_string(state_->get_nx()) + ")");
  }
  if (static_cast<std::size_t>(u.size()) != nu_) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " +
                        std::to_string(nu_) + ")");
  }

  Data* d = static_cast<Data*>(data.get());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q =
      x.head(state_->get_nq());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v =
      x.tail(state_->get_nv());

  actuation_->calc(d->multibody.actuation, x, u);

  // ---- Impedance --- See "ResidualModelImpedanceForceTpl"
  /*!
   * Xerr (= p-pref) = Tref㊀T = Tref^-1 * T, w.r.t. the reference joint
   * F = K*(p-pref) + D*V, of direction is (-)
   */
  const std::size_t id_ = state_->get_nq();   // the last joint
  pinocchio::forwardKinematics(pinocchio_, d->pinocchio, q);
  d->Xerr = pref_.inverse() * d->pinocchio.oMi[id_];
  pinocchio::computeJointJacobian(pinocchio_, d->pinocchio, q, 
                                  id_, d->J); // joint Jacobian (not frame)
  d->F = Kmat_ * pinocchio::log6(d->Xerr).toVector() + Dmat_ * d->J * v;
  d->fext[id_] = d->F;
  
  // Computing the dynamics using ABA or manually for armature case
  if (without_armature_) {
    d->xout = pinocchio::aba(pinocchio_, d->pinocchio, q, v,
                             d->multibody.actuation->tau, d->fext);
    pinocchio::updateGlobalPlacements(pinocchio_, d->pinocchio);
  } else {
    pinocchio::computeAllTerms(pinocchio_, d->pinocchio, q, v);
    d->pinocchio.M.diagonal() += armature_;
    pinocchio::cholesky::decompose(pinocchio_, d->pinocchio);
    d->Minv.setZero();
    pinocchio::cholesky::computeMinv(pinocchio_, d->pinocchio, d->Minv);
    d->u_drift = d->multibody.actuation->tau - d->pinocchio.nle;
    d->xout.noalias() = d->Minv * d->u_drift;
  }
  d->multibody.joint->a = d->xout;
  d->multibody.joint->tau = u;
  costs_->calc(d->costs, x, u);
  d->cost = d->costs->cost;
  if (constraints_ != nullptr) {
    d->constraints->resize(this, d);
    constraints_->calc(d->constraints, x, u);
  }
}

template <typename Scalar>
void DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::calc(
    const boost::shared_ptr<DifferentialActionDataAbstract>& data,
    const Eigen::Ref<const VectorXs>& x) {
  if (static_cast<std::size_t>(x.size()) != state_->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " +
                        std::to_string(state_->get_nx()) + ")");
  }

  Data* d = static_cast<Data*>(data.get());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q =
      x.head(state_->get_nq());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v =
      x.tail(state_->get_nv());

  pinocchio::computeAllTerms(pinocchio_, d->pinocchio, q, v);

  costs_->calc(d->costs, x);
  d->cost = d->costs->cost;
  if (constraints_ != nullptr) {
    d->constraints->resize(this, d);
    constraints_->calc(d->constraints, x);
  }
}

template <typename Scalar>
void DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::calcDiff(
    const boost::shared_ptr<DifferentialActionDataAbstract>& data,
    const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>& u) {
  if (static_cast<std::size_t>(x.size()) != state_->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " +
                        std::to_string(state_->get_nx()) + ")");
  }
  if (static_cast<std::size_t>(u.size()) != nu_) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " +
                        std::to_string(nu_) + ")");
  }

  const std::size_t nv = state_->get_nv();
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q =
      x.head(state_->get_nq());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v =
      x.tail(nv);

  Data* d = static_cast<Data*>(data.get());

  actuation_->calcDiff(d->multibody.actuation, x, u);

  // Computing the dynamics derivatives
  if (without_armature_) {
    pinocchio::computeABADerivatives(
        pinocchio_, d->pinocchio, q, v, d->multibody.actuation->tau, d->fext,
        d->Fx.leftCols(nv), d->Fx.rightCols(nv), d->pinocchio.Minv);
    // d->Fx.noalias() += d->pinocchio.Minv * d->multibody.actuation->dtau_dx;
    // d->Fu.noalias() = d->pinocchio.Minv * d->multibody.actuation->dtau_du;

    // ---- Impedance ---
    pinocchio::Jlog6(d->Xerr, d->dXerr);  // right derivative of log6
    d->dtau_dx.leftCols(nv) =
        d->multibody.actuation->dtau_dx.leftCols(nv) + d->J.transpose() * Kmat_ * d->dXerr* d->J;
    d->dtau_dx.rightCols(nv) =
        d->multibody.actuation->dtau_dx.rightCols(nv) + d->J.transpose() * Dmat_ * d->J;
    d->Fx.noalias() += d->pinocchio.Minv * d->dtau_dx;
    d->Fu.noalias() = d->pinocchio.Minv * d->multibody.actuation->dtau_du;
  } else {
    pinocchio::computeRNEADerivatives(pinocchio_, d->pinocchio, q, v, d->xout);
    d->dtau_dx.leftCols(nv) =
        d->multibody.actuation->dtau_dx.leftCols(nv) - d->pinocchio.dtau_dq;
    d->dtau_dx.rightCols(nv) =
        d->multibody.actuation->dtau_dx.rightCols(nv) - d->pinocchio.dtau_dv;
    d->Fx.noalias() = d->Minv * d->dtau_dx;
    d->Fu.noalias() = d->Minv * d->multibody.actuation->dtau_du;
  }
  d->multibody.joint->da_dx = d->Fx;
  d->multibody.joint->da_du = d->Fu;
  costs_->calcDiff(d->costs, x, u);
  if (constraints_ != nullptr) {
    constraints_->calcDiff(d->constraints, x, u);
  }
}

template <typename Scalar>
void DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::calcDiff(
    const boost::shared_ptr<DifferentialActionDataAbstract>& data,
    const Eigen::Ref<const VectorXs>& x) {
  if (static_cast<std::size_t>(x.size()) != state_->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " +
                        std::to_string(state_->get_nx()) + ")");
  }
  Data* d = static_cast<Data*>(data.get());

  costs_->calcDiff(d->costs, x);
  if (constraints_ != nullptr) {
    constraints_->calcDiff(d->constraints, x);
  }
}

template <typename Scalar>
boost::shared_ptr<DifferentialActionDataAbstractTpl<Scalar> >
DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::createData() {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this);
}

template <typename Scalar>
bool DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::checkData(
    const boost::shared_ptr<DifferentialActionDataAbstract>& data) {
  boost::shared_ptr<Data> d = boost::dynamic_pointer_cast<Data>(data);
  if (d != NULL) {
    return true;
  } else {
    return false;
  }
}

template <typename Scalar>
void DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::quasiStatic(
    const boost::shared_ptr<DifferentialActionDataAbstract>& data,
    Eigen::Ref<VectorXs> u, const Eigen::Ref<const VectorXs>& x,
    const std::size_t, const Scalar) {
  if (static_cast<std::size_t>(u.size()) != nu_) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " +
                        std::to_string(nu_) + ")");
  }
  if (static_cast<std::size_t>(x.size()) != state_->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " +
                        std::to_string(state_->get_nx()) + ")");
  }
  // Static casting the data
  Data* d = static_cast<Data*>(data.get());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q =
      x.head(state_->get_nq());

  const std::size_t nq = state_->get_nq();
  const std::size_t nv = state_->get_nv();

  d->tmp_xstatic.head(nq) = q;
  d->tmp_xstatic.tail(nv).setZero();
  u.setZero();

  pinocchio::rnea(pinocchio_, d->pinocchio, q, d->tmp_xstatic.tail(nv),
                  d->tmp_xstatic.tail(nv));
  actuation_->calc(d->multibody.actuation, d->tmp_xstatic, u);
  actuation_->calcDiff(d->multibody.actuation, d->tmp_xstatic, u);

  u.noalias() =
      pseudoInverse(d->multibody.actuation->dtau_du) * d->pinocchio.tau;
  d->pinocchio.tau.setZero();
}

template <typename Scalar>
std::size_t DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::get_ng() const {
  if (constraints_ != nullptr) {
    return constraints_->get_ng();
  } else {
    return Base::get_ng();
  }
}

template <typename Scalar>
std::size_t DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::get_nh() const {
  if (constraints_ != nullptr) {
    return constraints_->get_nh();
  } else {
    return Base::get_nh();
  }
}

template <typename Scalar>
const typename MathBaseTpl<Scalar>::VectorXs&
DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::get_g_lb() const {
  if (constraints_ != nullptr) {
    return constraints_->get_lb();
  } else {
    return g_lb_;
  }
}

template <typename Scalar>
const typename MathBaseTpl<Scalar>::VectorXs&
DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::get_g_ub() const {
  if (constraints_ != nullptr) {
    return constraints_->get_ub();
  } else {
    return g_lb_;
  }
}

template <typename Scalar>
void DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::print(
    std::ostream& os) const {
  os << "DifferentialActionModelImpedanceFwdDynamics {nx=" << state_->get_nx()
     << ", ndx=" << state_->get_ndx() << ", nu=" << nu_ << "}";
}

template <typename Scalar>
pinocchio::ModelTpl<Scalar>&
DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::get_pinocchio() const {
  return pinocchio_;
}

template <typename Scalar>
const boost::shared_ptr<ActuationModelAbstractTpl<Scalar> >&
DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::get_actuation() const {
  return actuation_;
}

template <typename Scalar>
const boost::shared_ptr<CostModelSumTpl<Scalar> >&
DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::get_costs() const {
  return costs_;
}

template <typename Scalar>
const boost::shared_ptr<ConstraintModelManagerTpl<Scalar> >&
DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::get_constraints() const {
  return constraints_;
}

template <typename Scalar>
const typename MathBaseTpl<Scalar>::VectorXs&
DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::get_armature() const {
  return armature_;
}

// ---- Impedance ---
template <typename Scalar>
const pinocchio::SE3Tpl<Scalar>&
DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::get_reference() const {
  return pref_;
}
template <typename Scalar>
const typename MathBaseTpl<Scalar>::Matrix6s&
DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::get_stiffness() const {
  return Kmat_;
}
template <typename Scalar>
const typename MathBaseTpl<Scalar>::Matrix6s&
DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::get_damping() const {
  return Dmat_;
}

// ---- Impedance ---
template <typename Scalar>
void DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::set_reference(
  const pinocchio::SE3Tpl<Scalar>& pref) {
  pref_ = pref;
}
template <typename Scalar>
void DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::set_stiffness(
  const Matrix6s& K) {
  Kmat_ = K;
}
template <typename Scalar>
void DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::set_damping(
  const Matrix6s& D) {
  Dmat_ = D;
}

template <typename Scalar>
void DifferentialActionModelImpedanceFwdDynamicsTpl<Scalar>::set_armature(
    const VectorXs& armature) {
  if (static_cast<std::size_t>(armature.size()) != state_->get_nv()) {
    throw_pretty("Invalid argument: "
                 << "The armature dimension is wrong (it should be " +
                        std::to_string(state_->get_nv()) + ")");
  }

  armature_ = armature;
  without_armature_ = false;
}

}  // namespace crocoddyl
