///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2022, LAAS-CNRS, University of Edinburgh,
//                          Heriot-Watt University
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MULTIBODY_ACTIONS_IMPEDANCE_FWDDYN_HPP_
#define CROCODDYL_MULTIBODY_ACTIONS_IMPEDANCE_FWDDYN_HPP_

#include <stdexcept>

#ifdef PINOCCHIO_WITH_CPPAD_SUPPORT  // TODO(cmastalli): Removed after merging
                                     // Pinocchio v.2.4.8
#include <pinocchio/codegen/cppadcg.hpp>
#endif

#include "crocoddyl/core/actuation-base.hpp"
#include "crocoddyl/core/constraints/constraint-manager.hpp"
#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/core/diff-action-base.hpp"
#include "crocoddyl/core/utils/exception.hpp"
#include "crocoddyl/multibody/data/multibody.hpp"
#include "crocoddyl/multibody/fwd.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"

namespace crocoddyl {

/**
 * @brief Differential action model for free forward dynamics in multibody
 * systems.
 *
 * This class implements free forward dynamics, i.e.,
 * \f[
 * \mathbf{M}\dot{\mathbf{v}} + \mathbf{h}(\mathbf{q},\mathbf{v}) =
 * \boldsymbol{\tau}, \f] where \f$\mathbf{q}\in Q\f$,
 * \f$\mathbf{v}\in\mathbb{R}^{nv}\f$ are the configuration point and
 * generalized velocity (its tangent vector), respectively;
 * \f$\boldsymbol{\tau}\f$ is the torque inputs and
 * \f$\mathbf{h}(\mathbf{q},\mathbf{v})\f$ are the Coriolis effect and gravity
 * field.
 *
 * The derivatives of the system acceleration is computed efficiently based on
 * the analytical derivatives of Articulate Body Algorithm (ABA) as described in
 * \cite carpentier-rss18.
 *
 * The stack of cost functions is implemented in `CostModelSumTpl`. The
 * computation of the free forward dynamics and its derivatives are carrying out
 * inside `calc()` and `calcDiff()` functions, respectively. It is also
 * important to remark that `calcDiff()` computes the derivatives using the
 * latest stored values by `calc()`. Thus, we need to run `calc()` first.
 *
 * \sa `DifferentialActionModelAbstractTpl`, `calc()`, `calcDiff()`,
 * `createData()`
 */
template <typename _Scalar>
class DifferentialActionModelImpedanceFwdDynamicsTpl
    : public DifferentialActionModelAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef DifferentialActionModelAbstractTpl<Scalar> Base;
  typedef DifferentialActionDataImpedanceFwdDynamicsTpl<Scalar> Data;
  typedef DifferentialActionDataAbstractTpl<Scalar>
      DifferentialActionDataAbstract;
  typedef StateMultibodyTpl<Scalar> StateMultibody;
  typedef CostModelSumTpl<Scalar> CostModelSum;
  typedef ConstraintModelManagerTpl<Scalar> ConstraintModelManager;
  typedef ActuationModelAbstractTpl<Scalar> ActuationModelAbstract;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;

  // ---- Impedance ---
  typedef typename MathBase::Vector6s Vector6s;
  typedef typename MathBase::Matrix6s Matrix6s;

  DifferentialActionModelImpedanceFwdDynamicsTpl(
      const pinocchio::SE3Tpl<Scalar>& pref,
      boost::shared_ptr<StateMultibody> state,
      boost::shared_ptr<ActuationModelAbstract> actuation,
      boost::shared_ptr<CostModelSum> costs,
      boost::shared_ptr<ConstraintModelManager> constraints = nullptr);
  virtual ~DifferentialActionModelImpedanceFwdDynamicsTpl();

  /**
   * @brief Compute the system acceleration, and cost value
   *
   * It computes the system acceleration using the free forward-dynamics.
   *
   * @param[in] data  Free forward-dynamics data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calc(
      const boost::shared_ptr<DifferentialActionDataAbstract>& data,
      const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>& u);

  /**
   * @brief @copydoc Base::calc(const
   * boost::shared_ptr<DifferentialActionDataAbstract>& data, const
   * Eigen::Ref<const VectorXs>& x)
   */
  virtual void calc(
      const boost::shared_ptr<DifferentialActionDataAbstract>& data,
      const Eigen::Ref<const VectorXs>& x);

  /**
   * @brief Compute the derivatives of the contact dynamics, and cost function
   *
   * @param[in] data  Free forward-dynamics data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calcDiff(
      const boost::shared_ptr<DifferentialActionDataAbstract>& data,
      const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>& u);

  /**
   * @brief @copydoc Base::calcDiff(const
   * boost::shared_ptr<DifferentialActionDataAbstract>& data, const
   * Eigen::Ref<const VectorXs>& x)
   */
  virtual void calcDiff(
      const boost::shared_ptr<DifferentialActionDataAbstract>& data,
      const Eigen::Ref<const VectorXs>& x);

  /**
   * @brief Create the free forward-dynamics data
   *
   * @return free forward-dynamics data
   */
  virtual boost::shared_ptr<DifferentialActionDataAbstract> createData();

  /**
   * @brief Check that the given data belongs to the free forward-dynamics data
   */
  virtual bool checkData(
      const boost::shared_ptr<DifferentialActionDataAbstract>& data);

  /**
   * @brief @copydoc Base::quasiStatic()
   */
  virtual void quasiStatic(
      const boost::shared_ptr<DifferentialActionDataAbstract>& data,
      Eigen::Ref<VectorXs> u, const Eigen::Ref<const VectorXs>& x,
      const std::size_t maxiter = 100, const Scalar tol = Scalar(1e-9));

  /**
   * @brief Return the number of inequality constraints
   */
  virtual std::size_t get_ng() const;

  /**
   * @brief Return the number of equality constraints
   */
  virtual std::size_t get_nh() const;

  /**
   * @brief Return the lower bound of the inequality constraints
   */
  virtual const VectorXs& get_g_lb() const;

  /**
   * @brief Return the upper bound of the inequality constraints
   */
  virtual const VectorXs& get_g_ub() const;

  /**
   * @brief Return the actuation model
   */
  const boost::shared_ptr<ActuationModelAbstract>& get_actuation() const;

  /**
   * @brief Return the cost model
   */
  const boost::shared_ptr<CostModelSum>& get_costs() const;

  /**
   * @brief Return the constraint model manager
   */
  const boost::shared_ptr<ConstraintModelManager>& get_constraints() const;

  /**
   * @brief Return the Pinocchio model
   */
  pinocchio::ModelTpl<Scalar>& get_pinocchio() const;

  /**
   * @brief Return the armature vector
   */
  const VectorXs& get_armature() const;

  /**
   * @brief Set the impedance control (reference = contact point)
   */
  const pinocchio::SE3Tpl<Scalar>& get_reference() const;
  const Matrix6s& get_stiffness() const;
  const Matrix6s& get_damping() const;

  void set_reference(const pinocchio::SE3Tpl<Scalar>& pref);
  void set_stiffness(const Matrix6s& K);
  void set_damping(const Matrix6s& D);

  /**
   * @brief Modify the armature vector
   */
  void set_armature(const VectorXs& armature);

  /**
   * @brief Print relevant information of the free forward-dynamics model
   *
   * @param[out] os  Output stream object
   */
  virtual void print(std::ostream& os) const;

 protected:
  using Base::g_lb_;   //!< Lower bound of the inequality constraints
  using Base::g_ub_;   //!< Upper bound of the inequality constraints
  using Base::nu_;     //!< Control dimension
  using Base::state_;  //!< Model of the state

 private:
  boost::shared_ptr<ActuationModelAbstract> actuation_;    //!< Actuation model
  boost::shared_ptr<CostModelSum> costs_;                  //!< Cost model
  boost::shared_ptr<ConstraintModelManager> constraints_;  //!< Constraint model
  pinocchio::ModelTpl<Scalar>& pinocchio_;                 //!< Pinocchio model
  bool without_armature_;  //!< Indicate if we have defined an armature
  VectorXs armature_;      //!< Armature vector

  // ---- Impedance ---
  pinocchio::SE3Tpl<Scalar> pref_;      //!< Reference joint placement
  Matrix6s Kmat_, Dmat_;                //!< Stiffness, damping
};

template <typename _Scalar>
struct DifferentialActionDataImpedanceFwdDynamicsTpl
    : public DifferentialActionDataAbstractTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef DifferentialActionDataAbstractTpl<Scalar> Base;
  typedef JointDataAbstractTpl<Scalar> JointDataAbstract;
  typedef DataCollectorJointActMultibodyTpl<Scalar>
      DataCollectorJointActMultibody;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;

  // ---- Impedance ---
  typedef typename MathBase::Vector6s Vector6s;
  typedef typename MathBase::Matrix6s Matrix6s;
  typedef typename MathBase::Matrix6xs Matrix6xs;

  template <template <typename Scalar> class Model>
  explicit DifferentialActionDataImpedanceFwdDynamicsTpl(Model<Scalar>* const model)
      : Base(model),
        pinocchio(pinocchio::DataTpl<Scalar>(model->get_pinocchio())),
        multibody(
            &pinocchio, model->get_actuation()->createData(),
            boost::make_shared<JointDataAbstract>(
                model->get_state(), model->get_actuation(), model->get_nu())),
        costs(model->get_costs()->createData(&multibody)),
        Minv(model->get_state()->get_nv(), model->get_state()->get_nv()),
        u_drift(model->get_state()->get_nv()),
        dtau_dx(model->get_state()->get_nv(), model->get_state()->get_ndx()),
        tmp_xstatic(model->get_state()->get_nx()),
        J(6, model->get_state()->get_nv()) {      // ---- Impedance ---
    multibody.joint->dtau_du.diagonal().setOnes();
    costs->shareMemory(this);
    if (model->get_constraints() != nullptr) {
      constraints = model->get_constraints()->createData(&multibody);
      constraints->shareMemory(this);
    }
    Minv.setZero();
    u_drift.setZero();
    dtau_dx.setZero();
    tmp_xstatic.setZero();

    // ---- Impedance ---
    dXerr.setZero();
    F.setZero();
    J.setZero();
    fext.resize(model->get_pinocchio().joints.size(), pinocchio::ForceTpl<Scalar>::Zero());
  }

  pinocchio::DataTpl<Scalar> pinocchio;
  DataCollectorJointActMultibody multibody;
  boost::shared_ptr<CostDataSumTpl<Scalar> > costs;
  boost::shared_ptr<ConstraintDataManagerTpl<Scalar> > constraints;
  MatrixXs Minv;
  VectorXs u_drift;
  MatrixXs dtau_dx;
  VectorXs tmp_xstatic;

  // ---- Impedance ---
  pinocchio::SE3Tpl<Scalar> Xerr;   //!< Error joint placement of the joint
  Matrix6s dXerr;                   //!< Right derivative of the error joint placement
  Vector6s F;                       //!< External wrench of the joint
  Matrix6xs J;                      //!< Local Jacobian of the joint
  PINOCCHIO_ALIGNED_STD_VECTOR(pinocchio::ForceTpl<Scalar>) fext;
  
  using Base::cost;
  using Base::Fu;
  using Base::Fx;
  using Base::Lu;
  using Base::Luu;
  using Base::Lx;
  using Base::Lxu;
  using Base::Lxx;
  using Base::r;
  using Base::xout;
};

}  // namespace crocoddyl

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include <crocoddyl/multibody/actions/impedance-fwddyn.hxx>

#endif  // CROCODDYL_MULTIBODY_ACTIONS_IMPEDANCE_FWDDYN_HPP_
