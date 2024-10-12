///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2023, LAAS-CNRS, University of Edinburgh
//                          Heriot-Watt University
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "crocoddyl/multibody/actions/impedance-fwddyn.hpp"

#include "python/crocoddyl/core/diff-action-base.hpp"
#include "python/crocoddyl/multibody/multibody.hpp"
#include "python/crocoddyl/utils/copyable.hpp"

namespace crocoddyl {
namespace python {

void exposeDifferentialActionImpedanceFwdDynamics() {
  bp::register_ptr_to_python<
      boost::shared_ptr<DifferentialActionModelImpedanceFwdDynamics> >();

  bp::class_<DifferentialActionModelImpedanceFwdDynamics,
             bp::bases<DifferentialActionModelAbstract> >(
      "DifferentialActionModelImpedanceFwdDynamics",
      "Differential action model for free forward dynamics in multibody "
      "systems.\n\n"
      "This class implements a the dynamics using Articulate Body Algorithm "
      "(ABA),\n"
      "or a custom implementation in case of system with armatures. If you "
      "want to\n"
      "include the armature, you need to use set_armature(). On the other "
      "hand, the\n"
      "stack of cost functions are implemented in CostModelSum().",
      bp::init<boost::shared_ptr<StateMultibody>,
               boost::shared_ptr<ActuationModelAbstract>,
               boost::shared_ptr<CostModelSum>,
               bp::optional<boost::shared_ptr<ConstraintModelManager> > >(
          bp::args("self", "state", "actuation", "costs", "constraints"),
          "Initialize the free forward-dynamics action model.\n\n"
          ":param state: multibody state\n"
          ":param actuation: abstract actuation model\n"
          ":param costs: stack of cost functions\n"
          ":param constraints: stack of constraint functions"))
      .def<void (DifferentialActionModelImpedanceFwdDynamics::*)(
          const boost::shared_ptr<DifferentialActionDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &DifferentialActionModelImpedanceFwdDynamics::calc,
          bp::args("self", "data", "x", "u"),
          "Compute the next state and cost value.\n\n"
          "It describes the time-continuous evolution of the multibody system "
          "without any contact.\n"
          "Additionally it computes the cost value associated to this state "
          "and control pair.\n"
          ":param data: free forward-dynamics action data\n"
          ":param x: time-continuous state vector\n"
          ":param u: time-continuous control input")
      .def<void (DifferentialActionModelImpedanceFwdDynamics::*)(
          const boost::shared_ptr<DifferentialActionDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &DifferentialActionModelAbstract::calc,
          bp::args("self", "data", "x"))
      .def<void (DifferentialActionModelImpedanceFwdDynamics::*)(
          const boost::shared_ptr<DifferentialActionDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &DifferentialActionModelImpedanceFwdDynamics::calcDiff,
          bp::args("self", "data", "x", "u"),
          "Compute the derivatives of the differential multibody system (free "
          "of contact) and\n"
          "its cost functions.\n\n"
          "It computes the partial derivatives of the differential multibody "
          "system and the\n"
          "cost function. It assumes that calc has been run first.\n"
          "This function builds a quadratic approximation of the\n"
          "action model (i.e. dynamical system and cost function).\n"
          ":param data: free forward-dynamics action data\n"
          ":param x: time-continuous state vector\n"
          ":param u: time-continuous control input\n")
      .def<void (DifferentialActionModelImpedanceFwdDynamics::*)(
          const boost::shared_ptr<DifferentialActionDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &DifferentialActionModelAbstract::calcDiff,
          bp::args("self", "data", "x"))
      .def("createData", &DifferentialActionModelImpedanceFwdDynamics::createData,
           bp::args("self"),
           "Create the free forward dynamics differential action data.")
      .add_property("pinocchio",
                    bp::make_function(
                        &DifferentialActionModelImpedanceFwdDynamics::get_pinocchio,
                        bp::return_internal_reference<>()),
                    "multibody model (i.e. pinocchio model)")
      .add_property("actuation",
                    bp::make_function(
                        &DifferentialActionModelImpedanceFwdDynamics::get_actuation,
                        bp::return_value_policy<bp::return_by_value>()),
                    "actuation model")
      .add_property(
          "costs",
          bp::make_function(&DifferentialActionModelImpedanceFwdDynamics::get_costs,
                            bp::return_value_policy<bp::return_by_value>()),
          "total cost model")
      .add_property(
          "constraints",
          bp::make_function(
              &DifferentialActionModelImpedanceFwdDynamics::get_constraints,
              bp::return_value_policy<bp::return_by_value>()),
          "constraint model manager")
      .add_property("armature",
                    bp::make_function(
                        &DifferentialActionModelImpedanceFwdDynamics::get_armature,
                        bp::return_internal_reference<>()),
                    bp::make_function(
                        &DifferentialActionModelImpedanceFwdDynamics::set_armature),
                    "set an armature mechanism in the joints")
      .def(CopyableVisitor<DifferentialActionModelImpedanceFwdDynamics>());

  bp::register_ptr_to_python<
      boost::shared_ptr<DifferentialActionDataImpedanceFwdDynamics> >();

  bp::class_<DifferentialActionDataImpedanceFwdDynamics,
             bp::bases<DifferentialActionDataAbstract> >(
      "DifferentialActionDataImpedanceFwdDynamics",
      "Action data for the free forward dynamics system.",
      bp::init<DifferentialActionModelImpedanceFwdDynamics*>(
          bp::args("self", "model"),
          "Create free forward-dynamics action data.\n\n"
          ":param model: free forward-dynamics action model"))
      .add_property(
          "pinocchio",
          bp::make_getter(&DifferentialActionDataImpedanceFwdDynamics::pinocchio,
                          bp::return_internal_reference<>()),
          "pinocchio data")
      .add_property(
          "multibody",
          bp::make_getter(&DifferentialActionDataImpedanceFwdDynamics::multibody,
                          bp::return_internal_reference<>()),
          "multibody data")
      .add_property(
          "costs",
          bp::make_getter(&DifferentialActionDataImpedanceFwdDynamics::costs,
                          bp::return_value_policy<bp::return_by_value>()),
          "total cost data")
      .add_property(
          "constraints",
          bp::make_getter(&DifferentialActionDataImpedanceFwdDynamics::constraints,
                          bp::return_value_policy<bp::return_by_value>()),
          "constraint data")
      .add_property(
          "Minv",
          bp::make_getter(&DifferentialActionDataImpedanceFwdDynamics::Minv,
                          bp::return_internal_reference<>()),
          "inverse of the joint-space inertia matrix")
      .add_property(
          "u_drift",
          bp::make_getter(&DifferentialActionDataImpedanceFwdDynamics::u_drift,
                          bp::return_internal_reference<>()),
          "force-bias vector that accounts for control, Coriolis and "
          "gravitational effects")
      .def(CopyableVisitor<DifferentialActionDataImpedanceFwdDynamics>());
}

}  // namespace python
}  // namespace crocoddyl
