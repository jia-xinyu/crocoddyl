///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021-2023, University of Edinburgh, Heriot-Watt University
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "crocoddyl/multibody/residuals/impedance-force.hpp"

#include "python/crocoddyl/multibody/multibody.hpp"
#include "python/crocoddyl/utils/copyable.hpp"

namespace crocoddyl {
namespace python {

void exposeResidualImpedanceForce() {
  bp::register_ptr_to_python<boost::shared_ptr<ResidualModelImpedanceForce> >();

  bp::class_<ResidualModelImpedanceForce, bp::bases<ResidualModelAbstract> >(
      "ResidualModelImpedanceForce",
      "This residual function defines the tracking of theframe placement "
      "residual as r = p - pref, with p and pref "
      "as\n"
      "the current and reference frame placements, respectively.",
      bp::init<boost::shared_ptr<StateMultibody>, pinocchio::FrameIndex,
               pinocchio::SE3, std::size_t>(
          bp::args("self", "state", "id", "pref", "nu"),
          "Initialize the frame placement residual model.\n\n"
          ":param state: state of the multibody system\n"
          ":param id: reference frame id\n"
          ":param pref: reference frame placement\n"
          ":param nu: dimension of control vector"))
      .def(bp::init<boost::shared_ptr<StateMultibody>, pinocchio::FrameIndex,
                    pinocchio::SE3>(
          bp::args("self", "state", "id", "pref"),
          "Initialize the frame placement residual model.\n\n"
          "The default nu value is obtained from state.nv.\n"
          ":param state: state of the multibody system\n"
          ":param id: reference frame id\n"
          ":param pref: reference frame placement"))
      .def<void (ResidualModelImpedanceForce::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelImpedanceForce::calc,
          bp::args("self", "data", "x", "u"),
          "Compute the frame placement residual.\n\n"
          ":param data: residual data\n"
          ":param x: state point (dim. state.nx)\n"
          ":param u: control input (dim. nu)")
      .def<void (ResidualModelImpedanceForce::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelAbstract::calc, bp::args("self", "data", "x"))
      .def<void (ResidualModelImpedanceForce::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelImpedanceForce::calcDiff,
          bp::args("self", "data", "x", "u"),
          "Compute the Jacobians of the frame placement residual.\n\n"
          "It assumes that calc has been run first.\n"
          ":param data: action data\n"
          ":param x: state point (dim. state.nx)\n"
          ":param u: control input (dim. nu)")
      .def<void (ResidualModelImpedanceForce::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelAbstract::calcDiff,
          bp::args("self", "data", "x"))
      .def("createData", &ResidualModelImpedanceForce::createData,
           bp::with_custodian_and_ward_postcall<0, 2>(),
           bp::args("self", "data"),
           "Create the frame placement residual data.\n\n"
           "Each residual model has its own data that needs to be allocated. "
           "This function\n"
           "returns the allocated data for the frame placement residual.\n"
           ":param data: shared data\n"
           ":return residual data.")
      .add_property("id", &ResidualModelImpedanceForce::get_id,
                    &ResidualModelImpedanceForce::set_id, "reference frame id")
      .add_property(
          "reference",
          bp::make_function(&ResidualModelImpedanceForce::get_reference,
                            bp::return_internal_reference<>()),
          &ResidualModelImpedanceForce::set_reference,
          "reference frame placement")
      .def(CopyableVisitor<ResidualModelImpedanceForce>());

  bp::register_ptr_to_python<boost::shared_ptr<ResidualDataImpedanceForce> >();

  bp::class_<ResidualDataImpedanceForce, bp::bases<ResidualDataAbstract> >(
      "ResidualDataImpedanceForce", "Data for frame placement residual.\n\n",
      bp::init<ResidualModelImpedanceForce*, DataCollectorAbstract*>(
          bp::args("self", "model", "data"),
          "Create frame placement residual data.\n\n"
          ":param model: frame placement residual model\n"
          ":param data: shared data")[bp::with_custodian_and_ward<
          1, 2, bp::with_custodian_and_ward<1, 3> >()])
      .add_property("pinocchio",
                    bp::make_getter(&ResidualDataImpedanceForce::pinocchio,
                                    bp::return_internal_reference<>()),
                    "pinocchio data")
      .add_property(
          "rMf",
          bp::make_getter(&ResidualDataImpedanceForce::rMf,
                          bp::return_value_policy<bp::return_by_value>()),
          "error frame placement of the frame")
      .add_property("rJf",
                    bp::make_getter(&ResidualDataImpedanceForce::rJf,
                                    bp::return_internal_reference<>()),
                    "error Jacobian of the frame")
      .add_property("fJf",
                    bp::make_getter(&ResidualDataImpedanceForce::fJf,
                                    bp::return_internal_reference<>()),
                    "local Jacobian of the frame")
      .def(CopyableVisitor<ResidualDataImpedanceForce>());
}

}  // namespace python
}  // namespace crocoddyl
