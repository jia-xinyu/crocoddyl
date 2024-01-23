///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2024, University of Edinburgh, Heriot-Watt University
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "actuation.hpp"

#include "crocoddyl/core/actuation/actuation-squashing.hpp"
#include "crocoddyl/core/actuation/squashing-base.hpp"
#include "crocoddyl/core/actuation/squashing/smooth-sat.hpp"
#include "crocoddyl/core/utils/exception.hpp"
#include "crocoddyl/multibody/actuations/floating-base-propellers.hpp"
#include "crocoddyl/multibody/actuations/floating-base.hpp"
#include "crocoddyl/multibody/actuations/full.hpp"

namespace crocoddyl {
namespace unittest {

const std::vector<ActuationModelTypes::Type> ActuationModelTypes::all(
    ActuationModelTypes::init_all());

std::ostream& operator<<(std::ostream& os, ActuationModelTypes::Type type) {
  switch (type) {
    case ActuationModelTypes::ActuationModelFull:
      os << "ActuationModelFull";
      break;
    case ActuationModelTypes::ActuationModelFloatingBase:
      os << "ActuationModelFloatingBase";
      break;
    case ActuationModelTypes::ActuationModelFloatingBasePropellers:
      os << "ActuationModelFloatingBasePropellers";
      break;
    case ActuationModelTypes::ActuationModelSquashingFull:
      os << "ActuationModelSquashingFull";
      break;
    case ActuationModelTypes::NbActuationModelTypes:
      os << "NbActuationModelTypes";
      break;
    default:
      break;
  }
  return os;
}

ActuationModelFactory::ActuationModelFactory() {}
ActuationModelFactory::~ActuationModelFactory() {}

boost::shared_ptr<crocoddyl::ActuationModelAbstract>
ActuationModelFactory::create(ActuationModelTypes::Type actuation_type,
                              StateModelTypes::Type state_type) const {
  boost::shared_ptr<crocoddyl::ActuationModelAbstract> actuation;
  StateModelFactory factory;
  boost::shared_ptr<crocoddyl::StateAbstract> state =
      factory.create(state_type);
  boost::shared_ptr<crocoddyl::StateMultibody> state_multibody;
  // Propeller objects
  std::vector<crocoddyl::Propeller> ps;
  const double d_cog = 0.1525;
  const double cf = 6.6e-5;
  const double cm = 1e-6;
  pinocchio::SE3 p1(Eigen::Matrix3d::Identity(), Eigen::Vector3d(d_cog, 0, 0));
  pinocchio::SE3 p2(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, d_cog, 0));
  pinocchio::SE3 p3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-d_cog, 0, 0));
  pinocchio::SE3 p4(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, -d_cog, 0));
  ps.push_back(crocoddyl::Propeller(p1, cf, cm, crocoddyl::PropellerType::CCW));
  ps.push_back(crocoddyl::Propeller(p2, cf, cm, crocoddyl::PropellerType::CW));
  ps.push_back(crocoddyl::Propeller(p3, cf, cm, crocoddyl::PropellerType::CW));
  ps.push_back(crocoddyl::Propeller(p4, cf, cm, crocoddyl::PropellerType::CCW));
  // Actuation Squashing objects
  boost::shared_ptr<crocoddyl::ActuationModelAbstract> act;
  boost::shared_ptr<crocoddyl::SquashingModelSmoothSat> squash;
  Eigen::VectorXd lb;
  Eigen::VectorXd ub;
  switch (actuation_type) {
    case ActuationModelTypes::ActuationModelFull:
      state_multibody =
          boost::static_pointer_cast<crocoddyl::StateMultibody>(state);
      actuation =
          boost::make_shared<crocoddyl::ActuationModelFull>(state_multibody);
      break;
    case ActuationModelTypes::ActuationModelFloatingBase:
      state_multibody =
          boost::static_pointer_cast<crocoddyl::StateMultibody>(state);
      actuation = boost::make_shared<crocoddyl::ActuationModelFloatingBase>(
          state_multibody);
      break;
    case ActuationModelTypes::ActuationModelFloatingBasePropellers:
      state_multibody =
          boost::static_pointer_cast<crocoddyl::StateMultibody>(state);
      actuation =
          boost::make_shared<crocoddyl::ActuationModelFloatingBasePropellers>(
              state_multibody, ps);
      break;
    case ActuationModelTypes::ActuationModelSquashingFull:
      state_multibody =
          boost::static_pointer_cast<crocoddyl::StateMultibody>(state);

      act = boost::make_shared<crocoddyl::ActuationModelFull>(state_multibody);

      lb = Eigen::VectorXd::Zero(state->get_nv());
      ub = Eigen::VectorXd::Zero(state->get_nv());
      lb.fill(-100.0);
      ub.fill(100.0);
      squash = boost::make_shared<crocoddyl::SquashingModelSmoothSat>(
          lb, ub, state->get_nv());

      actuation = boost::make_shared<crocoddyl::ActuationSquashingModel>(
          act, squash, state->get_nv());
      break;
    default:
      throw_pretty(__FILE__ ":\n Construct wrong ActuationModelTypes::Type");
      break;
  }
  return actuation;
}

void updateActuation(
    const boost::shared_ptr<crocoddyl::ActuationModelAbstract>& model,
    const boost::shared_ptr<crocoddyl::ActuationDataAbstract>& data,
    const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
  model->calc(data, x, u);
}

}  // namespace unittest
}  // namespace crocoddyl
