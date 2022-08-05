///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, University of Edinburgh, LAAS-CNRS, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "sobec/crocomplements/residual-anticipated-state.hpp"

#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <eigenpy/eigenpy.hpp>
#include <pinocchio/multibody/fwd.hpp>  // Must be included first!

#include "sobec/fwd.hpp"

namespace sobec {
namespace python {

using namespace crocoddyl;
namespace bp = boost::python;

void exposeResidualAnticipatedState() {
  bp::register_ptr_to_python<
      boost::shared_ptr<ResidualModelAnticipatedState> >();

  bp::class_<ResidualModelAnticipatedState, bp::bases<ResidualModelAbstract> >(
      "ResidualModelAnticipatedState",
      "This cost function defines a residual vector as r = q + time * v, with "
      "q and v as\n"
      "the current position and velocity joints and time a diagonal matrix.",
      bp::init<boost::shared_ptr<StateAbstract>, std::size_t, double>(
          bp::args("self", "state", "nu", "anticipated_time"),
          "Initialize the state cost model.\n\n"
          ":param state: state description\n"
          ":param nu: dimension of control vector\n"
          ":param anticipated_time: time of anticipation"))
      .def(bp::init<boost::shared_ptr<StateAbstract>, double>(
          bp::args("self", "state", "anticipated_time"),
          "Initialize the state cost model.\n\n"
          "The default nu value is obtained from state.nv.\n"
          ":param state: state description\n"
          ":param anticipated_time: time of anticipation"))
      .def<void (ResidualModelAnticipatedState::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelAnticipatedState::calc,
          bp::args("self", "data", "x", "u"),
          "Compute the state cost.\n\n"
          ":param data: cost data\n"
          ":param x: time-discrete state vector\n"
          ":param u: time-discrete control input")
      .def<void (ResidualModelAnticipatedState::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelAbstract::calc, bp::args("self", "data", "x"))
      .def<void (ResidualModelAnticipatedState::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelAnticipatedState::calcDiff,
          bp::args("self", "data", "x", "u"),
          "Compute the derivatives of the state cost.\n\n"
          "It assumes that calc has been run first.\n"
          ":param data: action data\n"
          ":param x: time-discrete state vector\n"
          ":param u: time-discrete control input\n")
      .def<void (ResidualModelAnticipatedState::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelAbstract::calcDiff,
          bp::args("self", "data", "x"));

  bp::register_ptr_to_python<
      boost::shared_ptr<ResidualDataAnticipatedState> >();

  bp::class_<ResidualDataAnticipatedState, bp::bases<ResidualDataAbstract> >(
      "ResidualDataAnticipatedState",
      "Data for anticipated state residual.\n\n",
      bp::init<ResidualModelAnticipatedState*, DataCollectorAbstract*>(
          bp::args("self", "model", "data"),
          "Create anticipated state residual data.\n\n"
          ":param model: anticipated state residual model\n"
          ":param data: shared data")[bp::with_custodian_and_ward<
          1, 2, bp::with_custodian_and_ward<1, 3> >()]);
}

}  // namespace python
}  // namespace sobec
