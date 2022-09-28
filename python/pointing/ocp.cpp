#include <pinocchio/fwd.hpp>
// #include <pinocchio/multibody/fwd.hpp>

#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <eigenpy/eigenpy.hpp>

#include "sobec/fwd.hpp"
#include "sobec/pointing/ocp-pointing.hpp"

namespace sobec {
namespace python {

using namespace crocoddyl;
namespace bp = boost::python;

void exposeOCPPointParams() {
  bp::register_ptr_to_python<boost::shared_ptr<OCPSettings_Point>>();

  bp::class_<OCPSettings_Point>(
      "OCPSettings_Point",
      bp::init<>(bp::args("self"), "Empty initialization of the OCP params"))
      .def("readFromYaml", &OCPSettings_Point::readParamsFromYamlFile,
           bp::args("filename"))
      .add_property("horizon_length",
                    bp::make_getter(&OCPSettings_Point::horizon_length),
                    bp::make_setter(&OCPSettings_Point::horizon_length),
                    "Number of nodes in the horizon")
      .add_property(
          "modelMakerSettings",
          bp::make_getter(&OCPSettings_Point::modelMakerSettings),
          bp::make_setter(&OCPSettings_Point::modelMakerSettings),
          "Structure containing all the parameters to create the problem.");
}

void exposeOCPPointClass() {
  bp::register_ptr_to_python<boost::shared_ptr<OCP_Point>>();

  bp::class_<OCP_Point>(
      "OCP_Point",
      bp::init<const OCPSettings_Point &, const RobotDesigner &>(
          bp::args("self", "OCPSettings", "designer"),
          "Initialize the OCP from parameter list and a robot designer"))
      .def<void (OCP_Point::*)(const Eigen::VectorXd &,
                               const pinocchio::SE3 &)>(
          "initialize", &OCP_Point::initialize,
          bp::args("self", "x0", "oMtarget"))
      .def<void (OCP_Point::*)(const Eigen::VectorXd &)>(
          "solve", &OCP_Point::solve, bp::args("self", "x_measured"))
      .add_property("torque", &OCP_Point::get_torque,
                    "Torque command computed by the OCP")
      .add_property("gain", &OCP_Point::get_gain, "Gains computed by the OCP");
}

void exposeOCPPoint() {
  exposeOCPPointParams();
  exposeOCPPointClass();
}

}  // namespace python
}  // namespace sobec
