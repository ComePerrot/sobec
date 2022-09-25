#include "sobec/pointing/ocp-pointing.hpp"

namespace sobec {
OCP_Point::OCP_Point(const OCPSettings_Point &OCPSettings,
                     RobotDesigner &designer)
    : settings_(OCPSettings), designer_(designer) {}

void OCP_Point::initialize(const Eigen::VectorXd x0, pinocchio::SE3 oMtarget) {
  if (!designer_.initialized_) {
    throw std::runtime_error("The designer must be initialized.");
  }

  buildSolver(x0, oMtarget, settings_.modelMakerSettings);
  solveFirst(x0);

  initialized_ = true;
}

void OCP_Point::solve(const Eigen::VectorXd &measured_x) {
  warm_xs_ = solver_->get_xs();
  warm_xs_.erase(warm_xs_.begin());
  warm_xs_[0] = measured_x;
  warm_xs_.push_back(warm_xs_[warm_xs_.size() - 1]);

  warm_us_ = solver_->get_us();
  warm_us_.erase(warm_us_.begin());
  warm_us_.push_back(warm_us_[warm_us_.size() - 1]);

  // Update initial state
  solver_->get_problem()->set_x0(measured_x);
  solver_->allocateData();

  solver_->solve(warm_xs_, warm_us_, 1, false);
}

///@todo: add initialization check before returning torque or gain
Eigen::VectorXd OCP_Point::get_torque() { return (solver_->get_us()[0]); }
Eigen::MatrixXd OCP_Point::get_gain() { return (solver_->get_K()[0]); }

}  // namespace sobec