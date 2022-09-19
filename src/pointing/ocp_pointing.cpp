#include "sobec/pointing/ocp_pointing.hpp"

namespace sobec {
OCP_Point::OCP_Point() {}

OCP_Point::OCP_Point(const OCPSettings_Point &OCPSettings,
                     RobotDesigner &designer, const ModelMaker &modelMaker,
                     const Eigen::VectorXd x0) {
  initialize(OCPSettings, designer, modelMaker, x0);
}

void OCP_Point::initialize(const OCPSettings_Point &OCPSettings,
                           RobotDesigner &designer,
                           const ModelMaker &modelMaker,
                           const Eigen::VectorXd x0) {
  settings_ = OCPSettings;
  designer_ = designer;
  modelMaker_ = modelMaker;

  buildSolver(x0);

  // horizon settings
  std::vector<Eigen::VectorXd> xs_init;
  std::vector<Eigen::VectorXd> us_init;
  Eigen::VectorXd zero_u = Eigen::VectorXd::Zero(designer_.get_rModel().nv - 6);

  for (std::size_t i = 0; i < settings_.horizon_length; i++) {
    xs_init.push_back(x0);
    us_init.push_back(zero_u);
  }
  xs_init.push_back(x0);

  ddp_->solve(xs_init, us_init, 500, false);

  initialized_ = true;
}

void OCP_Point::buildSolver(const Eigen::VectorXd x0) {
  std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> runningModels =
      std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>>(
          settings_.horizon_length);

  for (size_t i = 0; i < settings_.horizon_length; i++) {
    runningModels[i] = modelMaker_.formulatePointingTask();
  }

  boost::shared_ptr<crocoddyl::ActionModelAbstract> terminalModel =
      modelMaker_.formulatePointingTask();

  boost::shared_ptr<crocoddyl::ShootingProblem> shooting_problem =
      boost::make_shared<crocoddyl::ShootingProblem>(x0, runningModels,
                                                     terminalModel);
  ddp_ = boost::make_shared<crocoddyl::SolverFDDP>(shooting_problem);
}

void OCP_Point::solve(const Eigen::VectorXd &measured_x) {
  warm_xs_ = ddp_->get_xs();
  warm_xs_.erase(warm_xs_.begin());
  warm_xs_[0] = measured_x;
  warm_xs_.push_back(warm_xs_[warm_xs_.size() - 1]);

  warm_us_ = ddp_->get_us();
  warm_us_.erase(warm_us_.begin());
  warm_us_.push_back(warm_us_[warm_us_.size() - 1]);

  // Update initial state
  ddp_->get_problem()->set_x0(measured_x);
  ddp_->allocateData();

  ddp_->solve(warm_xs_, warm_us_, 1, false);
}

// Functions to interact with ddp
void OCP_Point::recede() {
  ddp_->get_problem()->circularAppend(
      ddp_->get_problem()->get_runningModels()[0],
      ddp_->get_problem()->get_runningDatas()[0]);
}
void OCP_Point::changeTarget(const size_t index,
                             const Eigen::Ref<const Eigen::Vector3d> position) {
  if (index == settings_.horizon_length) {
    IAM_ = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(
        ddp_->get_problem()->get_terminalModel());
  } else {
    IAM_ = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(
        ddp_->get_problem()->get_runningModels()[index]);
  }
  DAM_ = boost::static_pointer_cast<
      crocoddyl::DifferentialActionModelContactFwdDynamics>(
      IAM_->get_differential());
  frameTranslationResidual_ = boost::static_pointer_cast<
      crocoddyl::ResidualModelFrameTranslation>(
      DAM_->get_costs()->get_costs().at("gripperPosition")->cost->get_residual());

  frameTranslationResidual_->set_reference(position);
}
void OCP_Point::updateGoalPosition(
    const Eigen::Ref<const Eigen::Vector3d> position) {
  for (size_t modelIndex = 0; modelIndex < settings_.horizon_length; modelIndex++) {
    if (modelIndex == settings_.horizon_length) {
      IAM_ = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(
          ddp_->get_problem()->get_terminalModel());
    } else {
      IAM_ = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(
          ddp_->get_problem()->get_runningModels()[modelIndex]);
    }
    DAM_ = boost::static_pointer_cast<
        crocoddyl::DifferentialActionModelContactFwdDynamics>(
        IAM_->get_differential());
    frameTranslationResidual_ = boost::static_pointer_cast<
        crocoddyl::ResidualModelFrameTranslation>(
        DAM_->get_costs()->get_costs().at("gripperPosition")->cost->get_residual());

    frameTranslationResidual_->set_reference(position);
  }
}
void OCP_Point::changeGoalCostActivation(const size_t index, const bool value) {
  if (index == settings_.horizon_length) {
    IAM_ = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(
        ddp_->get_problem()->get_terminalModel());
  } else {
    IAM_ = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(
        ddp_->get_problem()->get_runningModels()[index]);
  }
  DAM_ = boost::static_pointer_cast<
      crocoddyl::DifferentialActionModelContactFwdDynamics>(
      IAM_->get_differential());
  DAM_->get_costs()->get_costs().at("gripperPosition")->active = value;
  DAM_->get_costs()->get_costs().at("gripperRotation")->active = value;
}
void OCP_Point::changeGoaleTrackingWeights() {
  for (size_t modelIndex = 0; modelIndex < settings_.horizon_length; modelIndex++) {
    if (modelIndex == settings_.horizon_length) {
      IAM_ = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(
          ddp_->get_problem()->get_terminalModel());
      DAM_ = boost::static_pointer_cast<
          crocoddyl::DifferentialActionModelContactFwdDynamics>(
          IAM_->get_differential());
      DAM_->get_costs()->get_costs().at("gripperPosition")->weight =
          terminal_goal_weight_;
    } else {
      IAM_ = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(
          ddp_->get_problem()->get_runningModels()[modelIndex]);
      DAM_ = boost::static_pointer_cast<
          crocoddyl::DifferentialActionModelContactFwdDynamics>(
          IAM_->get_differential());
      DAM_->get_costs()->get_costs().at("gripperPosition")->weight =
          running_goal_weight_;
    }
  }
}

Eigen::VectorXd OCP_Point::get_torque() { return (ddp_->get_us()[0]); }

Eigen::MatrixXd OCP_Point::get_gain() { return (ddp_->get_K()[0]); }

}  // namespace sobec