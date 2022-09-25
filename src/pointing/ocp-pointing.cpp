#include "sobec/pointing/ocp-pointing.hpp"

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

void OCP_Point::solveFirst(const Eigen::VectorXd x) {
  // horizon settings
  std::vector<Eigen::VectorXd> xs_init;
  std::vector<Eigen::VectorXd> us_init;
  Eigen::VectorXd zero_u = Eigen::VectorXd::Zero(designer_.get_rModel().nv - 6);

  for (std::size_t i = 0; i < settings_.horizon_length; i++) {
    xs_init.push_back(x);
    us_init.push_back(zero_u);
  }
  xs_init.push_back(x);

  ddp_->solve(xs_init, us_init, 500, false);
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
  boost::static_pointer_cast<crocoddyl::ResidualModelFrameTranslation>(
      costs(index)->get_costs().at("gripperPosition")->cost->get_residual())
      ->set_reference(position);
}
void OCP_Point::setBalancingTorques() {
  for (size_t modelIndex = 0; modelIndex < settings_.horizon_length;
       modelIndex++) {
    Eigen::VectorXd x_ref =
        boost::static_pointer_cast<crocoddyl::ResidualModelState>(
            costs(modelIndex)
                ->get_costs()
                .at("postureTask")
                ->cost->get_residual())
            ->get_reference();

    Eigen::VectorXd balancingTorque;
    balancingTorque.resize((long) iam(modelIndex)->get_nu());
    iam(modelIndex)->quasiStatic(ada(modelIndex), balancingTorque, x_ref);

    boost::static_pointer_cast<crocoddyl::ResidualModelControl>(
        costs(modelIndex)
            ->get_costs()
            .at("actuationTask")
            ->cost->get_residual())
        ->set_reference(balancingTorque);
  }
}
void OCP_Point::updateGoalPosition(
    const Eigen::Ref<const Eigen::Vector3d> position) {
  for (size_t modelIndex = 0; modelIndex < settings_.horizon_length;
       modelIndex++) {
    boost::static_pointer_cast<crocoddyl::ResidualModelFrameTranslation>(
        costs(modelIndex)
            ->get_costs()
            .at("gripperPosition")
            ->cost->get_residual())
        ->set_reference(position);
  }
}
void OCP_Point::updateGoalRotation(
    const Eigen::Ref<const Eigen::Matrix3d> rotation) {
  for (size_t modelIndex = 0; modelIndex < settings_.horizon_length;
       modelIndex++) {
    boost::static_pointer_cast<crocoddyl::ResidualModelFrameRotation>(
        costs(modelIndex)
            ->get_costs()
            .at("gripperRotation")
            ->cost->get_residual())
        ->set_reference(rotation);
  }
}
void OCP_Point::changeGoalCostActivation(const size_t index, const bool value) {
  costs(index)->get_costs().at("gripperPosition")->active = value;
  costs(index)->get_costs().at("gripperRotation")->active = value;
}
void OCP_Point::changeGoaleTrackingWeights(double weight) {
  for (size_t modelIndex = 0; modelIndex < settings_.horizon_length;
       modelIndex++) {
    costs(modelIndex)->get_costs().at("gripperPosition")->weight = weight;
  }
}

AMA OCP_Point::ama(const unsigned long time) {
  if (time == settings_.horizon_length) {
    return ddp_->get_problem()->get_terminalModel();
  } else {
    return ddp_->get_problem()->get_runningModels()[time];
  }
}

IAM OCP_Point::iam(const unsigned long time) {
  return boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(
      ama(time));
}

DAM OCP_Point::dam(const unsigned long time) {
  return boost::static_pointer_cast<
      crocoddyl::DifferentialActionModelContactFwdDynamics>(
      iam(time)->get_differential());
}

Cost OCP_Point::costs(const unsigned long time) {
  return dam(time)->get_costs();
}

ADA OCP_Point::ada(const unsigned long time) {
  return ddp_->get_problem()->get_runningDatas()[time];
}

Eigen::VectorXd OCP_Point::get_torque() { return (ddp_->get_us()[0]); }

Eigen::MatrixXd OCP_Point::get_gain() { return (ddp_->get_K()[0]); }

}  // namespace sobec