#ifndef SOBEC_OCP_P
#define SOBEC_OCP_P

#include <pinocchio/fwd.hpp>

// include pinocchio first

#include "sobec/walk-with-traj/designer.hpp"
#include "sobec/walk-with-traj/model_factory.hpp"

namespace sobec {
struct OCPSettings_Point {
  size_t horizon_length;
};

class OCP_Point {
 private:
  OCPSettings_Point settings_;
  RobotDesigner designer_;
  ModelMaker modelMaker_;
  DDP ddp_;

  size_t horizon_length_;
  double running_goal_weight_;
  double terminal_goal_weight_;

  // prealocated memory:
  std::vector<Eigen::VectorXd> warm_xs_;
  std::vector<Eigen::VectorXd> warm_us_;
  boost::shared_ptr<crocoddyl::ResidualModelFrameTranslation>
      frameTranslationResidual_;
  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> IAM_;
  boost::shared_ptr<crocoddyl::DifferentialActionModelContactFwdDynamics> DAM_;

 public:
  OCP_Point();

  OCP_Point(const OCPSettings_Point &OCPSettings, const ModelMaker &modelMaker,
            const Eigen::VectorXd x0);

  void initialize(const OCPSettings_Point &OCPSettings,
                  const ModelMaker &modelMaker, const Eigen::VectorXd x0);
  bool initialized_ = false;
  void buildSolver(const Eigen::VectorXd x0);

  void solve(const Eigen::VectorXd &measured_x);

  void recede();
  void changeTarget(const size_t index,
                    const Eigen::Ref<const Eigen::Vector3d> position);
  void updateGoalPosition(const Eigen::Ref<const Eigen::Vector3d> position);
  void changeGoalCostActivation(const size_t index, const bool value);
  void changeGoaleTrackingWeights();

  Eigen::VectorXd get_torque();
  Eigen::MatrixXd get_gain();

  size_t get_horizonLength() { return (horizon_length_); };
};

}  // namespace sobec

#endif  // SOBEC_OCP_P
