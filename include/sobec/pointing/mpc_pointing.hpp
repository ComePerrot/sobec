#ifndef SOBEC_MPC_P
#define SOBEC_MPC_P

#include <pinocchio/fwd.hpp>

// include pinocchio first

#include "sobec/pointing/ocp_pointing.hpp"
#include "sobec/walk-with-traj/designer.hpp"

namespace sobec {

struct MPCSettings_Point {
  // Timing parameters
  size_t T_initialization;
  size_t T_stabilization;
  size_t T_drilling;

  // Mocap
  int use_mocap;

  // GainScheduling
  int use_gainScheduling;
  double gainSchedulig_slope;
  double maxGoalWeight;

  // Target
  Eigen::Vector3d targetPos;
  std::vector<pinocchio::SE3> holes_offsets;
  pinocchio::SE3 backwardOffset;
  double tolerance;
};

class MPC_Point {
 private:
  MPCSettings_Point settings_;
  RobotDesigner designer_;
  OCP_Point OCP_;
  boost::shared_ptr<crocoddyl::SolverFDDP> ddp_;

  Eigen::VectorXd x0_;
  Eigen::VectorXd u0_;
  Eigen::MatrixXd K0_;

  // MPC State
  size_t current_hole_ = 0;
  int drilling_state_ = 0;
  size_t iteration_ = 0;
  double running_goal_weight_ = 0;
  double terminal_goal_weight_ = 0;

  // Target related variables
  size_t number_holes_;
  std::vector<pinocchio::SE3>
      list_oMhole_;  // Holes position in the robot frame

  // Security management
  bool initialized_ = false;

  // Memory preallocations:
  pinocchio::SE3 oMtarget_;
  pinocchio::SE3 oMbackwardHole_;
  pinocchio::SE3 tool_se3_hole_;
  double position_error_ = 0;
  std::vector<unsigned long> controlled_joints_id_;
  Eigen::VectorXd x_internal_;

 private:
  void setTarget(pinocchio::SE3 tool_se3_target);
  void updateTarget(pinocchio::SE3 tool_se3_target);
  void updateOCP();
  void setHolesPlacement();

 public:
  MPC_Point();
  MPC_Point(const MPCSettings_Point &settings, const RobotDesigner &design,
            const OCP_Point &OCP, const Eigen::VectorXd &q0,
            const Eigen::VectorXd &v0, pinocchio::SE3 tool_se3_target);

  void initialize(const MPCSettings_Point &settings,
                  const RobotDesigner &design, const OCP_Point &OCP,
                  const Eigen::VectorXd &q0, const Eigen::VectorXd &v0,
                  pinocchio::SE3 tool_se3_target);

  void iterate(const Eigen::VectorXd &q_current,
               const Eigen::VectorXd &v_current,
               pinocchio::SE3 tool_se3_target);

  const Eigen::VectorXd &shapeState(const Eigen::VectorXd &q,
                                    const Eigen::VectorXd &v);

  // getters and setters
  MPCSettings_Point &get_settings() { return settings_; }

  const Eigen::VectorXd &get_x0() const { return x0_; }
  void set_x0(const Eigen::VectorXd &x0) { x0_ = x0; }

  OCP_Point &get_OCP() { return OCP_; }
  void set_OCP(const OCP_Point &OCP) { OCP_ = OCP; }

  RobotDesigner &get_designer() { return designer_; }
  void set_designer(const RobotDesigner &designer) { designer_ = designer; }
};
}  // namespace sobec

#endif  // SOBEC_MPC_P
