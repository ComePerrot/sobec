#include "sobec/pointing/mpc_pointing.hpp"

namespace sobec {

#define PI 3.14159265

MPC_Point::MPC_Point() {}

MPC_Point::MPC_Point(const MPCSettings_Point &settings,
                     const RobotDesigner &design, const OCP_Point &OCP,
                     const Eigen::VectorXd &q0, const Eigen::VectorXd &v0,
                     pinocchio::SE3 tool_se3_target) {
  initialize(settings, design, OCP, q0, v0, tool_se3_target);
}

void MPC_Point::initialize(const MPCSettings_Point &settings,
                           const RobotDesigner &design, const OCP_Point &OCP,
                           const Eigen::VectorXd &q0, const Eigen::VectorXd &v0,
                           pinocchio::SE3 tool_se3_target) {
  if (!design.initialized_ || !OCP.initialized_) {
    throw std::runtime_error("The designer and OCP must be initialized.");
  }
  settings_ = settings;
  designer_ = design;
  OCP_ = OCP;

  x0_.resize(designer_.get_rModel().nq + designer_.get_rModel().nv);
  x0_ << shapeState(q0, v0);
  designer_.updateReducedModel(x0_);
  designer_.updateCompleteModel(q0);

  // Setup target
  setTarget(tool_se3_target);

  // Init OCP
  OCP_.solve(x0_);

  u0_ = OCP_.get_torque();
  K0_ = OCP_.get_gain();
  initialized_ = true;
}

void MPC_Point::iterate(const Eigen::VectorXd &q_current,
                        const Eigen::VectorXd &v_current,
                        pinocchio::SE3 tool_se3_target) {
  x0_ = shapeState(q_current, v_current);

  designer_.updateReducedModel(x0_);

  updateTarget(tool_se3_target);
  updateOCP();
  OCP_.solve(x0_);
  u0_ = OCP_.get_torque();
  K0_ = OCP_.get_gain();
}

void MPC_Point::setTarget(pinocchio::SE3 tool_se3_target) {
  // Setup target
  number_holes_ = settings_.holes_offsets.size();

  //  Define oMtarget
  if (settings_.use_mocap == 1 || settings_.use_mocap == 2) {
    tool_se3_hole_ =
        tool_se3_target.act(settings_.holes_offsets[current_hole_]);
    oMtarget_ = designer_.get_EndEff_frame().act(tool_se3_hole_);
  } else {
    oMtarget_.translation() = settings_.targetPos;

    // To have the gripper_left_fingertip_3_link in the right orientation we
    // need :
    //   90° around the y axis
    //   180° around the z axis
    double beta = -PI * 0.5;
    double gamma = PI;

    Eigen::Matrix3d rotationY;
    rotationY.row(0) << cos(beta), 0, -sin(beta);
    rotationY.row(1) << 0, 1, 0;
    rotationY.row(2) << sin(beta), 0, cos(beta);
    Eigen::Matrix3d rotationZ;
    rotationZ.row(0) << cos(gamma), sin(gamma), 0;
    rotationZ.row(1) << -sin(gamma), cos(gamma), 0;
    rotationZ.row(2) << 0, 0, 1;

    oMtarget_.rotation() = rotationZ * rotationY;
  }

  //  Setup list_oMholes
  setHolesPlacement();
}

void MPC_Point::updateTarget(pinocchio::SE3 tool_se3_target) {
  if (settings_.use_mocap == 0 || settings_.use_mocap == 1) {
    tool_se3_hole_ =
        designer_.get_EndEff_frame().actInv(list_oMhole_[current_hole_]);
    position_error_ = tool_se3_hole_.translation().norm();
  } else if (settings_.use_mocap == 2) {
    tool_se3_hole_ =
        tool_se3_target.act(settings_.holes_offsets[current_hole_]);

    position_error_ = tool_se3_hole_.translation().norm();

    if (position_error_ < 0.05 && drilling_state_ == 2) {
      oMtarget_ = designer_.get_EndEff_frame().act(tool_se3_hole_);

      setHolesPlacement();
      OCP_.updateGoalPosition(list_oMhole_[current_hole_].translation());
    }
  }
}

void MPC_Point::updateOCP() {
  switch (drilling_state_) {
    case 0:  // Initialization
      if (iteration_ < settings_.T_initialization) {
        iteration_++;
      } else {
        iteration_ = 0;
        drilling_state_++;
      }
      break;

    case 1:  // Approaching target
      if (iteration_ == 0) {
        std::cout << "Initiating movement" << std::endl;
      }
      if (iteration_ <= OCP_.get_horizonLength()) {
        OCP_.changeGoalCostActivation(OCP_.get_horizonLength() - iteration_,
                                      true);
        iteration_++;
      } else {
        std::cout << "Approach phase over" << std::endl;

        iteration_ = 0;
        drilling_state_++;
      }
      break;

    case 2:  // Waiting for arm to stabilize (and closing loop on MOCAP
             // feedback)
      if (iteration_ == 0) {
        std::cout << "Waiting for arm to stabilize" << std::endl;
      }
      if (iteration_ < settings_.T_stabilization) {
        iteration_++;
      } else {
        if (position_error_ > 0.05) {
          iteration_ = 0;
          drilling_state_ = 6;
        } else {
          iteration_ = 0;
          drilling_state_++;
        }
      }
      break;

    case 3:  // Increasing gain
      if (iteration_ == 0) {
        std::cout << "Initiating increase gain phase" << std::endl;
      }
      if (settings_.use_gainScheduling == 1 &&
          position_error_ > settings_.tolerance &&
          running_goal_weight_ < settings_.maxGoalWeight) {
        running_goal_weight_ += settings_.gainSchedulig_slope;
        terminal_goal_weight_ += settings_.gainSchedulig_slope;

        OCP_.changeGoaleTrackingWeights();

        iteration_++;
      } else {
        std::cout << "Gain Scheduling over" << std::endl;

        iteration_ = 0;
        drilling_state_++;
      }
      break;

    case 4:  // Drilling
      if (iteration_ == 0) {
        std::cout << "Initiating drilling phase" << std::endl;
      }
      if (iteration_ <= settings_.T_drilling) {
        iteration_++;
      } else {
        std::cout << "Drilling over" << std::endl;

        iteration_ = 0;
        drilling_state_++;
      }
      break;

    case 5:  // Getting out of hole
      if (iteration_ == 0) {
        std::cout << "Getting out of the hole" << std::endl;
        if (settings_.use_gainScheduling == 1) {
          running_goal_weight_ = 5;
          terminal_goal_weight_ = 50;
          OCP_.changeGoaleTrackingWeights();
        }
        oMbackwardHole_ =
            list_oMhole_[current_hole_].act(settings_.backwardOffset);
      }
      if (iteration_ <= OCP_.get_horizonLength()) {
        OCP_.changeTarget(OCP_.get_horizonLength() - iteration_,
                          oMbackwardHole_.translation());
        iteration_++;
      } else {
        iteration_ = 0;
        drilling_state_++;
      }
      break;

    case 6:  // Switching to next target
      if (iteration_ == 0) {
        if (current_hole_ < (number_holes_ - 1)) {
          current_hole_++;
          std::cout << "Switching to target:" << current_hole_ + 1 << std::endl;
          iteration_++;
        } else {
          std::cout << "Going back to inital position" << std::endl;
          drilling_state_ = 7;
        }
      }
      if (iteration_ <= OCP_.get_horizonLength()) {
        OCP_.changeTarget(OCP_.get_horizonLength() - iteration_,
                          list_oMhole_[current_hole_].translation());
        iteration_++;
      } else {
        iteration_ = 0;
        drilling_state_ = 3;
      }
      break;

    case 7:  // Returning to initial position
      if (iteration_ <= OCP_.get_horizonLength()) {
        OCP_.changeGoalCostActivation(OCP_.get_horizonLength() - iteration_,
                                      false);
        iteration_++;
      } else {
        std::cout << "Robot is back to initial position" << std::endl;

        iteration_ = 0;
        drilling_state_++;
      }
      break;
  }
}

void MPC_Point::setHolesPlacement() {
  if (list_oMhole_.empty()) {
    for (size_t h = 0; h < number_holes_; h++) {
      list_oMhole_.push_back(oMtarget_.act(settings_.holes_offsets[h]));
    }
  } else {
    for (size_t h = 0; h < number_holes_; h++) {
      list_oMhole_[h] = oMtarget_.act(settings_.holes_offsets[h]);
    }
  }
}

const Eigen::VectorXd &MPC_Point::shapeState(const Eigen::VectorXd &q,
                                             const Eigen::VectorXd &v) {
  if (q.size() == designer_.get_rModelComplete().nq &&
      v.size() == designer_.get_rModelComplete().nv) {
    x_internal_.head<7>() = q.head<7>();
    x_internal_.segment<6>(designer_.get_rModel().nq) = v.head<6>();

    int i = 0;
    for (unsigned long jointID : controlled_joints_id_)
      if (jointID > 1) {
        x_internal_(i + 7) = q(jointID + 5);
        x_internal_(designer_.get_rModel().nq + i + 6) = v(jointID + 4);
        i++;
      }
    return x_internal_;
  } else if (q.size() == designer_.get_rModel().nq &&
             v.size() == designer_.get_rModel().nv) {
    x_internal_ << q, v;
    return x_internal_;
  } else
    throw std::runtime_error(
        "q and v must have the dimentions of the reduced or complete model.");
}
}  // namespace sobec
