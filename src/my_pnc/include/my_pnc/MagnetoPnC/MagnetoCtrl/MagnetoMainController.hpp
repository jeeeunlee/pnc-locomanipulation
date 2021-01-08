#pragma once

#include <my_pnc/MagnetoPnC/MagnetoDefinition.hpp>
#include <my_pnc/MagnetoPnC/MagnetoInterface.hpp>
#include <my_pnc/MagnetoPnC/MagnetoStateProvider.hpp>
#include <my_pnc/MagnetoPnC/MagnetoTaskAndForceContainer/MagnetoTaskAndForceContainer.hpp>
#include <my_wbc/JointIntegrator.hpp>
#include <my_wbc/WBLC/KinWBC.hpp>
#include <my_wbc/WBMC/WBMC.hpp>

class MagnetoMainController {
 public:
  MagnetoMainController(MagnetoTaskAndForceContainer* _taf_container,
                      RobotSystem* _robot);
  virtual ~MagnetoMainController();
  virtual void ctrlInitialization(const YAML::Node& node);
  virtual void getCommand(void* _cmd);

  void getTorqueLimit(Eigen::VectorXd& tau_min, Eigen::VectorXd& tau_max){ 
    tau_min = tau_min_;
    tau_max = tau_max_;
  }

 protected:
  //  Processing Step for first visit
  virtual void firstVisit();
  // Redefine PreProcessing Command
  virtual void _PreProcessing_Command();

 protected:
  RobotSystem* robot_;
  MagnetoTaskAndForceContainer* taf_container_;
  MagnetoStateProvider* sp_;  

  // -------------------------------------------------------
  // Controller Objects
  // -------------------------------------------------------
  std::vector<bool> act_list_;
  KinWBC* kin_wbc_;

  Eigen::VectorXd Fd_des_;
  Eigen::VectorXd tau_cmd_;
  Eigen::VectorXd qddot_cmd_;

  Eigen::VectorXd jpos_des_;
  Eigen::VectorXd jvel_des_;
  Eigen::VectorXd jacc_des_;
  Eigen::VectorXd jtrq_des_;

  Eigen::MatrixXd A_;
  Eigen::MatrixXd Ainv_;
  Eigen::MatrixXd grav_;
  Eigen::MatrixXd coriolis_;
  std::vector<Task*> task_list_;
  std::vector<ContactSpec*> contact_list_;

  // -------------------------------------------------------
  // Parameters
  // -------------------------------------------------------  
  Eigen::VectorXd Kp_, Kd_;

  // Joint Integrator parameters
  double wbc_dt_;
  double vel_freq_cutoff_;  // Hz
  double pos_freq_cutoff_;  // Hz
  double max_pos_error_;    // radians. After position integrator, deviation
                            // from current position

  // 
  bool b_first_visit_;
  bool b_enable_torque_limits_;  // Enable IHWBC torque limits
  double torque_limit_;
  Eigen::VectorXd tau_min_;
  Eigen::VectorXd tau_max_;

 private:
  WBMC* wbmc_;
  WBMC_ExtraData* wbmc_param_;

};
