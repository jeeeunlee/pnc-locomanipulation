#pragma once

#include <vector>
#include <array>

#include <my_robot_system/RobotSystem.hpp>
#include <my_utils/IO/IOUtilities.hpp>

#include <my_wbc/Contact/ContactSpec.hpp>
#include <my_wbc/Contact/BasicContactSpec.hpp>
#include <my_wbc/Contact/GroundFrameContactSpec.hpp>
#include <my_wbc/Task/Task.hpp>
#include <my_robot_core/anymal_core/anymal_specs/ContactWeight.hpp>

#include <my_robot_core/anymal_core/anymal_definition.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/tasks/task_set.hpp>


// Object which publicly contains all the tasks, contacts and reaction forces
typedef int FootIdx;
typedef int FootLinkIdx;

namespace ANYMAL_TASK {
// in priority
// constexpr int COM = 0;
// constexpr int BASE_ORI = 1;
// constexpr int LF_POS = 2;
// constexpr int LH_POS = 3;
// constexpr int RF_POS = 4;
// constexpr int RH_POS = 5;
// constexpr int LF_ORI = 6;
// constexpr int LH_ORI = 7;
// constexpr int RF_ORI = 8;
// constexpr int RH_ORI = 9;

constexpr int LF_POS = 0;
constexpr int LH_POS = 1;
constexpr int RF_POS = 2;
constexpr int RH_POS = 3;
constexpr int LF_ORI = 4;
constexpr int LH_ORI = 5;
constexpr int RF_ORI = 6;
constexpr int RH_ORI = 7;
constexpr int BASE_ORI = 8;
constexpr int COM = 9;
constexpr int EE_POS = 10;
constexpr int EE_ORI = 11;
constexpr int JOINT_TASK = 12;
constexpr int n_task = 13;
}; //namespace ANYMAL_TASK 

class ANYmalWbcSpecContainer {
 public:
  ANYmalWbcSpecContainer(RobotSystem* _robot);
  ~ANYmalWbcSpecContainer();
  void weightParamInitialization(const YAML::Node& node);
  void contactParamInitialization(const YAML::Node& node);

  // utils
  int footLink2FootIdx(int moving_cop);
  
 public:
  RobotSystem* robot_;
  // -------------------------------------------------------
  // Task Member variables
  // -------------------------------------------------------
  std::array<bool, ANYMAL_TASK::n_task> b_task_list_;
  std::array<Task*, ANYMAL_TASK::n_task> task_container_;

  void clear_task_list();
  void delete_task_list(int task_id){ b_task_list_[task_id] = false; }
  void add_task_list(int task_id){ b_task_list_[task_id] = true; }
  void check_task_list();

  // -------------------------------------------------------
  // Contact Member variables
  // -------------------------------------------------------
  std::array<bool, ANYmal::n_leg> b_feet_contact_list_;
  std::array<ContactSpec*, ANYmal::n_leg> feet_contacts_;
  int full_contact_dim_;
  Eigen::VectorXd friction_coeff_;

  // -------------------------------------------------------
  // Weight Parameters
  // -------------------------------------------------------
  std::array<ContactWeight*, ANYmal::n_leg> feet_weights_;
  
  // Max rf_z for contactSpec in contact_list
  double max_rf_z_contact_;
  double max_rf_z_nocontact_;
  double max_rf_z_trans_; // be updated from TrajManager

  // QP weights init & target
  double w_qddot_;
  double w_xddot_;
  double w_xddot_z_contact_;
  double w_xddot_z_nocontact_;
  double w_xddot_z_trans_; // be updated from TrajManager
  double w_rf_;
  double w_rf_z_contact_;
  double w_rf_z_nocontact_;  
  double w_rf_z_trans_; // be updated from TrajManager

  // QP weights // be updated from TrajManager
  Eigen::VectorXd W_qddot_; 

  void reshape_weight_param(double alpha,
                            int slip_cop,
                            int moving_cop=-1);  

};
