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

class ANYmalWbcSpecContainer {
 public:
  ANYmalWbcSpecContainer(RobotSystem* _robot);
  ~ANYmalWbcSpecContainer();
  void weightParamInitialization(const YAML::Node& node);
  void contactParamInitialization(const YAML::Node& node);
  void setContactFriction();
  void setContactFriction(const Eigen::VectorXd& _mu_vec);
  void setContactFriction(int foot_idx, double mu);
  
  RobotSystem* robot_;
  std::vector<Task*> task_list_;
  std::vector<ContactSpec*> contact_list_;

 protected:
  void _InitializeTasks();
  void _InitializeContacts();
  void _InitializeWeightParams();
  void _DeleteTasks();
  void _DeleteContacts();
  void _DeleteOthers();

 public:
  // -------------------------------------------------------
  // Task Member variables
  // -------------------------------------------------------
  Task* com_task_;
  Task* joint_task_;
  Task* base_ori_task_;
  std::array<Task*, ANYmal::n_leg> feet_pos_tasks_;
  std::array<Task*, ANYmal::n_leg> feet_ori_tasks_;
  Task* ee_pos_task_;
  Task* ee_ori_task_;

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
  double w_xddot_z_contact_; // =w_xddot_
  double w_xddot_z_nocontact_;
  double w_rf_;
  double w_rf_z_contact_;
  double w_rf_z_nocontact_;

  double w_xddot_z_trans_;
  double w_rf_z_trans_;

  // 
  Eigen::VectorXd W_xddot_contact_; // contact dim for 1 foot 
  Eigen::VectorXd W_rf_contact_;
  Eigen::VectorXd W_xddot_nocontact_;
  Eigen::VectorXd W_rf_nocontact_;

  // QP weights // be updated from TrajManager
  Eigen::VectorXd W_qddot_; 
  Eigen::VectorXd W_xddot_; // contact dim for all feet in contact
  Eigen::VectorXd W_rf_;

 public:
  // -------------------------------------------------------
  //    set functions
  // -------------------------------------------------------

  // contact
  void set_contact_list(int moving_cop);
  // contact spec
  void set_contact_maxfz(int moving_cop=-1);

  void set_contact_weight_param(int trans_cop=-1);
  void reshape_weight_param(double alpha,
                            int slip_cop,
                            int moving_cop=-1);
  // task
  void clear_task_list();
  void add_task_list(Task* task);
  Task* get_foot_pos_task(int moving_cop);
  Task* get_foot_ori_task(int moving_cop);

  // 
  int footLink2FootIdx(int moving_cop);

};
