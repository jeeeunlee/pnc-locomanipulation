#pragma once

#include <my_pnc/MagnetoPnC/MagnetoDefinition.hpp>
#include <my_pnc/TaskAndForceContainer.hpp>
#include <my_wbc/Contact/BasicContactSpec.hpp>
#include <my_wbc/Contact/BodyFrameContactSpec.hpp>
#include <my_pnc/MagnetoPnC/MagnetoTask/TaskSet.hpp>


// Object which publicly contains all the tasks, contacts and reaction forces
class MagnetoTaskAndForceContainer : public TaskAndForceContainer {
 public:
  MagnetoTaskAndForceContainer(RobotSystem* _robot);
  ~MagnetoTaskAndForceContainer();
  void paramInitialization(const YAML::Node& node);
  
  void setContactFriction();
  void setContactFriction(double _mu);

  // -------------------------------------------------------
  //    set functions
  // -------------------------------------------------------
  // magnetism
  void set_magnetism(int moving_cop);
  void set_contact_magnetic_force(int moving_cop);
  void set_residual_magnetic_force(int moving_cop, double contact_distance=0.0);
  // contact
  void set_contact_list(int moving_cop);
  // contact spec
  void set_maxfz_contact(int moving_cop);
  void set_maxfz_contact(int moving_cop,
                        double max_rfz_cntct,
                        double max_rfz_nocntct);
  void compute_weight_param(int moving_cop,
                            const Eigen::VectorXd &W_contact,
                            const Eigen::VectorXd &W_nocontact,
                            Eigen::VectorXd &W_result);
  // task
  void clear_task_list();
  void add_task_list(Task* task);
  Task* get_foot_pos_task(int moving_cop);
  Task* get_foot_ori_task(int moving_cop);

 protected:
  void _InitializeTasks();
  void _InitializeContacts();
  void _InitializeMagnetisms();
  void _DeleteTasks();
  void _DeleteContacts();

 public:
  // -------------------------------------------------------
  // Task Member variables
  // -------------------------------------------------------
  Task* com_task_;
  Task* joint_task_;
  Task* base_ori_task_;

  Task* alfoot_pos_task_;
  Task* arfoot_pos_task_;
  Task* blfoot_pos_task_;
  Task* brfoot_pos_task_;

  Task* alfoot_ori_task_;
  Task* arfoot_ori_task_;
  Task* blfoot_ori_task_;
  Task* brfoot_ori_task_;

  // -------------------------------------------------------
  // Contact Member variables
  // -------------------------------------------------------
  ContactSpec* alfoot_contact_;
  ContactSpec* arfoot_contact_;
  ContactSpec* blfoot_contact_;
  ContactSpec* brfoot_contact_;
  std::vector<ContactSpec*> full_contact_list_;
  int dim_contact_;
  int full_dim_contact_;

  double max_fz_;

  // -------------------------------------------------------
  // Magnetic
  // -------------------------------------------------------
  std::map<int, bool> b_magnetism_map_;
  Eigen::VectorXd F_magnetic_;
  
  Eigen::VectorXd F_residual_;
  Eigen::MatrixXd J_residual_;
  double w_res_;

  double friction_coeff_;
  double magnetic_force_; //[N]
  double residual_ratio_; //[%]
  double residual_force_;

  // -------------------------------------------------------
  // Parameters
  // -------------------------------------------------------
  // Max rf_z for contactSpec in contact_list
  double max_rf_z_contact_;
  double max_rf_z_nocontact_;

  double max_rf_z_trans_; // be updated from TrajManager

  // QP weights init & target
  double w_qddot_;
  double w_xddot_contact_;
  double w_xddot_nocontact_;
  double w_rf_;
  double w_rf_z_contact_;
  double w_rf_z_nocontact_;

  // 
  Eigen::VectorXd W_xddot_contact_; // contact dim for 1 foot 
  Eigen::VectorXd W_rf_contact_;

  Eigen::VectorXd W_xddot_nocontact_;
  Eigen::VectorXd W_rf_nocontact_;

  // QP weights // be updated from TrajManager
  Eigen::VectorXd W_qddot_; 
  Eigen::VectorXd W_xddot_; // contact dim for all feet in contact
  Eigen::VectorXd W_rf_;

};
