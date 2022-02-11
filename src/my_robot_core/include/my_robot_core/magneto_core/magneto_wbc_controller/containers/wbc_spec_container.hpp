#pragma once

#include <vector>
#include <array>

#include <my_robot_system/RobotSystem.hpp>
#include <my_utils/IO/IOUtilities.hpp>

#include <my_wbc/Contact/ContactSpec.hpp>
#include <my_wbc/Contact/BasicContactSpec.hpp>
#include <my_wbc/Contact/BodyFrameContactSpec.hpp>
#include <my_wbc/Task/Task.hpp>
#include <my_robot_core/magneto_core/magneto_specs/MagnetSpec.hpp>

#include <my_robot_core/magneto_core/magneto_definition.hpp>
#include <my_robot_core/magneto_core/magneto_wbc_controller/tasks/task_set.hpp>


// Object which publicly contains all the tasks, contacts and reaction forces
typedef int FootIdx;
typedef int FootLinkIdx;

class MagnetoWbcSpecContainer {
 public:
  MagnetoWbcSpecContainer(RobotSystem* _robot);
  ~MagnetoWbcSpecContainer();
  void paramInitialization(const YAML::Node& node);
  void setContactFriction();
  void setContactFriction(const Eigen::VectorXd& _mu_vec);
  void setContactFriction(int foot_idx, double mu);
  
  RobotSystem* robot_;
  std::vector<Task*> task_list_;
  std::vector<ContactSpec*> contact_list_;

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
  std::array<Task*, Magneto::n_leg> feet_pos_tasks_;
  std::array<Task*, Magneto::n_leg> feet_ori_tasks_;

  // -------------------------------------------------------
  // Contact Member variables
  // -------------------------------------------------------
  std::array<ContactSpec*, Magneto::n_leg> feet_contacts_;
  int full_contact_dim_;

  // -------------------------------------------------------
  // Magnetic
  // -------------------------------------------------------
  // MagnetSpec* feet_magnets_[Magneto::n_leg];
  std::array<MagnetSpec*, Magneto::n_leg> feet_magnets_;

  Eigen::VectorXd F_magnetic_;
  
  Eigen::VectorXd F_residual_;
  Eigen::MatrixXd J_residual_;
  double w_res_;

  Eigen::VectorXd friction_coeff_;
  Eigen::VectorXd magnetic_force_; //[N]
  Eigen::VectorXd residual_ratio_; //[%]
  Eigen::VectorXd residual_force_;

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

 public:
  // -------------------------------------------------------
  //    set functions
  // -------------------------------------------------------
  // magnetism
  void update_magnetism_map(
              std::map<FootLinkIdx, bool> & b_map);
  void set_magnetism(int moving_cop);
  void set_contact_magnetic_force(int moving_cop);
  void set_residual_magnetic_force(int moving_cop, double contact_distance=0.0);
  // contact
  void set_contact_list(int moving_cop);
  // contact spec
  void set_contact_maxfz(int moving_cop);
  void set_contact_maxfz(int moving_cop,
                        double max_rfz_cntct,
                        double max_rfz_nocntct);
  void compute_weight_param(int moving_cop,
                            const Eigen::VectorXd &W_contact,
                            const Eigen::VectorXd &W_nocontact,
                            Eigen::VectorXd &W_result);
  void reshape_weight_param(double alpha,
                            int slip_cop, 
                            Eigen::VectorXd &W_result);
  // task
  void clear_task_list();
  void add_task_list(Task* task);
  Task* get_foot_pos_task(int moving_cop);
  Task* get_foot_ori_task(int moving_cop);

  // 
  int footLink2FootIdx(int moving_cop);

};
