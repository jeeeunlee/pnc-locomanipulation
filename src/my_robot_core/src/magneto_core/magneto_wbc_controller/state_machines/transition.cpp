#include <my_robot_core/magneto_core/magneto_wbc_controller/containers/reference_generator_container.hpp>
#include <my_robot_core/magneto_core/magneto_wbc_controller/containers/wbc_spec_container.hpp>
#include <my_robot_core/magneto_core/magneto_wbc_controller/state_machines/transition.hpp>

Transition::Transition(const StateIdentifier state_identifier_in,
    RobotSystem* _robot,
    MagnetoWbcSpecContainer* ws_container, 
    MagnetoReferenceGeneratorContainer* rg_container,
    bool contact_start)
    : StateMachine(state_identifier_in, _robot), b_contact_start_(contact_start) {
  my_utils::pretty_constructor(2, "StateMachine: Transition");

  // Set Pointer to Control Architecture
  ws_container_ = ws_container;
  rg_container_ = rg_container;

  // Get State Provider
  sp_ = MagnetoStateProvider::getStateProvider(robot_);
}

Transition::~Transition() {}

void Transition::firstVisit() {
  std::cout<<"-------------------------------" <<std::endl;
  std::cout << "[contact transition] Start : " << b_contact_start_ << std::endl;

  ctrl_start_time_ = sp_->curr_time;
  ctrl_duration_ = trans_duration_;
  ctrl_end_time_ = ctrl_start_time_ + ctrl_duration_;

  // ---------------------------------------
  //      TASK - SET TRAJECTORY
  // ---------------------------------------
  // -- set current motion param
  MotionCommand mc_curr_ = sp_->curr_motion_command;

  moving_foot_idx_ = mc_curr_.get_moving_foot();

  // --set com traj
  rg_container_->com_trajectory_manager_
            ->setCoMTrajectory(ctrl_start_time_, 
                              ctrl_duration_);

  // -- set base ori traj
  rg_container_->base_ori_trajectory_manager_
            ->setBaseOriTrajectory(ctrl_start_time_,
                                  ctrl_duration_);

  // -- set joint traj
  rg_container_->joint_trajectory_manager_
            ->setJointTrajectory(ctrl_start_time_,
                                ctrl_duration_);

  // -- set task_list in taf with hierachy
  ws_container_->clear_task_list();
  ws_container_->add_task_list(ws_container_->com_task_);
  ws_container_->add_task_list(ws_container_->base_ori_task_);
  ws_container_->add_task_list(ws_container_->joint_task_);


  // ---------------------------------------
  //      QP PARAM - SET MAGNETISM
  // ---------------------------------------
  // todo later : implement it with magnetic manager
  // simulation/real environment magnetism
  if(b_contact_start_){
    ws_container_->set_magnetism(-1);
    ws_container_->set_residual_magnetic_force(-1);
    ws_container_->set_contact_magnetic_force(-1);
    
  }  else {
    ws_container_->set_magnetism(moving_foot_idx_); // off-magnetism on moving foot
    ws_container_->set_residual_magnetic_force(moving_foot_idx_);
    ws_container_->set_contact_magnetic_force(-1); // build full contact dim F_magnetic_    
  }


  // ---------------------------------------
  //      CONTACT LIST
  // --------------------------------------- 
  ws_container_->set_contact_list(-1);  // contain full contact

  // ---------------------------------------
  //      QP PARAM - SET WEIGHT
  // --------------------------------------- 
  // ws_container_->W_qddot_ : will be always same 
  Eigen::VectorXd W_xddot_swing_, W_rf_swing_; 
  Eigen::VectorXd W_xddot_full_contact, W_rf_full_contact; 
  ws_container_->compute_weight_param(moving_foot_idx_, 
                                ws_container_->W_xddot_contact_,
                                ws_container_->W_xddot_nocontact_,
                                W_xddot_swing_);
  ws_container_->compute_weight_param(-1, 
                                ws_container_->W_xddot_contact_,
                                ws_container_->W_xddot_nocontact_,
                                W_xddot_full_contact);                                  
  ws_container_->compute_weight_param(moving_foot_idx_, 
                                ws_container_->W_rf_contact_,
                                ws_container_->W_rf_nocontact_,
                                W_rf_swing_);                                
  ws_container_->compute_weight_param(-1, 
                                ws_container_->W_rf_contact_,
                                ws_container_->W_rf_nocontact_,
                                W_rf_full_contact); 


  // rg_container_->QPweight_qddot_manager_->setTransition(ctrl_start_time_,ctrl_duration_, )
  if(b_contact_start_) {
    // moving foot : nocontact -> contact
    rg_container_->max_normal_force_manager_
              ->setTransition(ctrl_start_time_, ctrl_duration_,
                                  ws_container_->max_rf_z_nocontact_,
                                  ws_container_->max_rf_z_contact_);
    rg_container_->QPweight_xddot_manager_
              ->setTransition(ctrl_start_time_, ctrl_duration_, 
                                      W_xddot_swing_, W_xddot_full_contact);
    rg_container_->QPweight_reactforce_manager_
              ->setTransition(ctrl_start_time_, ctrl_duration_, 
                                      W_rf_swing_, W_rf_full_contact);
    rg_container_->weight_residualforce_manager_
              ->setTransition(ctrl_start_time_, ctrl_duration_, 
                                      1.0, 0.0);
  } else {
    // moving foot : contact -> nocontact
    rg_container_->max_normal_force_manager_
              ->setTransition(ctrl_start_time_, ctrl_duration_,
                                  ws_container_->max_rf_z_contact_, 
                                  ws_container_->max_rf_z_nocontact_);
    rg_container_->QPweight_xddot_manager_
              ->setTransition(ctrl_start_time_, ctrl_duration_, 
                                      W_xddot_full_contact, W_xddot_swing_);
    rg_container_->QPweight_reactforce_manager_
              ->setTransition(ctrl_start_time_, ctrl_duration_, 
                                      W_rf_full_contact, W_rf_swing_);
    rg_container_->weight_residualforce_manager_
              ->setTransition(ctrl_start_time_, ctrl_duration_, 
                                      0.0, 1.0);
  }
}

void Transition::_taskUpdate() {
  // rg_container_->com_trajectory_manager_->updateCoMTrajectory(sp_->curr_time);
  rg_container_->com_trajectory_manager_->updateTask(sp_->curr_time,
                                  ws_container_->com_task_);
  // rg_container_->base_ori_trajectory_manager_->updateBaseOriTrajectory(sp_->curr_time);
  rg_container_->base_ori_trajectory_manager_->updateTask(sp_->curr_time,
                                  ws_container_->base_ori_task_);
  // rg_container_->joint_trajectory_manager_->updateJointTrajectory(sp_->curr_time);
  rg_container_->joint_trajectory_manager_->updateTask(sp_->curr_time,
                                  ws_container_->joint_task_);
}

void Transition::_weightUpdate() {
  // change in weight
  // rg_container_->QPweight_qddot_manager_
  //           ->updateTransition(sp_->curr_time, 
  //                           ws_container_->W_qddot_);
  rg_container_->QPweight_xddot_manager_
            ->updateTransition(sp_->curr_time, 
                            ws_container_->W_xddot_);
  rg_container_->QPweight_reactforce_manager_
            ->updateTransition(sp_->curr_time, 
                            ws_container_->W_rf_);
  // rg_container_->weight_residualforce_manager_
  //           ->updateTransition(sp_->curr_time,
  //                               ws_container_->w_res_);
  ws_container_->w_res_ = 0.0;

  // change in normal force in contactSpec
  rg_container_->max_normal_force_manager_
            ->updateTransition(sp_->curr_time, 
                                  ws_container_->max_rf_z_trans_);
  ws_container_->set_maxfz_contact(moving_foot_idx_, 
                              ws_container_->max_rf_z_contact_,
                              ws_container_->max_rf_z_trans_);
}


void Transition::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  _taskUpdate();
  _weightUpdate();
}

void Transition::lastVisit() {}

bool Transition::endOfState() {
  // Also check if footstep list is non-zero
  if ( state_machine_time_ > ctrl_duration_) {
    std::cout << "[contact transition] End : " << b_contact_start_ << std::endl;
    return true;
  }
  return false;
}

void Transition::initialization(const YAML::Node& node) {

  try {
    my_utils::readParameter(node, "transition_duration", trans_duration_);

  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }  
}
