#include </my_robot_core/magneto_core/magneto_wbc_controller/containers/reference_generator_container.hpp>
#include </my_robot_core/magneto_core/magneto_wbc_controller/containers/wbc_spec_container.hpp>
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
  MotionCommand mc_curr_ = ctrl_arch_->get_motion_command();

  moving_foot_idx_ = mc_curr_.get_moving_foot();

  // --set com traj
  ctrl_arch_->com_trajectory_manager_
            ->setCoMTrajectory(ctrl_start_time_, 
                              ctrl_duration_);

  // -- set base ori traj
  ctrl_arch_->base_ori_trajectory_manager_
            ->setBaseOriTrajectory(ctrl_start_time_,
                                  ctrl_duration_);

  // -- set joint traj
  ctrl_arch_->joint_trajectory_manager_
            ->setJointTrajectory(ctrl_start_time_,
                                ctrl_duration_);

  // -- set task_list in taf with hierachy
  ctrl_arch_->ws_container_->clear_task_list();
  ctrl_arch_->ws_container_->add_task_list(ctrl_arch_->ws_container_->com_task_);
  ctrl_arch_->ws_container_->add_task_list(ctrl_arch_->ws_container_->base_ori_task_);
  ctrl_arch_->ws_container_->add_task_list(ctrl_arch_->ws_container_->joint_task_);


  // ---------------------------------------
  //      QP PARAM - SET MAGNETISM
  // ---------------------------------------
  // todo later : implement it with magnetic manager
  // simulation/real environment magnetism
  if(b_contact_start_){
    ctrl_arch_->ws_container_->set_magnetism(-1);
    ctrl_arch_->ws_container_->set_residual_magnetic_force(-1);
    ctrl_arch_->ws_container_->set_contact_magnetic_force(-1);
    
  }  else {
    ctrl_arch_->ws_container_->set_magnetism(moving_foot_idx_); // off-magnetism on moving foot
    ctrl_arch_->ws_container_->set_residual_magnetic_force(moving_foot_idx_);
    ctrl_arch_->ws_container_->set_contact_magnetic_force(-1); // build full contact dim F_magnetic_    
  }


  // ---------------------------------------
  //      CONTACT LIST
  // --------------------------------------- 
  ctrl_arch_->ws_container_->set_contact_list(-1);  // contain full contact

  // ---------------------------------------
  //      QP PARAM - SET WEIGHT
  // --------------------------------------- 
  // ctrl_arch_->ws_container_->W_qddot_ : will be always same 
  Eigen::VectorXd W_xddot_swing_, W_rf_swing_; 
  Eigen::VectorXd W_xddot_full_contact, W_rf_full_contact; 
  ctrl_arch_->ws_container_->compute_weight_param(moving_foot_idx_, 
                                ctrl_arch_->ws_container_->W_xddot_contact_,
                                ctrl_arch_->ws_container_->W_xddot_nocontact_,
                                W_xddot_swing_);
  ctrl_arch_->ws_container_->compute_weight_param(-1, 
                                ctrl_arch_->ws_container_->W_xddot_contact_,
                                ctrl_arch_->ws_container_->W_xddot_nocontact_,
                                W_xddot_full_contact);                                  
  ctrl_arch_->ws_container_->compute_weight_param(moving_foot_idx_, 
                                ctrl_arch_->ws_container_->W_rf_contact_,
                                ctrl_arch_->ws_container_->W_rf_nocontact_,
                                W_rf_swing_);                                
  ctrl_arch_->ws_container_->compute_weight_param(-1, 
                                ctrl_arch_->ws_container_->W_rf_contact_,
                                ctrl_arch_->ws_container_->W_rf_nocontact_,
                                W_rf_full_contact); 


  // ctrl_arch_->QPweight_qddot_manager_->setQPWeightTrajectory(ctrl_start_time_,ctrl_duration_, )
  if(b_contact_start_) {
    // moving foot : nocontact -> contact
    ctrl_arch_->max_normal_force_manager_
              ->setMaxNormalForceTrajectory(ctrl_start_time_, ctrl_duration_,
                                  ctrl_arch_->ws_container_->max_rf_z_nocontact_,
                                  ctrl_arch_->ws_container_->max_rf_z_contact_);
    ctrl_arch_->QPweight_xddot_manager_
              ->setQPWeightTrajectory(ctrl_start_time_, ctrl_duration_, 
                                      W_xddot_swing_, W_xddot_full_contact);
    ctrl_arch_->QPweight_reactforce_manager_
              ->setQPWeightTrajectory(ctrl_start_time_, ctrl_duration_, 
                                      W_rf_swing_, W_rf_full_contact);
    ctrl_arch_->weight_residualforce_manager_
              ->setSingleWeightTrajectory(ctrl_start_time_, ctrl_duration_, 
                                      1.0, 0.0);
  } else {
    // moving foot : contact -> nocontact
    ctrl_arch_->max_normal_force_manager_
              ->setMaxNormalForceTrajectory(ctrl_start_time_, ctrl_duration_,
                                  ctrl_arch_->ws_container_->max_rf_z_contact_, 
                                  ctrl_arch_->ws_container_->max_rf_z_nocontact_);
    ctrl_arch_->QPweight_xddot_manager_
              ->setQPWeightTrajectory(ctrl_start_time_, ctrl_duration_, 
                                      W_xddot_full_contact, W_xddot_swing_);
    ctrl_arch_->QPweight_reactforce_manager_
              ->setQPWeightTrajectory(ctrl_start_time_, ctrl_duration_, 
                                      W_rf_full_contact, W_rf_swing_);
    ctrl_arch_->weight_residualforce_manager_
              ->setSingleWeightTrajectory(ctrl_start_time_, ctrl_duration_, 
                                      0.0, 1.0);
  }
}

void Transition::_taskUpdate() {
  // ctrl_arch_->com_trajectory_manager_->updateCoMTrajectory(sp_->curr_time);
  ctrl_arch_->com_trajectory_manager_->updateTask(sp_->curr_time,
                                  ctrl_arch_->ws_container_->com_task_);
  // ctrl_arch_->base_ori_trajectory_manager_->updateBaseOriTrajectory(sp_->curr_time);
  ctrl_arch_->base_ori_trajectory_manager_->updateTask(sp_->curr_time,
                                  ctrl_arch_->ws_container_->base_ori_task_);
  // ctrl_arch_->joint_trajectory_manager_->updateJointTrajectory(sp_->curr_time);
  ctrl_arch_->joint_trajectory_manager_->updateTask(sp_->curr_time,
                                  ctrl_arch_->ws_container_->joint_task_);
}

void Transition::_weightUpdate() {
  // change in weight
  // ctrl_arch_->QPweight_qddot_manager_
  //           ->updateQPWeight(sp_->curr_time, 
  //                           ctrl_arch_->ws_container_->W_qddot_);
  ctrl_arch_->QPweight_xddot_manager_
            ->updateQPWeight(sp_->curr_time, 
                            ctrl_arch_->ws_container_->W_xddot_);
  ctrl_arch_->QPweight_reactforce_manager_
            ->updateQPWeight(sp_->curr_time, 
                            ctrl_arch_->ws_container_->W_rf_);
  // ctrl_arch_->weight_residualforce_manager_
  //           ->updateSingleWeight(sp_->curr_time,
  //                               ctrl_arch_->ws_container_->w_res_);
  ctrl_arch_->ws_container_->w_res_ = 0.0;

  // change in normal force in contactSpec
  ctrl_arch_->max_normal_force_manager_
            ->updateMaxNormalForce(sp_->curr_time, 
                                  ctrl_arch_->ws_container_->max_rf_z_trans_);
  ctrl_arch_->ws_container_->set_maxfz_contact(moving_foot_idx_, 
                              ctrl_arch_->ws_container_->max_rf_z_contact_,
                              ctrl_arch_->ws_container_->max_rf_z_trans_);
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

StateIdentifier Transition::getNextState() {
  return MAGNETO_STATES::SWING_START_TRANS;
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
