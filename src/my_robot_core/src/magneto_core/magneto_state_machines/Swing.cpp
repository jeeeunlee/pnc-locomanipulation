#include <my_robot_core/magneto_core/magneto_control_architecture/magneto_control_architecture.hpp>
#include <my_robot_core/magneto_core/magneto_state_machines/Swing.hpp>

Swing::Swing(const StateIdentifier state_identifier_in,
    MagnetoControlArchitecture* _ctrl_arch, RobotSystem* _robot)
    : StateMachine(state_identifier_in, _robot) {
  my_utils::pretty_constructor(2, "StateMachine: SWING");

  // Set Pointer to Control Architecture
  ctrl_arch_ = ((MagnetoControlArchitecture*)_ctrl_arch);
  taf_container_ = ctrl_arch_->taf_container_;
  // Get State Provider
  sp_ = MagnetoStateProvider::getStateProvider(robot_);
}

Swing::~Swing() {}

void Swing::firstVisit() {
  std::cout<<"-------------------------------" <<std::endl;
  std::cout << "[SWING] Start" << std::endl;

  ctrl_start_time_ = sp_->curr_time;

  // ---------------------------------------
  //      TASK - SET TRAJECTORY
  // ---------------------------------------
  // -- set current motion param
  MotionCommand mc_curr_ = ctrl_arch_->get_motion_command();

  // --moving foot setting
  // _set_moving_foot_frame();

  // -- set foot traj
  ctrl_arch_->foot_trajectory_manager_
            ->setFootPosTrajectory(ctrl_start_time_, &mc_curr_);
  ctrl_duration_ = ctrl_arch_->foot_trajectory_manager_->getTrajDuration();
  ctrl_end_time_ = ctrl_arch_->foot_trajectory_manager_->getTrajEndTime();
  moving_foot_idx_ = ctrl_arch_->foot_trajectory_manager_->getMovingFootIdx();

  // --set com traj
  ctrl_arch_->com_trajectory_manager_
            ->setCoMTrajectory(ctrl_start_time_, ctrl_duration_);

  // -- set base ori traj
  ctrl_arch_->base_ori_trajectory_manager_
            ->setBaseOriTrajectory(ctrl_start_time_,
                                  ctrl_duration_);

  // -- set joint traj
  ctrl_arch_->joint_trajectory_manager_
            ->setJointTrajectory(ctrl_start_time_,
                                ctrl_duration_);

  // -- set task_list in taf with hierachy
  ctrl_arch_->taf_container_->clear_task_list();
  ctrl_arch_->taf_container_->add_task_list(
        ctrl_arch_->taf_container_->com_task_);
  ctrl_arch_->taf_container_->add_task_list(
        ctrl_arch_->taf_container_->base_ori_task_);
  ctrl_arch_->taf_container_->add_task_list(
        ctrl_arch_->taf_container_->get_foot_pos_task(moving_foot_idx_));
  ctrl_arch_->taf_container_->add_task_list(
        ctrl_arch_->taf_container_->get_foot_ori_task(moving_foot_idx_));
  ctrl_arch_->taf_container_->add_task_list(
        ctrl_arch_->taf_container_->joint_task_);


  // ---------------------------------------
  //      QP PARAM - SET MAGNETISM
  // ---------------------------------------
  // todo later : implement it with magnetic manager
  // simulation/real environment magnetism
  ctrl_arch_->taf_container_->set_magnetism(moving_foot_idx_);    
  ctrl_arch_->taf_container_->set_residual_magnetic_force(moving_foot_idx_);
  ctrl_arch_->taf_container_->set_contact_magnetic_force(moving_foot_idx_);
  ctrl_arch_->taf_container_->w_res_ = 1.0;

  // ---------------------------------------
  //      CONTACT LIST
  // --------------------------------------- 
  ctrl_arch_->taf_container_->set_contact_list(moving_foot_idx_);


  // ---------------------------------------
  //      QP PARAM - SET WEIGHT
  // ---------------------------------------  
  // ctrl_arch_->max_normal_force_manager_->
  // ctrl_arch_->QPweight_qddot_manager_->setQPWeightTrajectory(ctrl_start_time_,ctrl_duration_, )
  // ctrl_arch_->QPweight_xddot_manager_
  //           ->setQPWeightTrajectory(ctrl_start_time_,ctrl_duration_, )
  // ctrl_arch_->QPweight_reactforce_manager_
  //           ->setQPWeightTrajectory(ctrl_start_time_,ctrl_duration_, )

  ctrl_arch_->taf_container_->set_maxfz_contact(moving_foot_idx_);
  // ctrl_arch_->taf_container_->W_qddot_ : will be always same  
  ctrl_arch_->taf_container_->compute_weight_param(moving_foot_idx_, 
                                  ctrl_arch_->taf_container_->W_xddot_contact_,
                                  ctrl_arch_->taf_container_->W_xddot_nocontact_,
                                  ctrl_arch_->taf_container_->W_xddot_);
  ctrl_arch_->taf_container_->compute_weight_param(moving_foot_idx_, 
                                  ctrl_arch_->taf_container_->W_rf_contact_,
                                  ctrl_arch_->taf_container_->W_rf_nocontact_,
                                  ctrl_arch_->taf_container_->W_rf_);
}

void Swing::_taskUpdate() {
  // ctrl_arch_->foot_trajectory_manager_->updateFootPosTrajectory(sp_->curr_time);
  ctrl_arch_->foot_trajectory_manager_->updateTask(sp_->curr_time,
              ctrl_arch_->taf_container_->get_foot_pos_task(moving_foot_idx_),
              ctrl_arch_->taf_container_->get_foot_ori_task(moving_foot_idx_));

  // ctrl_arch_->com_trajectory_manager_->updateCoMTrajectory(sp_->curr_time);
  ctrl_arch_->com_trajectory_manager_->updateTask(sp_->curr_time,
                                  ctrl_arch_->taf_container_->com_task_);

  // ctrl_arch_->base_ori_trajectory_manager_->updateBaseOriTrajectory(sp_->curr_time);
  ctrl_arch_->base_ori_trajectory_manager_->updateTask(sp_->curr_time,
                                  ctrl_arch_->taf_container_->base_ori_task_);
  
  // ctrl_arch_->joint_trajectory_manager_->updateJointTrajectory(sp_->curr_time);
  ctrl_arch_->joint_trajectory_manager_->updateTask(sp_->curr_time,
                                  ctrl_arch_->taf_container_->joint_task_);
}

void Swing::_weightUpdate() {
  // no change in weight

  // ctrl_arch_->set_maxfz_contact(-1); // full contact (no uncontact foot)
  // ctrl_arch_->QPweight_qddot_manager_
  //           ->updateQPWeight(sp_->curr_time, 
  //                           ctrl_arch_->taf_container_->W_qddot_);
  // ctrl_arch_->QPweight_xddot_manager_
  //           ->updateQPWeight(sp_->curr_time, 
  //                           ctrl_arch_->taf_container_->W_xddot_);
  // ctrl_arch_->QPweight_reactforce_manager_
  //           ->updateQPWeight(sp_->curr_time, 
  //                           ctrl_arch_->taf_container_->W_rf_);
}

void Swing::_ResidualMagnetismUpdate() {
  double contact_distance(0.0);
  contact_distance = ctrl_arch_->foot_trajectory_manager_->getTrajHeight();
  // std::cout << "contact_distance =  " << contact_distance << std::endl;
  ctrl_arch_->taf_container_->set_residual_magnetic_force(moving_foot_idx_, contact_distance);
}

void Swing::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  _taskUpdate();
  _weightUpdate();
  _ResidualMagnetismUpdate();
}

void Swing::lastVisit() {}

bool Swing::endOfState() {
  // Also check if footstep list is non-zero
  if ( state_machine_time_ > ctrl_duration_) {
    return true;
  } else if (state_machine_time_ > 0.5*ctrl_duration_ 
            && ctrl_arch_->foot_trajectory_manager_->getTrajHeight() < 0.01 ){
    std::cout<<"@@@@@@@@@@@@ SWING CONTACT END @@@@@@@@@@@@@ t=" << state_machine_time_ << std::endl;
    return true;
  }
  return false;
}

StateIdentifier Swing::getNextState() {
  return MAGNETO_STATES::SWING_START_TRANS;
}

void Swing::initialization(const YAML::Node& node) {}
