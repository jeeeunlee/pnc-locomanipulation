#include </my_robot_core/magneto_core/magneto_wbc_controller/containers/reference_generator_container.hpp>
#include </my_robot_core/magneto_core/magneto_wbc_controller/containers/wbc_spec_container.hpp>
#include <my_robot_core/magneto_core/magneto_wbc_controller/state_machines/swing.hpp>

Swing::Swing(const StateIdentifier state_identifier_in,
    RobotSystem* _robot,
    MagnetoWbcSpecContainer* ws_container, 
    MagnetoReferenceGeneratorContainer* rg_container)
    : StateMachine(state_identifier_in, _robot) {
  my_utils::pretty_constructor(2, "StateMachine: SWING");

  // Set Pointer to Control Architecture
  ws_container_ = ws_container;
  rg_container_ = rg_container;

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
  rg_container->foot_trajectory_manager_
            ->setFootPosTrajectory(ctrl_start_time_, &mc_curr_);
  ctrl_duration_ = rg_container->foot_trajectory_manager_->getTrajDuration();
  ctrl_end_time_ = rg_container->foot_trajectory_manager_->getTrajEndTime();
  moving_foot_idx_ = rg_container->foot_trajectory_manager_->getMovingFootIdx();

  // --set com traj
  rg_container->com_trajectory_manager_
            ->setCoMTrajectory(ctrl_start_time_, ctrl_duration_);

  // -- set base ori traj
  rg_container->base_ori_trajectory_manager_
            ->setBaseOriTrajectory(ctrl_start_time_,
                                  ctrl_duration_);

  // -- set joint traj
  rg_container->joint_trajectory_manager_
            ->setJointTrajectory(ctrl_start_time_,
                                ctrl_duration_);

  // -- set task_list in taf with hierachy
  ws_container_->clear_task_list();
  ws_container_->add_task_list(
        ws_container_->com_task_);
  ws_container_->add_task_list(
        ws_container_->base_ori_task_);
  ws_container_->add_task_list(
        ws_container_->get_foot_pos_task(moving_foot_idx_));
  ws_container_->add_task_list(
        ws_container_->get_foot_ori_task(moving_foot_idx_));
  ws_container_->add_task_list(
        ws_container_->joint_task_);


  // ---------------------------------------
  //      QP PARAM - SET MAGNETISM
  // ---------------------------------------
  // todo later : implement it with magnetic manager
  // simulation/real environment magnetism
  ws_container_->set_magnetism(moving_foot_idx_);    
  ws_container_->set_residual_magnetic_force(moving_foot_idx_);
  ws_container_->set_contact_magnetic_force(moving_foot_idx_);
  ws_container_->w_res_ = 1.0;

  // ---------------------------------------
  //      CONTACT LIST
  // --------------------------------------- 
  ws_container_->set_contact_list(moving_foot_idx_);


  // ---------------------------------------
  //      QP PARAM - SET WEIGHT
  // ---------------------------------------  
  // rg_container->max_normal_force_manager_->
  // rg_container->QPweight_qddot_manager_->setTransition(ctrl_start_time_,ctrl_duration_, )
  // rg_container->QPweight_xddot_manager_
  //           ->setTransition(ctrl_start_time_,ctrl_duration_, )
  // rg_container->QPweight_reactforce_manager_
  //           ->setTransition(ctrl_start_time_,ctrl_duration_, )

  ws_container_->set_maxfz_contact(moving_foot_idx_);
  // ws_container_->W_qddot_ : will be always same  
  ws_container_->compute_weight_param(moving_foot_idx_, 
                                  ws_container_->W_xddot_contact_,
                                  ws_container_->W_xddot_nocontact_,
                                  ws_container_->W_xddot_);
  ws_container_->compute_weight_param(moving_foot_idx_, 
                                  ws_container_->W_rf_contact_,
                                  ws_container_->W_rf_nocontact_,
                                  ws_container_->W_rf_);
}

void Swing::_taskUpdate() {
  // rg_container->foot_trajectory_manager_->updateFootPosTrajectory(sp_->curr_time);
  rg_container->foot_trajectory_manager_->updateTask(sp_->curr_time,
              ws_container_->get_foot_pos_task(moving_foot_idx_),
              ws_container_->get_foot_ori_task(moving_foot_idx_));

  // rg_container->com_trajectory_manager_->updateCoMTrajectory(sp_->curr_time);
  rg_container->com_trajectory_manager_->updateTask(sp_->curr_time,
                                  ws_container_->com_task_);

  // rg_container->base_ori_trajectory_manager_->updateBaseOriTrajectory(sp_->curr_time);
  rg_container->base_ori_trajectory_manager_->updateTask(sp_->curr_time,
                                  ws_container_->base_ori_task_);
  
  // rg_container->joint_trajectory_manager_->updateJointTrajectory(sp_->curr_time);
  rg_container->joint_trajectory_manager_->updateTask(sp_->curr_time,
                                  ws_container_->joint_task_);
}

void Swing::_weightUpdate() {
  // no change in weight
}

void Swing::_ResidualMagnetismUpdate() {
  double contact_distance(0.0);
  contact_distance = rg_container->foot_trajectory_manager_->getTrajHeight();
  // std::cout << "contact_distance =  " << contact_distance << std::endl;
  ws_container_->set_residual_magnetic_force(moving_foot_idx_, contact_distance);
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
            && rg_container->foot_trajectory_manager_->getTrajHeight() < 0.01 ){
    std::cout<<"@@@@@@@@@@@@ SWING CONTACT END @@@@@@@@@@@@@ t=" << state_machine_time_ << std::endl;
    return true;
  }
  return false;
}

StateIdentifier Swing::getNextState() {
  return MAGNETO_STATES::SWING_START_TRANS;
}

void Swing::initialization(const YAML::Node& node) {}
