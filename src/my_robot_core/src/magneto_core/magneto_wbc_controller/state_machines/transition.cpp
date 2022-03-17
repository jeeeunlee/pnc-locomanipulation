#include <my_robot_core/magneto_core/magneto_wbc_controller/containers/reference_generator_container.hpp>
#include <my_robot_core/magneto_core/magneto_wbc_controller/containers/wbc_spec_container.hpp>
#include <my_robot_core/magneto_core/magneto_wbc_controller/state_machines/transition.hpp>

Transition::Transition(const StateIdentifier state_identifier_in, 
    MagnetoReferenceGeneratorContainer* rg_container,
    bool contact_start): b_contact_start_(contact_start),
    StateMachine(state_identifier_in, rg_container->robot_) {
  my_utils::pretty_constructor(2, "StateMachine: Transition");

  // Set Pointer to Control Architecture
  ws_container_ = rg_container->ws_container_;
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
  //      CONTACT LIST
  // --------------------------------------- 
  ws_container_->set_contact_list(-1);  // contain full contact

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
    ws_container_->set_foot_magnet_off(-1);
    ws_container_->set_magnet_distance(-1, 0.0);   
  }  else {
    ws_container_->set_foot_magnet_off(moving_foot_idx_); // off-magnetism on moving foot
    ws_container_->set_magnet_distance(moving_foot_idx_, 0.0);
  }



  // ---------------------------------------
  //      QP PARAM - SET WEIGHT
  // --------------------------------------- 
  // ws_container_->W_qddot_ : will be always same 
 
  if(b_contact_start_) {
    // moving foot : nocontact -> contact
    rg_container_->max_normal_force_manager_->setTransition(ctrl_start_time_,
                                      ctrl_duration_,
                                      ws_container_->max_rf_z_nocontact_,
                                      ws_container_->max_rf_z_contact_);
    rg_container_->W_xddot_manager_->setTransition(ctrl_start_time_, 
                                      ctrl_duration_, 
                                      ws_container_->w_xddot_z_nocontact_, 
                                      ws_container_->w_xddot_z_contact_);
    rg_container_->W_rf_manager_->setTransition(ctrl_start_time_, 
                                      ctrl_duration_, 
                                      ws_container_->w_rf_z_nocontact_, 
                                      ws_container_->w_rf_z_contact_);
  } else {
    // moving foot : contact -> nocontact
    rg_container_->max_normal_force_manager_ ->setTransition(ctrl_start_time_,
                                      ctrl_duration_,
                                      ws_container_->max_rf_z_contact_, 
                                      ws_container_->max_rf_z_nocontact_);
    rg_container_->W_xddot_manager_->setTransition(ctrl_start_time_, 
                                      ctrl_duration_, 
                                      ws_container_->w_xddot_z_contact_, 
                                      ws_container_->w_xddot_z_nocontact_);
    rg_container_->W_rf_manager_->setTransition(ctrl_start_time_, 
                                      ctrl_duration_, 
                                      ws_container_->w_rf_z_contact_, 
                                      ws_container_->w_rf_z_nocontact_);
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
  rg_container_->W_xddot_manager_->updateTransition(sp_->curr_time);
  rg_container_->W_rf_manager_->updateTransition(sp_->curr_time);
  // change in normal force in contactSpec
  rg_container_->max_normal_force_manager_->updateTransition(sp_->curr_time);

  ws_container_->set_contact_weight_param(moving_foot_idx_);
  ws_container_->set_contact_maxfz(moving_foot_idx_);
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

  rg_container_->com_sequence_planner_->setTransitionDuration(trans_duration_);
  
}
