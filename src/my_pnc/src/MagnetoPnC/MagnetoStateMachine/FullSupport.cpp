#include <my_pnc/MagnetoPnC/MagnetoCtrlArchitecture/MagnetoCtrlArchitecture.hpp>
#include <my_pnc/MagnetoPnC/MagnetoStateMachine/FullSupport.hpp>

FullSupport::FullSupport(const StateIdentifier state_identifier_in,
    MagnetoControlArchitecture* _ctrl_arch, RobotSystem* _robot)
    : StateMachine(state_identifier_in, _robot) {
  my_utils::pretty_constructor(2, "StateMachine: Full Support (Balance)");

  // Set Pointer to Control Architecture
  ctrl_arch_ = ((MagnetoControlArchitecture*)_ctrl_arch);
  taf_container_ = ctrl_arch_->taf_container_;
  // Get State Provider
  sp_ = MagnetoStateProvider::getStateProvider(robot_);
}

FullSupport::~FullSupport() {}

void FullSupport::firstVisit() {
  std::cout<<"-------------------------------" <<std::endl;
  std::cout << "[Full Support Balance] Start" << std::endl;

  ctrl_start_time_ = sp_->curr_time;
  // -- set current motion param
  MotionCommand mc_curr_ = ctrl_arch_->get_motion_command();
  std::cout << ctrl_arch_->get_num_states() 
            << "states left!" <<std::endl;


  // ---------------------------------------
  //      Planning
  // ---------------------------------------
  Eigen::VectorXd q_goal; 
  ctrl_arch_->goal_planner_->computeGoal(mc_curr_);  
  ctrl_arch_->goal_planner_->getGoalConfiguration(q_goal);

  ctrl_arch_->trajectory_planner_->setMovingFoot(mc_curr_.get_moving_foot());
  ctrl_arch_->trajectory_planner_->compute(q_goal); 

  // ---------------------------------------
  //      TASK - SET TRAJECTORY
  // ---------------------------------------

  // --moving foot setting
  // _set_moving_foot_frame();

  // --set com traj
  ctrl_arch_->com_trajectory_manager_
            ->setCoMTrajectory(ctrl_start_time_, &mc_curr_);
  ctrl_duration_ = ctrl_arch_->com_trajectory_manager_->getTrajDuration();
  ctrl_end_time_ = ctrl_arch_->com_trajectory_manager_->getTrajEndTime();
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
  ctrl_arch_->taf_container_->add_task_list(ctrl_arch_->taf_container_->com_task_);
  ctrl_arch_->taf_container_->add_task_list(ctrl_arch_->taf_container_->base_ori_task_);
  ctrl_arch_->taf_container_->add_task_list(ctrl_arch_->taf_container_->joint_task_);


  // ---------------------------------------
  //      QP PARAM - SET MAGNETISM
  // ---------------------------------------
  // todo later : implement it with magnetic manager
  // simulation/real environment magnetism
  ctrl_arch_->taf_container_->set_magnetism(-1);  
  ctrl_arch_->taf_container_->set_residual_magnetic_force(-1);
  ctrl_arch_->taf_container_->set_contact_magnetic_force(-1);
  ctrl_arch_->taf_container_->w_res_ = 0.0;


  // ---------------------------------------
  //      CONTACT LIST
  // --------------------------------------- 
  ctrl_arch_->taf_container_->set_contact_list(-1);

  // ---------------------------------------
  //      QP PARAM - SET WEIGHT
  // ---------------------------------------  
  // ctrl_arch_->max_normal_force_manager_->
  // ctrl_arch_->QPweight_qddot_manager_->setQPWeightTrajectory(ctrl_start_time_,ctrl_duration_, )
  // ctrl_arch_->QPweight_xddot_manager_
  //           ->setQPWeightTrajectory(ctrl_start_time_,ctrl_duration_, )
  // ctrl_arch_->QPweight_reactforce_manager_
  //           ->setQPWeightTrajectory(ctrl_start_time_,ctrl_duration_, )

  ctrl_arch_->taf_container_->set_maxfz_contact(-1);
  // ctrl_arch_->taf_container_->W_qddot_ : will be always same  
  ctrl_arch_->taf_container_->compute_weight_param(-1, 
                      ctrl_arch_->taf_container_->W_xddot_contact_,
                      ctrl_arch_->taf_container_->W_xddot_nocontact_,
                      ctrl_arch_->taf_container_->W_xddot_);
  ctrl_arch_->taf_container_->compute_weight_param(-1, 
                      ctrl_arch_->taf_container_->W_rf_contact_,
                      ctrl_arch_->taf_container_->W_rf_nocontact_,
                      ctrl_arch_->taf_container_->W_rf_);

}

void FullSupport::_taskUpdate() {
  ctrl_arch_->com_trajectory_manager_->updateCoMTrajectory(sp_->curr_time);
  ctrl_arch_->com_trajectory_manager_->updateTask(
                                      ctrl_arch_->taf_container_->com_task_);

  ctrl_arch_->base_ori_trajectory_manager_->updateBaseOriTrajectory(sp_->curr_time);
  ctrl_arch_->base_ori_trajectory_manager_->updateTask(
                                      ctrl_arch_->taf_container_->base_ori_task_);
  
  ctrl_arch_->joint_trajectory_manager_->updateJointTrajectory(sp_->curr_time);
  ctrl_arch_->joint_trajectory_manager_->updateTask(
                                      ctrl_arch_->taf_container_->joint_task_);
}

void FullSupport::_weightUpdate() {
  // no change in weight
}

void FullSupport::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  _taskUpdate();
  _weightUpdate();
}

void FullSupport::lastVisit() {}

bool FullSupport::endOfState() {
  // Also check if footstep list is non-zero
  if ( state_machine_time_ > ctrl_duration_ && ctrl_arch_->get_num_states() > 0) {
    std::cout << "[Full Support Balance] End" << std:: endl;
    return true;
  }
  return false;
}

StateIdentifier FullSupport::getNextState() {
  return MAGNETO_STATES::SWING_START_TRANS;
}

void FullSupport::initialization(const YAML::Node& node) {}
