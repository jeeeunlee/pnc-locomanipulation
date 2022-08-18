#include <my_robot_core/anymal_core/anymal_wbc_controller/containers/reference_generator_container.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/containers/wbc_spec_container.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/state_machines/manipulation.hpp>

Manipulation::Manipulation(ANYmalReferenceGeneratorContainer* rg_container) 
    : StateMachine(rg_container->robot_) {
  my_utils::pretty_constructor(2, "StateMachine: Manipulation");

  // Set Pointer to wbc spec / reference generator container
  ws_container_ = rg_container->ws_container_;
  rg_container_ = rg_container;

  // Get State Provider
  sp_ = ANYmalStateProvider::getStateProvider(robot_);
}

Manipulation::~Manipulation() {}

void Manipulation::firstVisit() {
  std::cout<<"-------------------------------" <<std::endl;
  std::cout << "[Manipulation] Start" << std::endl;

  // -- set current motion param
  MotionCommand mc_curr_ = sp_->ee_motion_command;
  // mc_curr_.printMotionInfo();

  ctrl_start_time_ = sp_->curr_time;  
  ctrl_end_time_ = sp_->motion_start_time + mc_curr_.periods[1];
  ctrl_duration_ = ctrl_end_time_ - ctrl_start_time_;

  // ---------------------------------------
  //      TASK - SET TRAJECTORY
  // ---------------------------------------  
  // -- set EE pos/ori traj
  rg_container_->ee_trajectory_manager_
      ->setEETrajectory(ctrl_start_time_, ctrl_end_time_, mc_curr_);

  // -- set task_list in taf with hierachy
  ws_container_->add_task_list(ANYMAL_TASK::EE_POS);
  ws_container_->add_task_list(ANYMAL_TASK::EE_ORI);

  // ---------------------------------------
  //      QP PARAM - SET WEIGHT
  // ---------------------------------------  
}

void Manipulation::_taskUpdate() {
  rg_container_->ee_trajectory_manager_->updateTask(sp_->curr_time);
}

void Manipulation::_weightUpdate() {
  // no change in weight
}

void Manipulation::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  _taskUpdate();
  _weightUpdate();
}

void Manipulation::lastVisit() {}

bool Manipulation::endOfState() {
  // Also check if footstep list is non-zero
  // std::cout<<"state_machine_time_ = "<<state_machine_time_<<", ctrl_duration_ = " << ctrl_duration_;
  // std::cout<<", sp_->num_state = "<< sp_->num_state<< std::endl;
  if ( state_machine_time_ > ctrl_duration_ && sp_->num_ee_state>0 ) {
    std::cout << "[Manipulation ] End" << std:: endl;
    return true;
  }
  return false;
}

void Manipulation::initialization(const YAML::Node& node) {}
