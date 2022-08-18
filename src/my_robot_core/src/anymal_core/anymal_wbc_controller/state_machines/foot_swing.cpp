#include <my_robot_core/anymal_core/anymal_wbc_controller/containers/reference_generator_container.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/containers/wbc_spec_container.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/state_machines/foot_swing.hpp>

Swing::Swing(int _foot_idx, ANYmalReferenceGeneratorContainer* rg_container )
    : StateMachine(rg_container->robot_) {
  my_utils::pretty_constructor(2, "FootStateMachine: SWING" + ANYmalFoot::Names[_foot_idx]);

  // Set Pointer to Control Architecture
  ws_container_ = rg_container->ws_container_;
  rg_container_ = rg_container;

  foot_idx_= _foot_idx;
  foot_link_idx_ = ANYmalFoot::LinkIdx[_foot_idx];

  // Get State Provider
  sp_ = ANYmalStateProvider::getStateProvider(robot_);
}

Swing::~Swing() {}

void Swing::firstVisit() {
  std::cout<<"-------------------------------" <<std::endl;
  std::cout << "[SWING] Start - " << ANYmalFoot::Names[foot_idx_];// << std::endl;

  MotionCommand mc_curr_ = sp_->feet_motion_command[foot_idx_];
  
  ctrl_start_time_ = sp_->curr_time;  
  ctrl_end_time_ = sp_->motion_start_time + mc_curr_.periods[1];
  ctrl_duration_ = ctrl_end_time_ - ctrl_start_time_;
  std::cout<<" [ " << ctrl_start_time_<< " ~ " << ctrl_end_time_ << " ] "<<std::endl;

  // ---------------------------------------
  //      CONTACT LIST
  // --------------------------------------- 
  ws_container_->b_feet_contact_list_[foot_idx_]=false;

  // ---------------------------------------
  //      TASK - SET TRAJECTORY
  // --------------------------------------- 
  // -- set foot traj
  rg_container_->foot_trajectory_managers_[foot_idx_]
            ->setFootPosTrajectory(ctrl_start_time_, 
                                  ctrl_end_time_,
                                  mc_curr_);
  // -- add task
  ws_container_->add_task_list(ANYMAL_TASK::LF_POS + foot_idx_);
  // ws_container_->add_task_list(ANYMAL_TASK::LF_ORI + foot_idx_);

  // ---------------------------------------
  //      QP PARAM - SET WEIGHT
  // ---------------------------------------  

}

void Swing::_taskUpdate() {
  // rg_container_->foot_trajectory_managers_->updateFootPosTrajectory(sp_->curr_time);
  rg_container_->foot_trajectory_managers_[foot_idx_]->updateTask(sp_->curr_time);
}

void Swing::_weightUpdate() {
  // no change in weight
}

void Swing::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  _taskUpdate();
  _weightUpdate();
}

void Swing::lastVisit() {}

bool Swing::endOfState() {
  // Also check if footstep list is non-zero
  if ( state_machine_time_ > ctrl_duration_) {
    std::cout << "[Swing] End - " << ANYmalFoot::Names[foot_idx_] << std:: endl;
    return true;
  } 
  return false;
}

void Swing::initialization(const YAML::Node& node) {}
