#include <my_robot_core/anymal_core/anymal_wbc_controller/containers/reference_generator_container.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/containers/wbc_spec_container.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/state_machines/foot_support.hpp>

Support::Support(int _foot_idx, ANYmalReferenceGeneratorContainer* rg_container ) 
    : StateMachine(rg_container->robot_) {
  my_utils::pretty_constructor(2, "StateMachine: Foot Support (Balance)" + ANYmalFoot::Names[_foot_idx] );

  // Set Pointer to wbc spec / reference generator container
  ws_container_ = rg_container->ws_container_;
  rg_container_ = rg_container;

  foot_idx_= _foot_idx;
  foot_link_idx_ = ANYmalFoot::LinkIdx[_foot_idx];

  // Get State Provider
  sp_ = ANYmalStateProvider::getStateProvider(robot_);
}

Support::~Support() {}

void Support::firstVisit() {
  std::cout<<"-------------------------------" <<std::endl;
  std::cout << "[Support] Start - " << ANYmalFoot::Names[foot_idx_];// << std::endl;


  // -- set current motion param
  MotionCommand mc_curr_ = sp_->feet_motion_command[foot_idx_];
  
  ctrl_start_time_ = sp_->curr_time;  
  ctrl_end_time_ = sp_->motion_start_time + mc_curr_.periods[1];
  ctrl_duration_ = ctrl_end_time_ - ctrl_start_time_;
  std::cout<<" [ " << ctrl_start_time_<< " ~ " << ctrl_end_time_ << " ] "<<std::endl;
  
  // ---------------------------------------
  //      CONTACT LIST
  // --------------------------------------- 
  ws_container_->b_feet_contact_list_[foot_idx_]=true;
  ws_container_->feet_contacts_[foot_idx_]->setMaxFz(ws_container_->max_rf_z_contact_);

  // ---------------------------------------
  //      TASK - SET TRAJECTORY
  // --------------------------------------- 
  // no task
  ws_container_->delete_task_list(ANYMAL_TASK::LF_POS + foot_idx_);
  ws_container_->delete_task_list(ANYMAL_TASK::LF_ORI + foot_idx_);

  // ---------------------------------------
  //      QP PARAM - SET WEIGHT
  // ---------------------------------------  
  ws_container_->feet_weights_[foot_idx_]->setWeightRF(
    ws_container_->w_rf_, ws_container_->w_rf_z_contact_);
  ws_container_->feet_weights_[foot_idx_]->setWeightXddot(
    ws_container_->w_xddot_, ws_container_->w_xddot_z_contact_); 

}

void Support::_taskUpdate() {
}

void Support::_weightUpdate() {
  // no change in weight
}

void Support::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  _taskUpdate();
  _weightUpdate();
}

void Support::lastVisit() {}

bool Support::endOfState() {
  if ( state_machine_time_ > ctrl_duration_ && sp_->num_feet_state[foot_idx_]>0 ) { 
    std::cout << "[Support] End - " << ANYmalFoot::Names[foot_idx_] << std:: endl;
    return true;
  }
  return false;
}

void Support::initialization(const YAML::Node& node) {}
