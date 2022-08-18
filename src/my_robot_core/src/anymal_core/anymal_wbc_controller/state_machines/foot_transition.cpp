#include <my_robot_core/anymal_core/anymal_wbc_controller/containers/reference_generator_container.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/containers/wbc_spec_container.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/state_machines/foot_transition.hpp>

Transition::Transition(int _foot_idx, 
    ANYmalReferenceGeneratorContainer* rg_container,
    bool contact_start): b_contact_start_(contact_start),
    StateMachine(rg_container->robot_) {
  my_utils::pretty_constructor(2, "FootStateMachine: Transition" + ANYmalFoot::Names[_foot_idx]);

  // Set Pointer to Control Architecture
  ws_container_ = rg_container->ws_container_;
  rg_container_ = rg_container;

  foot_idx_ = _foot_idx;
  foot_link_idx_ = ANYmalFoot::LinkIdx[_foot_idx];

  // Get State Provider
  sp_ = ANYmalStateProvider::getStateProvider(robot_);
}

Transition::~Transition() {}

void Transition::firstVisit() {
  std::cout<<"-------------------------------" <<std::endl;
  std::cout << "[contact transition] Start - " << ANYmalFoot::Names[foot_idx_] 
            << " contact:" << b_contact_start_;// << std::endl;

  MotionCommand mc_curr_ = sp_->feet_motion_command[foot_idx_];
  
  ctrl_start_time_ = sp_->curr_time;  
  ctrl_end_time_ = sp_->motion_start_time + mc_curr_.periods[1];
  ctrl_duration_ = ctrl_end_time_ - ctrl_start_time_;
  std::cout<<" [ " << ctrl_start_time_<< " ~ " << ctrl_end_time_ << " ] "<<std::endl;
  
  // ---------------------------------------
  //      CONTACT LIST
  // --------------------------------------- 
  ws_container_->b_feet_contact_list_[foot_idx_]=true;
  ws_container_->feet_contacts_[foot_idx_]
                ->setMaxFz(ws_container_->max_rf_z_trans_);

  // ---------------------------------------
  //      TASK - SET TRAJECTORY
  // ---------------------------------------
  // no task
  ws_container_->delete_task_list(ANYMAL_TASK::LF_POS + foot_idx_);
  ws_container_->delete_task_list(ANYMAL_TASK::LF_ORI + foot_idx_);

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
    rg_container_->w_xddot_manager_->setTransition(ctrl_start_time_, 
                                      ctrl_duration_, 
                                      ws_container_->w_xddot_z_nocontact_, 
                                      ws_container_->w_xddot_z_contact_);
    rg_container_->w_rf_manager_->setTransition(ctrl_start_time_, 
                                      ctrl_duration_, 
                                      ws_container_->w_rf_z_nocontact_, 
                                      ws_container_->w_rf_z_contact_);
  } else {
    // moving foot : contact -> nocontact
    rg_container_->max_normal_force_manager_ ->setTransition(ctrl_start_time_,
                                      ctrl_duration_,
                                      ws_container_->max_rf_z_contact_, 
                                      ws_container_->max_rf_z_nocontact_);
    rg_container_->w_xddot_manager_->setTransition(ctrl_start_time_, 
                                      ctrl_duration_, 
                                      ws_container_->w_xddot_z_contact_, 
                                      ws_container_->w_xddot_z_nocontact_);
    rg_container_->w_rf_manager_->setTransition(ctrl_start_time_, 
                                      ctrl_duration_, 
                                      ws_container_->w_rf_z_contact_, 
                                      ws_container_->w_rf_z_nocontact_);
  }
}

void Transition::_taskUpdate() {}

void Transition::_weightUpdate() {
  // change in weight
  rg_container_->w_xddot_manager_->updateTransition(sp_->curr_time);
  rg_container_->w_rf_manager_->updateTransition(sp_->curr_time);
  // change in normal force in contactSpec
  rg_container_->max_normal_force_manager_->updateTransition(sp_->curr_time);

  ws_container_->feet_weights_[foot_idx_]
                ->setWeightRF( ws_container_->w_rf_, 
                              ws_container_->w_rf_z_trans_);
  ws_container_->feet_weights_[foot_idx_]
                ->setWeightXddot(ws_container_->w_xddot_, 
                            ws_container_->w_xddot_z_trans_); 
  ws_container_->feet_contacts_[foot_idx_]
                ->setMaxFz(ws_container_->max_rf_z_trans_);
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
    std::cout << "[contact transition] End - " << ANYmalFoot::Names[foot_idx_] 
            << " contact" << b_contact_start_ << std::endl;
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

  // rg_container_->com_sequence_planner_->setTransitionDuration(trans_duration_);
  
}
