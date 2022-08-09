#include <my_robot_core/anymal_core/anymal_wbc_controller/containers/reference_generator_container.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/containers/wbc_spec_container.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/state_machines/full_support.hpp>

FullSupport::FullSupport(const StateIdentifier state_identifier_in,
    ANYmalReferenceGeneratorContainer* rg_container) 
    : StateMachine(state_identifier_in, rg_container->robot_) {
  my_utils::pretty_constructor(2, "StateMachine: Full Support (Balance)");

  // Set Pointer to wbc spec / reference generator container
  ws_container_ = rg_container->ws_container_;
  rg_container_ = rg_container;

  // Get State Provider
  sp_ = ANYmalStateProvider::getStateProvider(robot_);
}

FullSupport::~FullSupport() {}

void FullSupport::firstVisit() {
  std::cout<<"-------------------------------" <<std::endl;
  std::cout << "[Full Support Balance] Start" << std::endl;

  ctrl_start_time_ = sp_->curr_time;
  // -- set current motion param
  MotionCommand mc_curr_ = sp_->curr_motion_command;

  // ---------------------------------------
  //      Planning
  // ---------------------------------------
  // robot goal configuration
  Eigen::VectorXd q_init =  robot_->getQ();
  Eigen::VectorXd q_goal =  robot_->getQ();
  Eigen::Vector3d pcom = robot_->getCoMPosition();
  Eigen::Vector3d pc_goal = robot_->getCoMPosition();  

  // CoM planner
  
  Eigen::Vector3d com_dpos;
  Eigen::VectorXd periods;  
  ComMotionCommand mc_com;
  if(mc_curr_.foot_motion_given) {
    SWING_DATA mdtmp;  
    mc_curr_.get_foot_motion(mdtmp);
    com_dpos = mdtmp.dpose.pos*0.25;
    if(mdtmp.dpose.is_baseframe){
        Eigen::MatrixXd Rwb = robot_->getBodyNodeIsometry(ANYmalBodyNode::base).linear();
        com_dpos = Rwb*com_dpos;
    }
    pc_goal = pcom + com_dpos;
    rg_container_->com_sequence_planner_->planCentroidalMotion(pc_goal,
                                        mc_curr_,
                                        ws_container_->feet_contacts_);    
    mc_com = rg_container_->com_sequence_planner_->getFullSupportCoMCmd();
  }else if(mc_curr_.com_motion_given) {
    POSE_DATA mdtmp;  
    mc_curr_.get_com_motion(mdtmp);
    com_dpos = mdtmp.pos;
    if(mdtmp.is_baseframe){
        Eigen::MatrixXd Rwb = robot_->getBodyNodeIsometry(ANYmalBodyNode::base).linear();
        com_dpos = Rwb*com_dpos;
    }
    pc_goal = pcom + com_dpos;
    double period = mc_curr_.motion_periods(0);
    Eigen::Vector3d zero_vel(0.0, 0.0, 0.0);
    Eigen::Vector3d pa = robot_ ->getCoMPosition(); 
    mc_com = ComMotionCommand( pa, zero_vel, pc_goal, zero_vel, period );
  }else{ // nothing given : zero step stability
    Eigen::Vector3d zero3d(0.0, 0.0, 0.0);
    Eigen::Vector3d pa = robot_ ->getCoMPosition(); 
    mc_com = ComMotionCommand( pa, zero3d, zero3d, 0.1 );
  }

  sp_->com_pos_init = pcom;
  sp_->com_pos_target = pc_goal;
  my_utils::pretty_print(pcom, std::cout, "pc_init");
  my_utils::pretty_print(pc_goal, std::cout, "pc_goal");
  my_utils::pretty_print(q_init, std::cout, "q_init");
  my_utils::pretty_print(q_goal, std::cout, "q_goal");
  
  // ---------------------------------------
  //      CONTACT LIST
  // --------------------------------------- 
  ws_container_->set_contact_list(-1);
  ws_container_->set_contact_maxfz();

  // ---------------------------------------
  //      TASK - SET TRAJECTORY
  // ---------------------------------------
  mc_curr_.printMotionInfo();
  // --set com traj
  rg_container_->com_trajectory_manager_
               ->setCoMTrajectory(ctrl_start_time_, mc_com);
  ctrl_duration_ = rg_container_->com_trajectory_manager_->getTrajDuration();
  ctrl_end_time_ = rg_container_->com_trajectory_manager_->getTrajEndTime();
  // -- set base ori traj
  rg_container_->base_ori_trajectory_manager_
               ->setBaseOriTrajectory(ctrl_start_time_, ctrl_duration_);
  // -- set EE pos/ori traj
  rg_container_->ee_trajectory_manager_
                ->setEETrajectory(ctrl_start_time_, ctrl_duration_);

  // -- set joint traj
  rg_container_->joint_trajectory_manager_
            ->setJointTrajectory(ctrl_start_time_,
                                ctrl_duration_,
                                q_goal);
 
  // -- set task_list in taf with hierachy
  ws_container_->clear_task_list();
  ws_container_->add_task_list(ws_container_->com_task_);
  ws_container_->add_task_list(ws_container_->base_ori_task_);
  ws_container_->add_task_list(ws_container_->ee_pos_task_);
  ws_container_->add_task_list(ws_container_->ee_ori_task_);
  ws_container_->add_task_list(ws_container_->joint_task_);

  // ---------------------------------------
  //      QP PARAM - SET WEIGHT
  // ---------------------------------------  
  
  // ws_container_->W_qddot_ : will be always same  
  ws_container_->set_contact_weight_param();

}

void FullSupport::_taskUpdate() {
  // rg_container_->com_trajectory_manager_->updateCoMTrajectory(sp_->curr_time);
  rg_container_->com_trajectory_manager_->updateTask(sp_->curr_time,
                                      ws_container_->com_task_);

  // rg_container_->base_ori_trajectory_manager_->updateBaseOriTrajectory(sp_->curr_time);
  rg_container_->base_ori_trajectory_manager_->updateTask(sp_->curr_time,
                                      ws_container_->base_ori_task_);

  rg_container_->ee_trajectory_manager_->updateTask(sp_->curr_time,
                                      ws_container_->ee_pos_task_, 
                                      ws_container_->ee_ori_task_);
  
  // rg_container_->joint_trajectory_manager_->updateJointTrajectory(sp_->curr_time);
  rg_container_->joint_trajectory_manager_->updateTask(sp_->curr_time,
                                      ws_container_->joint_task_);
}

void FullSupport::_weightUpdate() {
  // no change in weight
  ws_container_->set_contact_weight_param();
  ws_container_->set_contact_maxfz();
}

void FullSupport::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  _taskUpdate();
  _weightUpdate();
}

void FullSupport::lastVisit() {}

bool FullSupport::endOfState() {
  // Also check if footstep list is non-zero
  // std::cout<<"state_machine_time_ = "<<state_machine_time_<<", ctrl_duration_ = " << ctrl_duration_;
  // std::cout<<", sp_->num_state = "<< sp_->num_state<< std::endl;
  if ( state_machine_time_ > ctrl_duration_ && sp_->num_state > 0) {
    std::cout << "[Full Support Balance] End" << std:: endl;
    return true;
  }
  return false;
}

void FullSupport::initialization(const YAML::Node& node) {}
