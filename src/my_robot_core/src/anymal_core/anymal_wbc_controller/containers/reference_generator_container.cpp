#include <my_robot_core/anymal_core/anymal_wbc_controller/containers/reference_generator_container.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/containers/wbc_spec_container.hpp>
#include <my_robot_system/RobotSystem.hpp>

ANYmalReferenceGeneratorContainer::ANYmalReferenceGeneratorContainer(
  ANYmalWbcSpecContainer* _ws_container, RobotSystem* _robot){
  my_utils::pretty_constructor(2, "ANYmal Reference manager Container");
  robot_ = _robot;
  ws_container_ = _ws_container;

  // Initialize Trajectory managers
  for(int foot_idx(0);foot_idx<ANYmal::n_leg;++foot_idx){
    foot_trajectory_managers_[foot_idx] = new FootTrajectoryManager(robot_, 
                  ws_container_->task_container_[ANYMAL_TASK::LF_POS+foot_idx],
                  ws_container_->task_container_[ANYMAL_TASK::LF_ORI+foot_idx],
                  foot_idx);
  }                  
  com_trajectory_manager_ = new CoMTrajectoryManager(robot_, 
                  ws_container_->task_container_[ANYMAL_TASK::COM]);

  joint_trajectory_manager_ = new JointTrajectoryManager(robot_,
                  ws_container_->task_container_[ANYMAL_TASK::JOINT_TASK]);

  base_ori_trajectory_manager_ = new BaseOriTrajectoryManager(robot_,
                  ws_container_->task_container_[ANYMAL_TASK::BASE_ORI]);

  ee_trajectory_manager_ = new EETrajectoryManager(robot_,
                    ws_container_->task_container_[ANYMAL_TASK::EE_POS],
                    ws_container_->task_container_[ANYMAL_TASK::EE_ORI]);

  max_normal_force_manager_ = new SmoothTransitionManager(&ws_container_->max_rf_z_trans_);
  w_xddot_manager_ = new SmoothTransitionManager(&ws_container_->w_xddot_z_trans_);
  w_rf_manager_ = new SmoothTransitionManager(&ws_container_->w_rf_z_trans_);

  // com_sequence_planner_ = new ANYmalCoMPlanner(robot_);

}

ANYmalReferenceGeneratorContainer::~ANYmalReferenceGeneratorContainer(){
    for(int i(0);i<ANYmal::n_leg;++i)
      delete foot_trajectory_managers_[i];
    delete com_trajectory_manager_;
    delete joint_trajectory_manager_;
    delete base_ori_trajectory_manager_;

    delete max_normal_force_manager_;
    delete w_xddot_manager_;
    delete w_rf_manager_;

    // delete com_sequence_planner_;
}