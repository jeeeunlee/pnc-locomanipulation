#include <my_robot_core/anymal_core/anymal_wbc_controller/containers/reference_generator_container.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/containers/wbc_spec_container.hpp>
#include <my_robot_system/RobotSystem.hpp>

ANYmalReferenceGeneratorContainer::ANYmalReferenceGeneratorContainer(
  ANYmalWbcSpecContainer* _ws_container, RobotSystem* _robot){
  my_utils::pretty_constructor(2, "ANYmal Reference manager Container");
  robot_ = _robot;
  ws_container_ = _ws_container;

  // Initialize Trajectory managers
  foot_trajectory_manager_ = new FootPosTrajectoryManager(robot_);                    
  com_trajectory_manager_ = new CoMTrajectoryManager(robot_);
  joint_trajectory_manager_ = new JointTrajectoryManager(robot_);
  base_ori_trajectory_manager_ = new BaseOriTrajectoryManager(robot_);
  ee_trajectory_manager_ = new EETrajectoryManager(robot_);

  max_normal_force_manager_ = new SmoothTransitionManager(&ws_container_->max_rf_z_trans_);
  W_xddot_manager_ = new SmoothTransitionManager(&ws_container_->w_xddot_z_trans_);
  W_rf_manager_ = new SmoothTransitionManager(&ws_container_->w_rf_z_trans_);

  com_sequence_planner_ = new ANYmalCoMPlanner(robot_);

  
}

ANYmalReferenceGeneratorContainer::~ANYmalReferenceGeneratorContainer(){
    delete foot_trajectory_manager_;
    delete com_trajectory_manager_;
    delete joint_trajectory_manager_;
    delete base_ori_trajectory_manager_;

    delete max_normal_force_manager_;
    delete W_xddot_manager_;
    delete W_rf_manager_;

    delete com_sequence_planner_;
}