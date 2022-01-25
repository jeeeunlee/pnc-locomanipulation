#include <my_robot_core/magneto_core/magneto_wbc_controller/containers/reference_generator_container.hpp>
#include <my_robot_system/RobotSystem.hpp>

MagnetoReferenceGeneratorContainer::MagnetoReferenceGeneratorContainer(RobotSystem* _robot){
  robot_ = _robot;
  
  // Initialize Trajectory managers
  foot_trajectory_manager_ = new FootPosTrajectoryManager(robot_);                    
  com_trajectory_manager_ = new CoMTrajectoryManager(robot_);
  joint_trajectory_manager_ = new JointTrajectoryManager(robot_);
  base_ori_trajectory_manager_ = new BaseOriTrajectoryManager(robot_);

  max_normal_force_manager_ = new SmoothTransitionManager<double>();
  QPweight_qddot_manager_ = new SmoothTransitionManager<Eigen::VectorXd>();
  QPweight_xddot_manager_ = new SmoothTransitionManager<Eigen::VectorXd>();
  QPweight_reactforce_manager_ = new SmoothTransitionManager<Eigen::VectorXd>();
  weight_residualforce_manager_ = new SmoothTransitionManager<double>();

  goal_planner_ = new MagnetoGoalPlanner(robot_);
  trajectory_planner_ = new MagnetoTrajectoryManager(this);
}

MagnetoReferenceGeneratorContainer::~MagnetoReferenceGeneratorContainer(){
    delete foot_trajectory_manager_;
    delete com_trajectory_manager_;
    delete joint_trajectory_manager_;
    delete base_ori_trajectory_manager_;

    delete max_normal_force_manager_;
    delete QPweight_qddot_manager_;
    delete QPweight_qddot_manager_;
    delete QPweight_reactforce_manager_;
    delete weight_residualforce_manager_;

    delete goal_planner_;
    delete trajectory_planner_;
}