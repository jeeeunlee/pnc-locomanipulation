#pragma once

#include </my_robot_core/magneto_core/magneto_planner/magneto_planner_set.hpp>
#include </my_robot_core/reference_generator/reference_generator_set.hpp>

class RobotSystem;

// Object which publicly contains all the tasks, contacts and reaction forces
class MagnetoReferenceGeneratorContainer {
 public:
  MagnetoReferenceGeneratorContainer(RobotSystem* _robot);
  ~MagnetoReferenceGeneratorContainer();

  RobotSystem* robot_;

  // Trajectory Managers
  FootPosTrajectoryManager* foot_trajectory_manager_;
  CoMTrajectoryManager* com_trajectory_manager_;
  JointTrajectoryManager* joint_trajectory_manager_;
  BaseOriTrajectoryManager* base_ori_trajectory_manager_;

  // QP weight / max force transition manager
  SmoothTransitionManager<double>* max_normal_force_manager_;
  SmoothTransitionManager<Eigen::VectorXd>* QPweight_qddot_manager_;
  SmoothTransitionManager<Eigen::VectorXd>* QPweight_xddot_manager_;
  SmoothTransitionManager<Eigen::VectorXd>* QPweight_reactforce_manager_;
  SmoothTransitionManager<double>* weight_residualforce_manager_;

  // Planner
  MagnetoGoalPlanner* goal_planner_;
  MagnetoTrajectoryManager* trajectory_planner_;

};