#pragma once

#include <my_robot_core/magneto_core/magneto_planner/magneto_planner_set.hpp>
#include <my_robot_core/reference_generator/reference_generator_set.hpp>

class RobotSystem;
class MagnetoWbcSpecContainer;
class MotionCommand;


// Object which publicly contains all the tasks, contacts and reaction forces
class MagnetoReferenceGeneratorContainer {
 public:
  MagnetoReferenceGeneratorContainer(MagnetoWbcSpecContainer* _ws_container, RobotSystem* _robot);
  ~MagnetoReferenceGeneratorContainer();

  RobotSystem* robot_;
  MagnetoWbcSpecContainer* ws_container_;

  // Trajectory Managers
  FootPosTrajectoryManager* foot_trajectory_manager_;
  CoMTrajectoryManager* com_trajectory_manager_;
  JointTrajectoryManager* joint_trajectory_manager_;
  BaseOriTrajectoryManager* base_ori_trajectory_manager_;

  // QP weight / max force transition manager
  SmoothTransitionManager* max_normal_force_manager_;
  SmoothTransitionManager* W_xddot_manager_;
  SmoothTransitionManager* W_rf_manager_;

  // Planner
  MagnetoGoalPlanner* goal_planner_;
  // MagnetoTrajectoryManager* trajectory_planner_;

};