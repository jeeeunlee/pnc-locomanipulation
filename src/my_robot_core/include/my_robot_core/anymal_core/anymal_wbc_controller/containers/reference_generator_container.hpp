#pragma once

#include <my_robot_core/anymal_core/anymal_planner/anymal_planner_set.hpp>
#include <my_robot_core/reference_generator/reference_generator_set.hpp>

class RobotSystem;
class ANYmalWbcSpecContainer;
class MotionCommand;


// Object which publicly contains all the tasks, contacts and reaction forces
class ANYmalReferenceGeneratorContainer {
 public:
  ANYmalReferenceGeneratorContainer(ANYmalWbcSpecContainer* _ws_container, RobotSystem* _robot);
  ~ANYmalReferenceGeneratorContainer();

  RobotSystem* robot_;
  ANYmalWbcSpecContainer* ws_container_;

  // Trajectory Managers
  FootPosTrajectoryManager* foot_trajectory_manager_;
  CoMTrajectoryManager* com_trajectory_manager_;
  JointTrajectoryManager* joint_trajectory_manager_;
  BaseOriTrajectoryManager* base_ori_trajectory_manager_;
  EETrajectoryManager* ee_trajectory_manager_;

  // QP weight / max force transition manager
  SmoothTransitionManager* max_normal_force_manager_;
  SmoothTransitionManager* W_xddot_manager_;
  SmoothTransitionManager* W_rf_manager_;

  // Planner
  ANYmalCoMPlanner* com_sequence_planner_;


};