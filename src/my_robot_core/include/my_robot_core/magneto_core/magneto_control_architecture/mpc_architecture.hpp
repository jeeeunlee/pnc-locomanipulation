#pragma once

#include <vector>
#include <mutex> 

#include <my_robot_core/ControlArchitecture.hpp>
#include <my_robot_core/magneto_core/MagnetoDefinition.hpp>
#include <my_robot_core/magneto_core/magneto_state_machines/StateMachineSet.hpp>
#include <my_robot_core/magneto_core/magneto_controller/MagnetoMainController.hpp>
#include <my_robot_core/magneto_core/magneto_controller/MagnetoResidualController.hpp>
#include <my_robot_core/magneto_core/MagnetoTaskAndForceContainer/MagnetoTaskAndForceContainer.hpp>
#include <my_robot_core/reference_generator/reference_generator_set.hpp>
#include <my_robot_core/magneto_core/magneto_planner/magneto_plannerSet.hpp>


typedef std::pair<StateIdentifier, MotionCommand> StatePair;
typedef std::deque<StatePair> StatePairSequence;

class MagnetoStateProvider;

namespace CONTROLLER_TYPES {
constexpr int WBMC = 0;
constexpr int WBRMC = 1;
}

namespace MAGNETO_STATES {
constexpr int INITIALIZE = 0;
// constexpr int STAND = 1;

constexpr int BALANCE = 1; // DEFAULT
constexpr int SWING_START_TRANS = 2;
constexpr int SWING = 3;
constexpr int SWING_END_TRANS = 4;
};  // namespace MAGNETO_STATES

class MagnetoControlArchitecture : public ControlArchitecture {
 public:
  MagnetoControlArchitecture(RobotSystem* _robot);
  virtual ~MagnetoControlArchitecture();
  virtual void ControlArchitectureInitialization();
  virtual void getCommand(void* _command);
  void saveData();
  void getIVDCommand(void* _command);
  void smoothing_torque(void* _cmd);


  // states_sequence_ : deque of pair<StateIdentifier, motion_command*>
  int get_num_states();  
  void get_next_state_pair(StateIdentifier &_state, 
                          MotionCommand &_motion_command);
  MotionCommand get_motion_command();
  MotionCommand get_next_motion_command();

  void add_next_state(StateIdentifier _state,
                      const MotionCommand &_motion_command);
  void add_next_state(StatePair _state_pair);
  

  // initialize parameters
  void _ReadParameters();
  void _InitializeParameters();
  bool b_state_first_visit_;

 protected:
  MagnetoStateProvider* sp_;

  // MotionCommandDeque* mc_list_;
  // std::deque<int> states_sequence_;

  // std::deque<pair<int, MotionCommand*>*>* state_sequences_;
  MotionCommand motion_command_;
  StatePairSequence states_sequence_;
  std::mutex states_sequence_mtx_;
  
  YAML::Node cfg_;

  int controller_type_;

 public:
  // Task and Force Containers
  MagnetoTaskAndForceContainer* taf_container_;

  // Controller Object
  MagnetoMainController* main_controller_;
  // MagnetoResidualController* main_controller_;

  // Trajectory Managers
  FootPosTrajectoryManager* foot_trajectory_manager_;
  CoMTrajectoryManager* com_trajectory_manager_;
  JointTrajectoryManager* joint_trajectory_manager_;
  BaseOriTrajectoryManager* base_ori_trajectory_manager_;

  // QP weight trajectory manager
  MaxNormalForceTrajectoryManager* max_normal_force_manager_;
  QPWeightTrajectoryManager* QPweight_qddot_manager_;
  QPWeightTrajectoryManager* QPweight_xddot_manager_;
  QPWeightTrajectoryManager* QPweight_reactforce_manager_;
  SingleWeightTrajectoryManager* weight_residualforce_manager_;

  MagnetoGoalPlanner* goal_planner_;
  // MagnetoReachabilityPlanner* reachability_planner_;
  MagnetoTrajectoryManager* trajectory_planner_;

  private:
    Eigen::VectorXd tau_min_;
    Eigen::VectorXd tau_max_;

};
