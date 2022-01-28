#pragma once

#include <vector>
#include <mutex> 

#include <my_robot_core/control_architecture.hpp>
#include <my_robot_core/magneto_core/magneto_definition.hpp>
#include <my_robot_core/magneto_core/magneto_wbc_controller/state_machines/state_machine_set.hpp>
#include <my_robot_core/magneto_core/magneto_wbc_controller/magneto_wbmc.hpp>
#include <my_robot_core/magneto_core/magneto_wbc_controller/magneto_wbrmc.hpp>
#include <my_robot_core/magneto_core/magneto_wbc_controller/containers/wbc_spec_container.hpp>
#include <my_robot_core/magneto_core/magneto_wbc_controller/containers/reference_generator_container.hpp>
#include <my_robot_core/magneto_core/magneto_planner/magneto_planner_set.hpp>

namespace CONTROLLER_TYPES {
constexpr int WBMC = 0;
constexpr int WBRMC = 1;
}; // namespace CONTROLLER_TYPES

namespace MAGNETO_STATES {
constexpr int INITIALIZE = 0;
constexpr int BALANCE = 1; // DEFAULT
constexpr int SWING_START_TRANS = 2;
constexpr int SWING = 3;
constexpr int SWING_END_TRANS = 4;
};  // namespace MAGNETO_STATES

class MagnetoStateProvider;

class MagnetoWbmcControlArchitecture : public ControlArchitecture {
 public:
  MagnetoWbmcControlArchitecture(RobotSystem* _robot);
  virtual ~MagnetoWbmcControlArchitecture();
  virtual void ControlArchitectureInitialization();
  virtual void getCommand(void* _command);
  void saveData();
  void getIVDCommand(void* _command);
  void smoothing_torque(void* _cmd);


  // states_sequence_ : deque of pair<StateIdentifier, motion_command*>
  // int get_num_states();  
  // void get_next_state(StateIdentifier &_state);
  // void add_next_state(StateIdentifier _st_id, int _mt_id,
  //                     const MotionCommand &_motion_command);
  // void add_next_state(STMCommand _stm_cmd);
  

  // initialize parameters
  void _ReadParameters();
  void _InitializeParameters();
  bool b_state_first_visit_;

 protected:
  MagnetoStateProvider* sp_;
  
  YAML::Node cfg_;

  int controller_type_;

 public:
  // Task and Force Containers
  MagnetoWbcSpecContainer* ws_container_;
  MagnetoReferenceGeneratorContainer* rg_container_;

  // Controller Object
  MagnetoWBMC* wbc_controller;
  // MagnetoWBRMC* wbc_controller;

  private:
    Eigen::VectorXd tau_min_;
    Eigen::VectorXd tau_max_;

};
