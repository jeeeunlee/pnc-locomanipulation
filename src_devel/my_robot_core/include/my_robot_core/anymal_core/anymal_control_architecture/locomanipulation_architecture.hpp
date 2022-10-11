#pragma once

#include <vector>
#include <mutex> 

#include <my_robot_core/control_architecture.hpp>
#include <my_robot_core/anymal_core/anymal_definition.hpp>
#include <my_robot_core/anymal_core/anymal_command_api.hpp>

#include <my_robot_core/anymal_core/anymal_wbc_controller/state_machines/state_machine_set.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/anymal_wbc.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/containers/wbc_spec_container.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/containers/reference_generator_container.hpp>

#include <my_robot_core/anymal_core/anymal_logic_interrupt/locomanipulation_interrupt_logic.hpp>

class ANYmalStateProvider;

class ANYmalLocoManipulationControlArchitecture : public ControlArchitecture {
 public:
  ANYmalLocoManipulationControlArchitecture(RobotSystem* _robot);
  virtual ~ANYmalLocoManipulationControlArchitecture();
  virtual void ControlArchitectureInitialization();
  virtual void getCommand(void* _command);
  virtual void addState(StateIdentifier _state_id, void* _user_state_command, int state_type=-100);
  virtual void setMotionStartTime();

  void saveData();
  void getIVDCommand(void* _command);  

  MotionCommand user_cmd_;
    
  StateSequence<MotionCommand>* com_states_sequence_;
  StateMachine* com_state_machine;
  bool b_com_state_first_visit_;
  int com_state_;

  StateSequence<MotionCommand>* arm_states_sequence_;
  StateMachine* ee_state_machine;
  bool b_ee_state_first_visit_;  
  int arm_state_;

  std::array< StateSequence<MotionCommand>*, ANYmal::n_leg> feet_states_sequence_;
  std::array< std::map<StateIdentifier, StateMachine*>, ANYmal::n_leg> foot_state_machines_;
  std::array< bool, ANYmal::n_leg> b_foot_state_first_visit_;
  std::array< int, ANYmal::n_leg> foot_state_;
  
  // initialize parameters
  void _ReadParameters();
  void _InitializeParameters();
  
 protected:
  ANYmalStateProvider* sp_;   
  YAML::Node cfg_;

 public:
  // Task and Force Containers
  ANYmalWbcSpecContainer* ws_container_;
  ANYmalReferenceGeneratorContainer* rg_container_;

  // Controller Object
  ANYmalWBC* wbc_controller;

  private:
    Eigen::VectorXd tau_min_;
    Eigen::VectorXd tau_max_;
};


