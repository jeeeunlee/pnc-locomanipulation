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

#include <my_robot_core/anymal_core/anymal_planner/anymal_planner_set.hpp>


class ANYmalStateProvider;

class ANYmalWblcControlArchitecture : public ControlArchitecture {
 public:
  ANYmalWblcControlArchitecture(RobotSystem* _robot);
  virtual ~ANYmalWblcControlArchitecture();
  virtual void ControlArchitectureInitialization();
  virtual void getCommand(void* _command);
  virtual void addState(StateIdentifier _state_id, void* _user_state_command);

  void saveData();
  void getIVDCommand(void* _command);

  StateSequence<MotionCommand>* states_sequence_;
  MotionCommand user_cmd_;
  
  // initialize parameters
  void _ReadParameters();
  void _InitializeParameters();
  bool b_state_first_visit_;

 protected:
  ANYmalStateProvider* sp_;
  
  YAML::Node cfg_;

  int controller_type_;

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


