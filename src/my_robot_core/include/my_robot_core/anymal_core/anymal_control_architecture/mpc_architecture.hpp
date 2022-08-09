#pragma once

#include <vector>
#include <mutex> 

#include <my_robot_core/control_architecture.hpp>
#include <my_robot_core/anymal_core/anymal_definition.hpp>
#include <my_robot_core/anymal_core/anymal_command_api.hpp>

#include <my_robot_core/anymal_core/anymal_wbc_controller/state_machines/state_machine_set.hpp>
// #include <my_robot_core/anymal_core/anymal_wbc_controller/anymal_wbmc.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/anymal_wbc.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/containers/wbc_spec_container.hpp>
#include <my_robot_core/anymal_core/anymal_wbc_controller/containers/reference_generator_container.hpp>

#include <my_robot_core/anymal_core/anymal_planner/anymal_planner_set.hpp>
#include <my_robot_core/anymal_core/anymal_estimator/slip_observer.hpp>

class ANYmalStateProvider;

class ANYmalMpcControlArchitecture : public ControlArchitecture {
 public:
  ANYmalMpcControlArchitecture(RobotSystem* _robot);
  virtual ~ANYmalMpcControlArchitecture();
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
  bool b_env_param_updated_;

 protected:
  ANYmalStateProvider* sp_;   
  YAML::Node cfg_;

 public:
  // Task and Force Containers
  ANYmalWbcSpecContainer* ws_container_;
  ANYmalReferenceGeneratorContainer* rg_container_;

  // Controller Object
  ANYmalWBC* wbc_controller;

  // Observers
  SlipObserver* slip_ob_;
  SlipObserverData* slip_ob_data_;

  private:
    Eigen::VectorXd tau_min_;
    Eigen::VectorXd tau_max_;
};


