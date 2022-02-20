#pragma once

// 0: WBMC, 1: MCWBC
#define WBCMODE 0 
// #define WBCMODE 1

#include <vector>
#include <mutex> 

#include <my_robot_core/control_architecture.hpp>
#include <my_robot_core/magneto_core/magneto_definition.hpp>
#include <my_robot_core/magneto_core/magneto_command_api.hpp>

#include <my_robot_core/magneto_core/magneto_wbc_controller/state_machines/state_machine_set.hpp>
#include <my_robot_core/magneto_core/magneto_wbc_controller/magneto_wbmc.hpp>
#include <my_robot_core/magneto_core/magneto_wbc_controller/magneto_mcwbc.hpp>
#include <my_robot_core/magneto_core/magneto_wbc_controller/containers/wbc_spec_container.hpp>
#include <my_robot_core/magneto_core/magneto_wbc_controller/containers/reference_generator_container.hpp>

#include <my_robot_core/magneto_core/magneto_planner/magneto_planner_set.hpp>
#include <my_robot_core/magneto_core/magneto_estimator/slip_observer.hpp>

class MagnetoStateProvider;

class MagnetoMpcControlArchitecture : public ControlArchitecture {
 public:
  MagnetoMpcControlArchitecture(RobotSystem* _robot);
  virtual ~MagnetoMpcControlArchitecture();
  virtual void ControlArchitectureInitialization();
  virtual void getCommand(void* _command);
  virtual void addState(void* _user_state_command);

  void saveData();
  void getIVDCommand(void* _command);
  
  StateSequence<SimMotionCommand>* states_sequence_;
  SimMotionCommand user_cmd_;
  
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
  #if WBCMODE==0
  MagnetoWBMC* wbc_controller;
  #elif WBCMODE==1
  MagnetoMCWBC* wbc_controller;
  #endif

  // Observers
  SlipObserver* slip_ob_;
  SlipObserverData* slip_ob_data_;

  private:
    Eigen::VectorXd tau_min_;
    Eigen::VectorXd tau_max_;
};


