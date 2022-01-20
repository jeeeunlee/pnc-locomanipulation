#pragma once

// #include <my_robot_core/magneto_core/MagnetoDefinition.hpp>
// #include <my_robot_core/magneto_core/MagnetoInterface.hpp>
// #include <my_robot_core/magneto_core/MagnetoStateProvider.hpp>
// #include <my_robot_core/magneto_core/MagnetoTaskAndForceContainer/MagnetoTaskAndForceContainer.hpp>
// #include <my_wbc/JointIntegrator.hpp>
// #include <my_wbc/WBLC/KinWBC.hpp>
#include <my_wbc/WBMC/WBRMRC.hpp>
#include <my_robot_core/magneto_core/magneto_controller/MagnetoMainController.hpp>

class MagnetoResidualController: public MagnetoMainController {
 public:
  MagnetoResidualController(MagnetoTaskAndForceContainer* _taf_container,
                            RobotSystem* _robot);
  virtual ~MagnetoResidualController();

  virtual void getCommand(void* _cmd);
  virtual void ctrlInitialization(const YAML::Node& node);

 protected:
  //  Processing Step for first visit
  virtual void firstVisit();  

  // Redefine PreProcessing Command
  virtual void _PreProcessing_Command();

 private:
 // Controller Objects
  // WBRMC* wbrmc_;
  // WBRMC_ExtraData* wbrmc_param_;

  WBRMRC* wbrmc_;
  WBRMRC_ExtraData* wbrmc_param_;
  
  
};
