#pragma once

// #include <my_pnc/MagnetoPnC/MagnetoDefinition.hpp>
// #include <my_pnc/MagnetoPnC/MagnetoInterface.hpp>
// #include <my_pnc/MagnetoPnC/MagnetoStateProvider.hpp>
// #include <my_pnc/MagnetoPnC/MagnetoTaskAndForceContainer/MagnetoTaskAndForceContainer.hpp>
// #include <my_wbc/JointIntegrator.hpp>
// #include <my_wbc/WBLC/KinWBC.hpp>
#include <my_wbc/WBMC/WBRMRC.hpp>
#include <my_pnc/MagnetoPnC/MagnetoCtrl/MagnetoMainController.hpp>

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
