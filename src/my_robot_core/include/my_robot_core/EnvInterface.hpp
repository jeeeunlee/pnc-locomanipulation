#pragma once

#include <Eigen/Dense>
// #include <memory>
#include <my_robot_core/InterruptLogic.hpp>

class RobotSystem;
class ControlArchitecture;

class EnvInterface {
   protected:
    ControlArchitecture* control_architecture_;    
    RobotSystem* robot_;
    int count_;
    double running_time_;

   public:
    EnvInterface() {
        count_ = 0;
        running_time_ = 0.;
    }
    virtual ~EnvInterface(){};
    InterruptLogic* interrupt_;

    // Get Command through Test
    virtual void getCommand(void* _sensor_data, void* _command_data) = 0;
};
