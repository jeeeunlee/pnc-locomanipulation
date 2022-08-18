#pragma once

#include <my_robot_core/user_command.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <vector>

struct POSE_DATA {
    Eigen::Vector3d pos = Eigen::VectorXd::Zero(3);
    Eigen::Quaternion<double> ori = Eigen::Quaternion<double>::Identity();
    bool is_baseframe = true;
    double swing_height = 0.0;
    
    void printMotionInfo(){      
      std::cout<<"pos : ("<< pos[0]<<"," <<pos[1]<<"," <<pos[2]   << "), " ;
      std::cout<<"ori : ("<< ori.w() <<"," << ori.x() <<"," << ori.y() <<"," << ori.z() << ")" ;
      std::cout<<"[frame="<<is_baseframe<<"]"<< std::endl;
      std::cout<<"swing_height = "<< swing_height <<std::endl;
    }
};

class MotionCommand :  public UserCommand{
  public:
    Eigen::VectorXd periods=Eigen::VectorXd::Zero(2); // tstart,tend
    POSE_DATA dpose;
  public:
    MotionCommand(){}
    ~MotionCommand(){}
    MotionCommand(const MotionCommand& mc){
      periods=mc.periods;
      dpose=mc.dpose;
    }
    void printMotionInfo(){
      dpose.printMotionInfo();
      std::cout<<"periods = "<< periods.transpose() <<std::endl;
    }
};

//----------------------------------

class ComMotionCommand :  public UserCommand{
  public:
    ComMotionCommand() {
      pa.setZero();
      pb.setZero();
      va.setZero();
      vb.setZero();
      motion_period = 0.0;
      is_acc_constant = false;
    };
    ComMotionCommand(const Eigen::Vector3d& _pa,
                    const Eigen::Vector3d& _va,
                    const Eigen::Vector3d& _pb,
                    const Eigen::Vector3d& _vb,
                    double _T) {
      pa = _pa;
      pb = _pb;
      va = _va;
      vb = _vb;
      motion_period = _T;
      is_acc_constant = false;
    }
    ComMotionCommand(const Eigen::Vector3d& _pa,
                    const Eigen::Vector3d& _va,
                    const Eigen::Vector3d& _acc,
                    double _T) {
      pa = _pa;
      va = _va;
      acc = _acc;
      motion_period = _T;
      is_acc_constant = true;
    }

    ~ComMotionCommand() {};

  public:
    Eigen::Vector3d pa;
    Eigen::Vector3d pb;
    Eigen::Vector3d va;
    Eigen::Vector3d vb;
    Eigen::Vector3d acc;
    double motion_period;
    bool is_acc_constant;

};
