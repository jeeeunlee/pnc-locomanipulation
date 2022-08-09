#pragma once

#include <my_robot_core/user_command.hpp>
#include <Eigen/Dense>
#include <iostream>

typedef int FOOT_IDX;

struct POSE_DATA {
    Eigen::Vector3d pos;
    Eigen::Quaternion<double> ori;
    bool is_baseframe;

    POSE_DATA() {
        is_baseframe = true;
        pos = Eigen::VectorXd::Zero(3);
        ori = Eigen::Quaternion<double>::Identity();
    }

    POSE_DATA(const Eigen::Vector3d& _pos, 
              const Eigen::Quaternion<double>& _ori,
              bool _is_baseframe=true)
              : pos(_pos),ori(_ori),is_baseframe(_is_baseframe){
    }

    POSE_DATA(const Eigen::VectorXd& _pos, 
              const Eigen::VectorXd& _ori,
              bool _is_baseframe=true) 
              : is_baseframe(_is_baseframe) {
        pos << _pos[0], _pos[1], _pos[2];
        ori = Eigen::Quaternion<double>
                (_ori[0], _ori[1], _ori[2], _ori[3]);
    }

    POSE_DATA(double x, double y, double z, 
              double w, double qx, double qy, double qz) {
        is_baseframe = true;
        pos << x,y,z;
        ori = Eigen::Quaternion<double>(w,qx,qy,qz);
    }
    void printMotionInfo(){      
      std::cout<<"pos : ("<< pos[0]<<"," <<pos[1]<<"," <<pos[2]   << "), " ;
      std::cout<<"ori : ("<< ori.w() <<"," << ori.x() <<"," << ori.y() <<"," << ori.z() << ")" ;
      std::cout<<"[frame="<<is_baseframe<<"]"<< std::endl;
    }
};

class SWING_DATA {
  public:
    SWING_DATA(){
      foot_idx = -1;
      swing_height = 0.0;
      dpose = POSE_DATA();
    };
    SWING_DATA(FOOT_IDX _foot_idx,
              double _swing_height,
              const POSE_DATA& _dpose){
      foot_idx = _foot_idx;
      swing_height = _swing_height;
      dpose = _dpose;
    };
    void printMotionInfo(){
      std::cout<<"["<<foot_idx<<"] : h="<< swing_height << " / ";
      dpose.printMotionInfo();      
    }
  public:
    FOOT_IDX foot_idx;
    double swing_height;
    POSE_DATA dpose;
};



// specific component
class MotionCommand : public UserCommand {
  public:
    MotionCommand()
    :com_motion_given(false), foot_motion_given(false) {
      foot_motion_data = SWING_DATA();
      com_motion_data = POSE_DATA();
      motion_periods = Eigen::VectorXd::Zero(1);
    }
    MotionCommand( const POSE_DATA& _com_motion_data,
                  double motion_period ):MotionCommand() {
      com_motion_given = true;
      com_motion_data = _com_motion_data;
      motion_periods = Eigen::VectorXd::Constant(1,motion_period);
    }
    MotionCommand(const SWING_DATA& _foot_motion_data,
                  const Eigen::VectorXd& _motion_periods ): MotionCommand() {
      foot_motion_given = true;
      foot_motion_data = _foot_motion_data;
      motion_periods = _motion_periods;
    }
    MotionCommand(const POSE_DATA& _com_motion_data,
                const SWING_DATA& _foot_motion_data,
                const Eigen::VectorXd& _motion_periods )
    :com_motion_given(true), foot_motion_given(true) {
      com_motion_data = _com_motion_data;
      foot_motion_data = _foot_motion_data;
      motion_periods = _motion_periods;
    }
    MotionCommand(const MotionCommand& _mc){
      com_motion_given = _mc.com_motion_given;
      foot_motion_given = _mc.foot_motion_given;      
      com_motion_data = _mc.com_motion_data;  
      foot_motion_data = _mc.foot_motion_data;
      motion_periods =  _mc.motion_periods;
    }
    ~MotionCommand() {};

  public:
    int get_moving_foot() {return foot_motion_data.foot_idx; }
    bool get_foot_motion(SWING_DATA &_motion_data){
      _motion_data = foot_motion_data;
      return foot_motion_given;
    }
    bool get_com_motion(POSE_DATA &_motion_data){
      _motion_data = com_motion_data;
      return com_motion_given;
    }
    Eigen::VectorXd get_motion_periods(){
      return motion_periods;
    }
    double get_swing_period(){
      if(motion_periods.size()==4){
        return motion_periods(2);
      }else if(motion_periods.size()==2){
        return motion_periods(1);
      }
      else if(motion_periods.size()==1){
        return motion_periods(0);
      }else{
        return 1.0;
      }
    }

    void printMotionInfo(){
      std::cout<<" -------------------------------- "<<std::endl;
      std::cout<<" MotionCommand"<<std::endl;
      std::cout<<"  * com motion : ";
      if(com_motion_given){ com_motion_data.printMotionInfo(); }
      else { std::cout<< "not given"<< std::endl; }
      std::cout<<"  * foot motion";
      if(foot_motion_given){ foot_motion_data.printMotionInfo(); }
      else { std::cout<< " : not given"<< std::endl; }
      std::cout<<"  * periods : " << motion_periods.transpose() << std::endl;
      std::cout<<" -------------------------------- "<<std::endl;
    }

  public:
    bool com_motion_given;
    bool foot_motion_given;
    SWING_DATA foot_motion_data;
    POSE_DATA com_motion_data;
    Eigen::VectorXd motion_periods;
};


//----------------------------------

class ManipulationCommand:  public UserCommand{
  public:
  ManipulationCommand(){
    ee_idx = 0;
    ee_motion_data = POSE_DATA(); 
    motion_period = 0.0;
  }
  ManipulationCommand(int _ee_idx,
                      const POSE_DATA& _ee_motion_data,
                      double _period) {
    ee_idx = _ee_idx;
    ee_motion_data = _ee_motion_data;
    motion_period = _period;
  }

  // util functions
  double get_motion_period(){ return motion_period;}
  int get_ee_idx(){return ee_idx;}
  bool get_ee_motion(POSE_DATA &_motion_data) {
    _motion_data = ee_motion_data;
    if(motion_period > 0.0 ) return true;
    else return false; 
  }

  void printMotionInfo(){
      std::cout<<" -------------------------------- "<<std::endl;
      std::cout<<" ManipulationCommand"<<std::endl;
      std::cout<<"  * ee motion [" <<ee_idx<<"] ";
      ee_motion_data.printMotionInfo(); 
      std::cout<<"  * periods : " << motion_period << std::endl;
      std::cout<<" -------------------------------- "<<std::endl;
    }
  public:
    int ee_idx;
    POSE_DATA ee_motion_data;
    double motion_period;
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
