#pragma once

#include <my_robot_core/user_command.hpp>
#include <Eigen/Dense>
#include <iostream>

typedef int TARGET_LINK_IDX;

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
};

struct MOTION_DATA {
    POSE_DATA pose;
    double swing_height;
    double motion_period;

    MOTION_DATA() {
        pose = POSE_DATA();
        motion_period = 0.0;
        swing_height = 0.0;
    }
    MOTION_DATA(const POSE_DATA &_pose, 
                double _motion_period,
                double _swing_height=0.0) {
        pose = _pose;
        motion_period = _motion_period;
        swing_height = _swing_height;
    }

    void printMotionInfo(){
      std::cout<<"@("<<pose.pos[0]<<","<<pose.pos[1]<<","<<pose.pos[2]<< ")["<<pose.is_baseframe<<"] / motion period : "<<motion_period<<std::endl;
    }
};

// specific component
class MotionCommand : public UserCommand {
  public:
    MotionCommand()
    :com_motion_given(false), foot_motion_given(false), swing_foot_idx(-1) {
      foot_motion_data = MOTION_DATA();
      com_motion_data = MOTION_DATA();
    }
    MotionCommand( const MOTION_DATA& _com_motion_data ):MotionCommand() {
      com_motion_given = true;
      com_motion_data = _com_motion_data;
    }
    MotionCommand(int _moving_link_id,
                const MOTION_DATA& _foot_motion_data): MotionCommand() {
      foot_motion_given = true;
      swing_foot_idx =  _moving_link_id;
      foot_motion_data = _foot_motion_data;
    }
    MotionCommand(int _moving_link_id,
                const MOTION_DATA& _foot_motion_data,
                const MOTION_DATA& _com_motion_data)
    :com_motion_given(true), foot_motion_given(true), swing_foot_idx(_moving_link_id) {
      com_motion_data = _com_motion_data;
      foot_motion_data = _foot_motion_data;
    }
    MotionCommand(const MotionCommand& _mc){
      com_motion_given = _mc.com_motion_given;
      foot_motion_given = _mc.foot_motion_given;
      swing_foot_idx =  _mc.swing_foot_idx;
      com_motion_data = _mc.com_motion_data;  
      foot_motion_data = _mc.foot_motion_data;
    }
    ~MotionCommand() {};

  public:
    void set_com_motion(const MOTION_DATA& _com_motion_data){
      com_motion_given =  true;
      com_motion_data = _com_motion_data;
    }
    void set_foot_motion(int _moving_link_id,
                        const MOTION_DATA& _foot_motion_data){
      foot_motion_given = true;
      swing_foot_idx = _moving_link_id;
      foot_motion_data = _foot_motion_data;
    }
    int get_moving_foot() {return swing_foot_idx; }
    bool get_foot_motion(MOTION_DATA &_motion_data, TARGET_LINK_IDX &_idx){
      _motion_data = foot_motion_data;
      _idx = swing_foot_idx;
      return foot_motion_given;
    }
    bool get_foot_motion(MOTION_DATA &_motion_data){
      _motion_data = foot_motion_data;
      return foot_motion_given;
    }
    bool get_com_motion(MOTION_DATA &_motion_data){
      _motion_data = com_motion_data;
      return com_motion_given;
    }
    double get_foot_motion_period(){
      return foot_motion_data.motion_period;
    }

    void printMotionInfo(){
      std::cout<<" ---------------- "<<std::endl;
      std::cout<<" MotionCommand"<<std::endl;
      std::cout<<"  * com motion : ";
      if(com_motion_given){ com_motion_data.printMotionInfo(); }
      else { std::cout<< "not given"<< std::endl; }
      std::cout<<"  * foot motion";
      if(foot_motion_given){ std::cout<<"["<<swing_foot_idx<<"] : "; 
                            foot_motion_data.printMotionInfo(); }
      else { std::cout<< " : not given"<< std::endl; }
      std::cout<<" ---------------- "<<std::endl;
    }

  protected:
    bool com_motion_given;
    bool foot_motion_given;
    TARGET_LINK_IDX swing_foot_idx;
    MOTION_DATA foot_motion_data;
    MOTION_DATA com_motion_data;
};

class SimulationCommand : public UserCommand {
  public:
    SimulationCommand(): mu(0.7), f_adhesive(100.) {}
    SimulationCommand(double _mu, double _fm): mu(_mu), f_adhesive(_fm) {}
    ~SimulationCommand() {};
    void getSimEnv(double& _mu, double& _fm){
      _mu = mu;
      _fm = f_adhesive;
    }

  protected:
    double mu;
    double f_adhesive;
};

class SimMotionCommand : public MotionCommand, public SimulationCommand {
  public:
    SimMotionCommand(): MotionCommand(), SimulationCommand() {}
    SimMotionCommand(const MotionCommand& motion_cmd)
                    : MotionCommand(motion_cmd), SimulationCommand() {}
    SimMotionCommand(const MotionCommand& motion_cmd,
                    double _mu,  double _fm )
                    : MotionCommand(motion_cmd), SimulationCommand(_mu, _fm) {}
    // SimMotionCommand(const SimMotionCommand& sim_motion_cmd);
    ~SimMotionCommand() {}
};

