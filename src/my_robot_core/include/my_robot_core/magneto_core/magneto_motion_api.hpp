#pragma once

#include <../my_utils/Configuration.h>
#include <Eigen/Dense>
#include <deque>
#include <map>

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
        is_baseframe = true;
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
    MOTION_DATA(const Eigen::Vector3d& _pos,  
                const Eigen::Quaternion<double>& _ori,
                bool _is_baseframe,
                double _motion_period,
                double _swing_height=0.0) {
        pose = POSE_DATA(_pos,_ori,_is_baseframe);
        motion_period = _motion_period;
        swing_height = _swing_height;
    }
};

class MotionCommand{
public:
    MotionCommand();
    MotionCommand(int _moving_link_id,
                const MOTION_DATA& _motion_data);
    ~MotionCommand(){};

    // function
    void add_com_motion(const POSE_DATA& _pose_del,
                        double _motion_period);
    void add_motion(int _moving_link_id,
                    const MOTION_DATA& _pose_del);

    int get_moving_foot();
    bool get_foot_motion_command(MOTION_DATA &_motion_data, TARGET_LINK_IDX &_idx) ;
    bool get_foot_motion_command(MOTION_DATA &_motion_data);
    bool get_com_motion_command(MOTION_DATA &_motion_data);

protected:  
    // link_idx, assume IDX of COM is -1
    // just in case a robot should swing more than two legs at the same time.
    std::map<TARGET_LINK_IDX, MOTION_DATA> motion_sets_;
};

////////////////////////////////////////


// class SimEnvCommand {
//   public:
//     SimEnvCommand(): foot_idx(-1), mu(0.7), f_adhesive(100.){ };
//     SimEnvCommand(TARGET_LINK_IDX _idx, double _mu, double _f)
//     : foot_idx(_idx), mu(_mu), f_adhesive(_f){ };
//     ~SimEnvCommand(){};

//   protected:
//     TARGET_LINK_IDX foot_idx;
//     double mu;
//     double f_adhesive;
// };

class SimMotionCommand : public MotionCommand{
  public:
    SimMotionCommand();
    SimMotionCommand(int _moving_link_id,
                const MOTION_DATA& _motion_data,
                double mu = 0.7,
                double f_adhesive= 100.);
    ~SimMotionCommand() {};

    void getSimEnv(double& _mu, double _fm){
        _mu = mu;
        _fm = f_adhesive;
    };

  protected:
    // std::map<TARGET_LINK_IDX, double> mu_sets_;
    // std::map<TARGET_LINK_IDX, double> f_sets_;

    double mu;
    double f_adhesive;  
};