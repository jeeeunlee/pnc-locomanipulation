#pragma once

#include <../my_utils/Configuration.h>
#include <Eigen/Dense>
#include <deque>
#include <map>

class MotionCommand;

typedef int TARGET_LINK_IDX;
// typedef std::deque<MotionCommand*> MotionCommandDeque;

struct POSE_DATA {
    Eigen::Vector3d pos;
    Eigen::Quaternion<double> ori;
    bool is_bodyframe;

    POSE_DATA() {
        is_bodyframe = true;
        pos = Eigen::VectorXd::Zero(3);
        ori = Eigen::Quaternion<double>::Identity();
    }

    POSE_DATA(const Eigen::Vector3d& _pos, 
              const Eigen::Quaternion<double>& _ori,
              bool _is_bodyframe=true)
              : pos(_pos),ori(_ori),is_bodyframe(_is_bodyframe){
    }

    POSE_DATA(const Eigen::VectorXd& _pos, 
              const Eigen::VectorXd& _ori,
              bool _is_bodyframe=true) 
              : is_bodyframe(_is_bodyframe) {
        is_bodyframe = true;
        pos << _pos[0], _pos[1], _pos[2];
        ori = Eigen::Quaternion<double>
                (_ori[0], _ori[1], _ori[2], _ori[3]);
    }

    POSE_DATA(double x, double y, double z, 
              double w, double qx, double qy, double qz) {
        is_bodyframe = true;
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
                bool _is_bodyframe,
                double _motion_period,
                double _swing_height=0.0) {
        pose = POSE_DATA(_pos,_ori,_is_bodyframe);
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
    void add_foot_motion(int _moving_foot_link_id,
                        const POSE_DATA& _pose_del,
                        double _motion_period);
    void add_com_motion(const POSE_DATA& _pose_del,
                        double _motion_period);
    void add_motion(int _moving_link_id,
                    const MOTION_DATA& _pose_del);

    void clear_motion();
    void clear_and_add_motion(int _moving_link_id,
                    const MOTION_DATA& _motion_data);
    

    int get_moving_foot();
    bool get_foot_motion_command(MOTION_DATA &_motion_data, TARGET_LINK_IDX &_idx) ;
    bool get_foot_motion_command(MOTION_DATA &_motion_data);
    bool get_com_motion_command(MOTION_DATA &_motion_data);

    int get_num_of_target(){  return motion_sets_.size();  };
    int get_num_of_foot_target();
    bool is_com_target_exist();

private:  
    // link_idx, assume IDX of COM is -1
    // just in case a robot should swing more than two legs at the same time.
    std::map<TARGET_LINK_IDX, MOTION_DATA> motion_sets_;
};
