#pragma once

#include <Eigen/Dense>

#include <my_robot_core/magneto_core/magneto_command_api.hpp>

class RobotSystem;

class MagnetoCoMPlanner(){
    public:
    MagnetoCoMPlanner(RobotSystem* robot);
    ~MagnetoCoMPlanner() {};

    void computeSequence(const Eigen::Vector3d& pcom_goal,
                        const MotionCommand &_motion_command);
    void setTransitionDuration(double _Tt) {Tt_ = _Tt;}

    private:
        void _buildPfRf();

    private:
        Eigen::Vector3d p_init_;
        Eigen::Vector3d p_goal_;
        Eigen::Vector3d zero_vel_;

        double Tf_; // full_period;
        double Ts_; // swing_period;        
        double Tt_; // trans_period;

        int swing_foot_idx_;

        Eigen::MatrixXd Pf_; // stack full
        Eigen::MatrixXd Rf_; // stack full
        Eigen::MatrixXd Pc_; // stack only contact
        Eigen::MatrixXd Rc_; // stack only contact

        Eigen::Vector3d acc_swing_;
        Eigen::Vector3d p_swing_init_;
        Eigen::Vector3d v_swing_init_;

        bool initialized;
        MagnetoStateProvider* sp_;
}

