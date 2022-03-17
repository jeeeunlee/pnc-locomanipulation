
#include <../my_utils/Configuration.h>

#include <my_robot_system/RobotSystem.hpp>

#include <my_robot_core/magneto_core/magneto_definition.hpp>
// #include <my_robot_core/magneto_core/magneto_interface.hpp>

#include <my_robot_core/magneto_core/magneto_planner/magneto_com_planner.hpp>
#include <my_utils/IO/IOUtilities.hpp>
#include "my_utils/Math/pseudo_inverse.hpp"

MagnetoCoMPlanner::MagnetoCoMPlanner(RobotSystem* robot) {
    my_utils::pretty_constructor(2, "Magneto CoM Hermite Spline Parameter Planner");

    // robot system
    robot_ = robot;
    sp_ = MagnetoStateProvider::getStateProvider(robot_);

    initialized = false;
    Tt_ = 0.0; 
}

void MagnetoCoMPlanner::computeSequence(const Eigen::Vector3d& pcom_goal,
                                        const MotionCommand &_motion_command){
    // Tt_ set in advance
    Ts_ = _motion_command.get_foot_motion_period();
    Tf_ = Ts_;

    p_init_ = robot_ ->getCoMPosition();  
    p_goal_ = pcom_goal;
    zero_vel_.setZero();

    // next foot configuration
    MOTION_DATA md;    
    _motion_command.get_foot_motion(md, swing_foot_idx_);

}

void MagnetoCoMPlanner::_buildPfRf() {
    Pf_ = Eigen::MatrixXd::Zero(0,0);
    Rf_ = Eigen::MatrixXd::Zero(0,0);
    Pc_ = Eigen::MatrixXd::Zero(0,0);
    Rc_ = Eigen::MatrixXd::Zero(0,0);

    Eigen::MatrixXd PRi, Ri;
    Eigen::VectorXd pi;
    for(int i(0); i<Magneto::n_leg; ++i) {        
        pi = robot_->getBodyNodeIsometry(
            MagnetoFoot::LinkIdx[i]).translation();
        Ri = robot_->getBodyNodeIsometry(
            MagnetoFoot::LinkIdx[i]).linear();            
        PRi = skew(pi)*Ri;

        Pf_ = my_utils::hStackConserve(Pf_, PRi);
        Rf_ = my_utils::hStackConserve(Rf_, Ri);
        if(MagnetoFoot::LinkIdx[i] != swing_foot_idx_){
            Pc_ = my_utils::hStackConserve(Pc_, PRi);
            Rc_ = my_utils::hStackConserve(Rc_, Ri);
        }
    }
}



ComMotionCommand MagnetoCoMPlanner::getFullSupportCoMCmd() {
    Eigen::Vector3d pa = robot_ ->getCoMPosition();  
    Eigen::Vector3d va = Eigen::VectorXd::Zero(3);

    return ComMotionCommand( pa, va, p_swing_init_, v_swing_init_, Tf_ );
}

ComMotionCommand MagnetoCoMPlanner::getSwingStartCoMCmd() {
    // constant acc
    Eigen::Vector3d pa = sp_->com_pos_des_;
    Eigen::Vector3d va = sp_->com_vel_des_;

    return ComMotionCommand( pa, va, acc_swing_, Tt_ );
}

ComMotionCommand MagnetoCoMPlanner::getSwingCoMCmd() {
    // constant acc
    Eigen::Vector3d pa = sp_->com_pos_des_;
    Eigen::Vector3d va = sp_->com_vel_des_;

    return ComMotionCommand( pa, va, acc_swing_, Ts_ );
}

ComMotionCommand MagnetoCoMPlanner::getSwingEndCoMCmd() {
    Eigen::Vector3d pa = sp_->com_pos_des_;
    Eigen::Vector3d va = sp_->com_vel_des_;
    
    return ComMotionCommand( pa, va, p_goal_, zero_vel_, Tt_ );
}




