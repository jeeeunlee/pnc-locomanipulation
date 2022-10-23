#include <my_robot_system/RobotSystem.hpp>
#include <my_robot_core/anymal_core/anymal_definition.hpp>
#include <my_robot_core/anymal_core/anymal_interface.hpp>
#include <my_robot_core/anymal_core/anymal_state_estimator.hpp>
#include <my_robot_core/anymal_core/anymal_state_provider.hpp>
#include <my_utils/IO/IOUtilities.hpp>

ANYmalStateEstimator::ANYmalStateEstimator(RobotSystem* robot) {
    my_utils::pretty_constructor(1, "ANYmal State Estimator");

    robot_ = robot;
    sp_ = ANYmalStateProvider::getStateProvider(robot_);
    
    curr_vjoint_pos_ = Eigen::VectorXd::Zero(ANYmal::n_vdof+1);
    curr_vjoint_vel_ = Eigen::VectorXd::Zero(ANYmal::n_vdof);
    prev_tau_cmd_ = Eigen::VectorXd::Zero(ANYmal::n_dof);
}

ANYmalStateEstimator::~ANYmalStateEstimator() {}

void ANYmalStateEstimator::Initialization(ANYmalSensorData* data) {
    sp_->jpos_ini = data->q; //sp_->getActiveJointValue(curr_config_);
    _JointUpdate(data);
    _ConfigurationAndModelUpdate();    
    _FootContactUpdate(data);
    sp_->saveCurrentData();
}

void ANYmalStateEstimator::Update(ANYmalSensorData* data) {
    _JointUpdate(data);
    _ConfigurationAndModelUpdate();
    _FootContactUpdate(data);
    sp_->saveCurrentData();
}

void ANYmalStateEstimator::_JointUpdate(ANYmalSensorData* data) {
    // active joints
    curr_joint_pos_ = data->q;
    curr_joint_vel_ = data->qdot;

    // virtual joints
    curr_base_pos_ = data->virtual_q.segment(0,3);
    curr_base_quat_ = Eigen::Quaterniond(
                            data->virtual_q[3],
                            data->virtual_q[4], 
                            data->virtual_q[5],
                            data->virtual_q[6]); // w,x,y,z
    curr_base_lin_vel_ = data->virtual_qdot.segment(0,3);
    curr_base_ang_vel_ = data->virtual_qdot.segment(3,3);

    curr_vjoint_pos_  = data->virtual_q;
    curr_vjoint_vel_ = data->virtual_qdot;

    // my_utils::pretty_print(curr_joint_pos_, std::cout, "curr_joint_pos_");
    // my_utils::pretty_print(curr_joint_vel_, std::cout, "curr_joint_vel_");
    // my_utils::pretty_print(curr_base_pos_, std::cout, "curr_base_pos_");
    // my_utils::pretty_print(curr_base_quat_, std::cout, "curr_base_quat_");
    // my_utils::pretty_print(curr_base_lin_vel_, std::cout, "curr_base_lin_vel_");
    // my_utils::pretty_print(curr_base_ang_vel_, std::cout, "curr_base_ang_vel_");

    // whole torques
    prev_tau_cmd_.setZero();    
    for (int i = 0; i < ANYmal::n_adof; ++i) {
        prev_tau_cmd_[ANYmal::idx_adof[i]] = data->tau_cmd_prev[i];
    }
}

void ANYmalStateEstimator::_ConfigurationAndModelUpdate() {    
    // robot_->updateSystem(curr_config_, curr_qdot_, true);
    robot_->updateSystem(curr_base_pos_,    curr_base_quat_,
                        curr_base_lin_vel_, curr_base_ang_vel_,
                        curr_joint_pos_,    curr_joint_vel_, 
                        true);

    sp_->q = sp_->getFullJointValue(
                    curr_joint_pos_, curr_vjoint_pos_);
    sp_->qdot = sp_->getFullJointValue(
                    curr_joint_vel_, curr_vjoint_vel_);
    sp_->tau_cmd_prev = prev_tau_cmd_;
}

void ANYmalStateEstimator::_FootContactUpdate(ANYmalSensorData* data) {

    for(int ii(0); ii<ANYmal::n_leg;++ii){
        sp_->b_foot_contact[ii] = data->b_foot_contact[ii] ? 1:0;
        sp_->foot_rf[ii] = data->foot_wrench[ii];
    }
}
