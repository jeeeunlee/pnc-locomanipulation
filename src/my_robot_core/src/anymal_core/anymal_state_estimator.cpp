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
    curr_config_ = Eigen::VectorXd::Zero(ANYmal::n_dof);
    curr_qdot_ = Eigen::VectorXd::Zero(ANYmal::n_dof);
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
    curr_config_.setZero();
    curr_qdot_.setZero();
    prev_tau_cmd_.setZero();
    for (int i = 0; i < ANYmal::n_vdof; ++i) {
        curr_config_[ANYmal::idx_vdof[i]] = data->virtual_q[i];
        curr_qdot_[ANYmal::idx_vdof[i]] = data->virtual_qdot[i];
    }
    for (int i = 0; i < ANYmal::n_adof; ++i) {
        curr_config_[ANYmal::idx_adof[i]] = data->q[i];
        curr_qdot_[ANYmal::idx_adof[i]] = data->qdot[i];
        prev_tau_cmd_[ANYmal::idx_adof[i]] = data->tau_cmd_prev[i];
    }
}

void ANYmalStateEstimator::_ConfigurationAndModelUpdate() {    
    robot_->updateSystem(curr_config_, curr_qdot_, true);
    sp_->q = curr_config_;
    sp_->qdot = curr_qdot_;
    sp_->tau_cmd_prev = prev_tau_cmd_;
}

void ANYmalStateEstimator::_FootContactUpdate(ANYmalSensorData* data) {

    for(int ii(0); ii<ANYmal::n_leg;++ii){
        sp_->b_foot_contact[ii] = data->b_foot_contact[ii] ? 1:0;
        sp_->foot_rf[ii] = data->foot_wrench[ii];
    }
}
