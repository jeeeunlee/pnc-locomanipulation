#include <my_robot_system/RobotSystem.hpp>
#include <my_robot_core/magneto_core/magneto_definition.hpp>
#include <my_robot_core/magneto_core/magneto_interface.hpp>
#include <my_robot_core/magneto_core/magneto_state_estimator.hpp>
#include <my_robot_core/magneto_core/magneto_state_provider.hpp>
#include <my_utils/IO/IOUtilities.hpp>

MagnetoStateEstimator::MagnetoStateEstimator(RobotSystem* robot) {
    my_utils::pretty_constructor(1, "Magneto State Estimator");

    robot_ = robot;
    sp_ = MagnetoStateProvider::getStateProvider(robot_);
    curr_config_ = Eigen::VectorXd::Zero(Magneto::n_dof);
    curr_qdot_ = Eigen::VectorXd::Zero(Magneto::n_dof);
    prev_tau_cmd_ = Eigen::VectorXd::Zero(Magneto::n_dof);
}

MagnetoStateEstimator::~MagnetoStateEstimator() {}

void MagnetoStateEstimator::Initialization(MagnetoSensorData* data) {
    sp_->jpos_ini = data->q; //sp_->getActiveJointValue(curr_config_);
    _JointUpdate(data);
    _ConfigurationAndModelUpdate();    
    _FootContactUpdate(data);
    sp_->saveCurrentData();
}

void MagnetoStateEstimator::Update(MagnetoSensorData* data) {
    _JointUpdate(data);
    _ConfigurationAndModelUpdate();
    _FootContactUpdate(data);
    sp_->saveCurrentData();
}

void MagnetoStateEstimator::_JointUpdate(MagnetoSensorData* data) {
    curr_config_.setZero();
    curr_qdot_.setZero();
    prev_tau_cmd_.setZero();
    for (int i = 0; i < Magneto::n_vdof; ++i) {
        curr_config_[Magneto::idx_vdof[i]] = data->virtual_q[i];
        curr_qdot_[Magneto::idx_vdof[i]] = data->virtual_qdot[i];
    }
    for (int i = 0; i < Magneto::n_adof; ++i) {
        curr_config_[Magneto::idx_adof[i]] = data->q[i];
        curr_qdot_[Magneto::idx_adof[i]] = data->qdot[i];
        prev_tau_cmd_[Magneto::idx_adof[i]] = data->tau_cmd_prev[i];
    }
}

void MagnetoStateEstimator::_ConfigurationAndModelUpdate() {    
    robot_->updateSystem(curr_config_, curr_qdot_, true);
    sp_->q = curr_config_;
    sp_->qdot = curr_qdot_;
    sp_->tau_cmd_prev = prev_tau_cmd_;
}

void MagnetoStateEstimator::_FootContactUpdate(MagnetoSensorData* data) {
    sp_->b_arfoot_contact = data->arfoot_contact ? 1:0;
    sp_->b_brfoot_contact = data->brfoot_contact ? 1:0;
    sp_->b_alfoot_contact = data->alfoot_contact ? 1:0;
    sp_->b_blfoot_contact = data->blfoot_contact ? 1:0;

    sp_->al_rf = data->alf_wrench;
    sp_->bl_rf = data->blf_wrench;
    sp_->ar_rf = data->arf_wrench;
    sp_->br_rf = data->brf_wrench;

    sp_->surface_normal = data->surface_normal;
}
