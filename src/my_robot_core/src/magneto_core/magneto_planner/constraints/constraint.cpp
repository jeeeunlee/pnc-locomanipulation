#pragma once

#include <my_robot_core/magneto_core/magneto_planner/constraints/constraint.hpp>

Constraint::Constraint(RobotSystem* _robot, int _link_idx) {
    robot_ = _robot;
    link_idx_ = _link_idx;
    dim_constraint_ = 6;
    dim_position_constraint_ = 3;
    dim_joint_constraint_ = _robot->getNumDofs();
    b_updated_ = false;
}

void Constraint::setDesired(const POSE_DATA& pos_del) {
    pos_ini_ = robot_->getBodyNodeIsometry(link_idx_).translation();
    ori_ini_ = Eigen::Quaternion<double>(robot_->getBodyNodeIsometry(link_idx_).linear());

    if(pos_del.is_bodyframe) {
        Eigen::MatrixXd R_wb = 
            robot_->getBodyNodeIsometry(MagnetoBodyNode::base_link).linear(); 
        pos_des_ = pos_ini_ + R_wb*(pos_del.pos);     
    } else {
        pos_des_ = pos_ini_ + pos_del.pos;
    }
    ori_des_ = Eigen::Quaternion<double>(ori_ini_); //pos_del.ori
}

void Constraint::update() { 
    _updatePosition();
    _updatePositionError();
    _updateJacobian();
    b_updated_ = true;
}

void Constraint::_updatePositionError() {
    Pcs_err_ = Pcs_ - pos_des_;
}

void Constraint::_updatePosition() {
    Pcs_ = robot_->getBodyNodeIsometry(link_idx_).translation();
    // dim_position_constraint_ = Pcs_.size();
}

void Constraint::_updateJacobian() {
    Jcs_ = robot_->getBodyNodeJacobian(link_idx_);
    // dim_constraint_ = Jcs_.rows(); //6
    // dim_joint_constraint_ = Jcs_.cols(); //30
}