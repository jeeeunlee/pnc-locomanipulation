#include <my_wbc/Contact/GroundFrameContactSpec.hpp>


GroundFramePointContactSpec::GroundFramePointContactSpec(RobotSystem* robot, 
                                                int _link_idx, double _mu)
    : ContactSpec(robot, 3, _link_idx, _mu) {
    my_utils::pretty_constructor(3, "Ground Frame Point Contact Spec");

    updateContactSpec();
}

GroundFramePointContactSpec::~GroundFramePointContactSpec() {}

bool GroundFramePointContactSpec::_UpdateJc() {
    // spacial Jacobian wrt targeting body com frame (J_sb): getBodyNodeJacobian
    // body Jacobian wrt targeting body com frame (J_bb): getBodyNodeCoMBodyJacobian

    Eigen::MatrixXd Jtmp = robot_->getBodyNodeCoMJacobian(link_idx_);
    Jc_ = Jtmp.block(dim_contact_, 0, dim_contact_, robot_->getNumDofs());
    return true;
}

bool GroundFramePointContactSpec::_UpdateJcDotQdot() {

    Eigen::VectorXd JcDotQdot_tmp =
        robot_->getBodyNodeCoMJacobianDot(link_idx_) * robot_->getQdot();
    JcDotQdot_ = JcDotQdot_tmp.tail(dim_contact_);
    
    return true;
}

bool GroundFramePointContactSpec::_UpdateJcQdot() {
    Eigen::VectorXd JcQdot_tmp =
        robot_->getBodyNodeCoMJacobian(link_idx_) * robot_->getQdot();
    JcQdot_ = JcQdot_tmp.tail(dim_contact_);

    // JcQdot_.setZero();
    return true;
}

bool GroundFramePointContactSpec::_UpdateUf() {
    _setU(mu_, Uf_);
    return true;
}

bool GroundFramePointContactSpec::_UpdateInequalityVector() {
    _setIeqVec(max_Fz_, ieq_vec_);
    return true;
}

void GroundFramePointContactSpec::_setU( double mu,
                               Eigen::MatrixXd& U) {
    U = Eigen::MatrixXd::Zero(6, 3);
    // Linear
    U(0, 2) = 1.;  // Fz >= 0
    // x
    U(1, 2) = mu;
    U(2, 2) = mu;    
    U(1, 0) = 1.;
    U(2, 0) = -1.;    
    // y
    U(3, 1) = 1.;
    U(3, 2) = mu;
    U(4, 1) = -1.;
    U(4, 2) = mu;
    // Upper bound of vertical directional reaction force
    U(5, 2) = -1.0;  // -Fz >= -max_Fz_
}

void GroundFramePointContactSpec::_setIeqVec(double max_Fz, 
                                Eigen::VectorXd& ieq_vec) {
    ieq_vec = Eigen::VectorXd::Zero(6);
    ieq_vec[5] = -max_Fz;
}

void GroundFramePointContactSpec::getRFConstraintMtx(Eigen::MatrixXd& Uf) { 
    _setU(mu_, Uf);
}

void GroundFramePointContactSpec::getRFConstraintVec(Eigen::VectorXd& ieq_vec) { 
    _setIeqVec(max_Fz_, ieq_vec);
}