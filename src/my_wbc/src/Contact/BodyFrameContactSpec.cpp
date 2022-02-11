#include <my_wbc/Contact/BodyFrameContactSpec.hpp>


BodyFramePointContactSpec::BodyFramePointContactSpec(RobotSystem* robot, 
                                                int _link_idx, double _mu)
    : ContactSpec(robot, 3, _link_idx, _mu) {
    my_utils::pretty_constructor(3, "Body Frame Point Contact Spec");

    updateContactSpec();
}

BodyFramePointContactSpec::~BodyFramePointContactSpec() {}

bool BodyFramePointContactSpec::_UpdateJc() {
    // spacial Jacobian wrt targeting body com frame (J_sb): getBodyNodeJacobian
    // body Jacobian wrt targeting body com frame (J_bb): getBodyNodeCoMBodyJacobian

    Eigen::MatrixXd Jtmp = robot_->getBodyNodeCoMBodyJacobian(link_idx_);
    Jc_ = Jtmp.block(dim_contact_, 0, dim_contact_, robot_->getNumDofs());
    return true;
}

bool BodyFramePointContactSpec::_UpdateJcDotQdot() {

    Eigen::VectorXd JcDotQdot_tmp =
        robot_->getBodyNodeCoMBodyJacobianDot(link_idx_) * robot_->getQdot();
    JcDotQdot_ = JcDotQdot_tmp.tail(dim_contact_);

    // Assum xc_ddot = Jc*qddot + JcdotQdot = 0
    // Eigen::MatrixXd Jtmp = robot_->getBodyNodeCoMBodyJacobian(link_idx_);
    // Eigen::VectorXd Jcqddot = Jtmp * robot_->getQddot();
    // JcDotQdot_= -Jcqddot.tail(dim_contact_);

    // JcDotQdot_.setZero();
    
    return true;
}

bool BodyFramePointContactSpec::_UpdateJcQdot() {
    Eigen::VectorXd JcQdot_tmp =
        robot_->getBodyNodeCoMBodyJacobian(link_idx_) * robot_->getQdot();
    JcQdot_ = JcQdot_tmp.tail(dim_contact_);

    // JcQdot_.setZero();
    return true;
}

bool BodyFramePointContactSpec::_UpdateUf() {

    Uf_ = Eigen::MatrixXd::Zero(6, dim_contact_);
    // Linear
    Uf_(0, 2) = 1.;  // Fz >= 0

    Uf_(1, 0) = 1.0;
    Uf_(1, 2) = mu_;
    Uf_(2, 0) = -1.0;
    Uf_(2, 2) = mu_;

    Uf_(3, 1) = 1.0;
    Uf_(3, 2) = mu_;
    Uf_(4, 1) = -1.0;
    Uf_(4, 2) = mu_;

    // Upper bound of vertical directional reaction force
    Uf_(5, 2) = -1.0;  // -Fz >= -max_Fz_

    return true;
}

bool BodyFramePointContactSpec::_UpdateInequalityVector() {
    ieq_vec_ = Eigen::VectorXd::Zero(6);
    ieq_vec_[5] = -max_Fz_;
    return true;
}



BodyFrameSurfaceContactSpec::BodyFrameSurfaceContactSpec(RobotSystem* robot,
                        int _link_idx, double _x, double _y, double _mu)
    : ContactSpec(robot, 6, _link_idx, _mu) {
    my_utils::pretty_constructor(3, "BodyFrame Surface Contact Spec");

    x_ = _x;
    y_ = _y;
    updateContactSpec();
}

BodyFrameSurfaceContactSpec::~BodyFrameSurfaceContactSpec() {}

bool BodyFrameSurfaceContactSpec::_UpdateJc() {
    // spacial Jacobian wrt targeting body frame (J_sb): getBodyNodeJacobian
    // body Jacobian wrt targeting body frame (J_bb): getBodyNodeBodyJacobian
    Jc_ = robot_->getBodyNodeBodyJacobian(link_idx_);
    return true;
}

bool BodyFrameSurfaceContactSpec::_UpdateJcDotQdot() {
    JcDotQdot_ = robot_->getBodyNodeBodyJacobianDot(link_idx_) * robot_->getQdot();
    // JcDotQdot_.setZero();
    return true;
}

bool BodyFrameSurfaceContactSpec::_UpdateJcQdot() {
    JcQdot_ = robot_->getBodyNodeBodyJacobian(link_idx_) * robot_->getQdot();
    // JcQdot_.setZero();
    return true;
}

bool BodyFrameSurfaceContactSpec::_UpdateUf() {
    _setU(x_, y_, mu_, Uf_);
    return true;
}

bool BodyFrameSurfaceContactSpec::_UpdateInequalityVector() {
    ieq_vec_ = Eigen::VectorXd::Zero(16 + 2);
    ieq_vec_[17] = -max_Fz_;
    return true;
}

void BodyFrameSurfaceContactSpec::_setU(double x, double y, double mu,
                               Eigen::MatrixXd& U) {
    U = Eigen::MatrixXd::Zero(16 + 2, 6);

    U(0, 5) = 1.;

    U(1, 3) = 1.;
    U(1, 5) = mu;
    U(2, 3) = -1.;
    U(2, 5) = mu;

    U(3, 4) = 1.;
    U(3, 5) = mu;
    U(4, 4) = -1.;
    U(4, 5) = mu;

    U(5, 0) = 1.;
    U(5, 5) = y;
    U(6, 0) = -1.;
    U(6, 5) = y;

    U(7, 1) = 1.;
    U(7, 5) = x;
    U(8, 1) = -1.;
    U(8, 5) = x;

    // Tau
    U(9, 0) = -mu;
    U(9, 1) = -mu;
    U(9, 2) = 1;
    U(9, 3) = y;
    U(9, 4) = x;
    U(9, 5) = (x + y) * mu;

    U(10, 0) = -mu;
    U(10, 1) = mu;
    U(10, 2) = 1;
    U(10, 3) = y;
    U(10, 4) = -x;
    U(10, 5) = (x + y) * mu;

    U(11, 0) = mu;
    U(11, 1) = -mu;
    U(11, 2) = 1;
    U(11, 3) = -y;
    U(11, 4) = x;
    U(11, 5) = (x + y) * mu;

    U(12, 0) = mu;
    U(12, 1) = mu;
    U(12, 2) = 1;
    U(12, 3) = -y;
    U(12, 4) = -x;
    U(12, 5) = (x + y) * mu;
    /////////////////////////////////////////////////
    U(13, 0) = -mu;
    U(13, 1) = -mu;
    U(13, 2) = -1;
    U(13, 3) = -y;
    U(13, 4) = -x;
    U(13, 5) = (x + y) * mu;

    U(14, 0) = -mu;
    U(14, 1) = mu;
    U(14, 2) = -1;
    U(14, 3) = -y;
    U(14, 4) = x;
    U(14, 5) = (x + y) * mu;

    U(15, 0) = mu;
    U(15, 1) = -mu;
    U(15, 2) = -1;
    U(15, 3) = y;
    U(15, 4) = -x;
    U(15, 5) = (x + y) * mu;

    U(16, 0) = mu;
    U(16, 1) = mu;
    U(16, 2) = -1;
    U(16, 3) = y;
    U(16, 4) = x;
    U(16, 5) = (x + y) * mu;
    // ////////////////////////////////////////////////////
    U(17, 5) = -1.;
}
