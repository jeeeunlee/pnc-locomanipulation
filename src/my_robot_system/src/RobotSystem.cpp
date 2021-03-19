#include <my_robot_system//RobotSystem.hpp>
#include <chrono>

RobotSystem::RobotSystem(const RobotSystem& robotsys) 
:RobotSystem(robotsys.num_virtual_dof_, robotsys.skel_file_name_)  {
}

RobotSystem::RobotSystem(int numVirtual_, std::string file) {
    my_utils::pretty_constructor(1, "Robot Model");
    skel_file_name_ = file;

    dart::utils::DartLoader urdfLoader;
    skel_ptr_ = urdfLoader.parseSkeleton(file);
    num_dof_ = skel_ptr_->getNumDofs();
    num_virtual_dof_ = numVirtual_;
    num_actuated_dof_ = num_dof_ - num_virtual_dof_;
    num_body_nodes_ = skel_ptr_->getNumBodyNodes();
    I_cent_ = Eigen::MatrixXd::Zero(6, 6);
    J_cent_ = Eigen::MatrixXd::Zero(6, num_dof_);
    A_cent_ = Eigen::MatrixXd::Zero(6, num_dof_);
    
    setActuatedJoint();
}

RobotSystem::~RobotSystem() {}

void RobotSystem::setActuatedJoint()  {
    idx_adof_.clear();
    for(int i=0;i<num_actuated_dof_;++i)    
        idx_adof_.push_back(num_virtual_dof_ + i); 
}
void RobotSystem::setActuatedJoint(const int *_idx_adof)  {
    idx_adof_.clear();
    for(int i=0;i<num_actuated_dof_;++i)    
        idx_adof_.push_back(_idx_adof[i]);  
}

Eigen::VectorXd RobotSystem::getActiveJointValue(const Eigen::VectorXd& q_full)  {
    Eigen::VectorXd q_a(num_actuated_dof_);
    for(int i = 0; i < num_actuated_dof_; ++i)        
        q_a[i] = q_full[idx_adof_[i]]; 
    return  q_a;
}

Eigen::MatrixXd RobotSystem::getMassMatrix() {
    return skel_ptr_->getMassMatrix();
}

Eigen::MatrixXd RobotSystem::getInvMassMatrix() {
    return skel_ptr_->getInvMassMatrix();
}
Eigen::VectorXd RobotSystem::getCoriolisGravity() {
    return skel_ptr_->getCoriolisAndGravityForces();
}

Eigen::VectorXd RobotSystem::getCoriolis() {
    return skel_ptr_->getCoriolisForces();
}

Eigen::VectorXd RobotSystem::getGravity() {
    return skel_ptr_->getGravityForces();
}

Eigen::MatrixXd RobotSystem::getCentroidJacobian() { return J_cent_; }

Eigen::MatrixXd RobotSystem::getCentroidInertiaTimesJacobian() {
    return A_cent_;
}

Eigen::MatrixXd RobotSystem::getCentroidInertia() { return I_cent_; }

Eigen::Vector3d RobotSystem::getCoMPosition(dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getCOM(wrt_);
}

Eigen::Vector3d RobotSystem::getCoMVelocity(dart::dynamics::Frame* rl_,
                                            dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getCOMLinearVelocity(rl_, wrt_);
}

Eigen::Vector3d RobotSystem::getCoMAcceleration(dart::dynamics::Frame* rl_,
                                            dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getCOMLinearAcceleration(rl_, wrt_);
}

Eigen::Isometry3d RobotSystem::getBodyNodeIsometry(
    const std::string& name_, dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getBodyNode(name_)->getTransform(wrt_);
}

Eigen::Isometry3d RobotSystem::getBodyNodeIsometry(
    const int& _bn_idx, dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getBodyNode(_bn_idx)->getTransform(wrt_);
}

Eigen::Isometry3d RobotSystem::getBodyNodeCoMIsometry(
    const std::string& name_, dart::dynamics::Frame* wrt_) {
    Eigen::Isometry3d ret = Eigen::Isometry3d::Identity();
    ret.linear() = getBodyNodeIsometry(name_, wrt_).linear();
    ret.translation() = skel_ptr_->getBodyNode(name_)->getCOM(wrt_);
    return ret;
}

Eigen::Isometry3d RobotSystem::getBodyNodeCoMIsometry(
    const int& _bn_idx, dart::dynamics::Frame* wrt_) {
    Eigen::Isometry3d ret = Eigen::Isometry3d::Identity();
    ret.linear() = getBodyNodeIsometry(_bn_idx, wrt_).linear();
    ret.translation() = skel_ptr_->getBodyNode(_bn_idx)->getCOM(wrt_);
    return ret;
}

Eigen::Vector6d RobotSystem::getBodyNodeSpatialVelocity(
    const std::string& name_, dart::dynamics::Frame* rl_,
    dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getBodyNode(name_)->getSpatialVelocity(rl_, wrt_);
}

Eigen::Vector6d RobotSystem::getBodyNodeSpatialVelocity(
    const int& _bn_idx, dart::dynamics::Frame* rl_,
    dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getBodyNode(_bn_idx)->getSpatialVelocity(rl_, wrt_);
}

Eigen::Vector6d RobotSystem::getBodyNodeCoMSpatialVelocity(
    const std::string& name_, dart::dynamics::Frame* rl_,
    dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getBodyNode(name_)->getCOMSpatialVelocity(rl_, wrt_);
}

Eigen::Vector6d RobotSystem::getBodyNodeCoMSpatialVelocity(
    const int& _bn_idx, dart::dynamics::Frame* rl_,
    dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getBodyNode(_bn_idx)->getCOMSpatialVelocity(rl_, wrt_);
}

// JE added 2020/2/9
Eigen::Vector6d RobotSystem::getBodyNodeSpatialAcceleration(
    const std::string& name_, dart::dynamics::Frame* rl_,
    dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getBodyNode(name_)->getSpatialAcceleration(rl_, wrt_);
}

Eigen::Vector6d RobotSystem::getBodyNodeSpatialAcceleration(
    const int& _bn_idx, dart::dynamics::Frame* rl_,
    dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getBodyNode(_bn_idx)->getSpatialAcceleration(rl_, wrt_);
}

Eigen::Vector6d RobotSystem::getBodyNodeCoMSpatialAcceleration(
    const std::string& name_, dart::dynamics::Frame* rl_,
    dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getBodyNode(name_)->getCOMSpatialAcceleration(rl_, wrt_);
}

Eigen::Vector6d RobotSystem::getBodyNodeCoMSpatialAcceleration(
    const int& _bn_idx, dart::dynamics::Frame* rl_,
    dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getBodyNode(_bn_idx)->getCOMSpatialAcceleration(rl_, wrt_);
}
//

Eigen::VectorXd RobotSystem::getCentroidVelocity() {
    return J_cent_ * skel_ptr_->getVelocities();
}

Eigen::VectorXd RobotSystem::getCentroidMomentum() {
    return A_cent_ * skel_ptr_->getVelocities();
}

Eigen::MatrixXd RobotSystem::getCoMJacobian(dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getCOMJacobian(wrt_);
}

Eigen::MatrixXd RobotSystem::getBodyNodeJacobian(const std::string& name_,
                                                 Eigen::Vector3d localOffset_,
                                                 dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getJacobian(skel_ptr_->getBodyNode(name_), localOffset_,
                                  wrt_);
}

Eigen::MatrixXd RobotSystem::getBodyNodeJacobian(const int& _bn_idx,
                                                 Eigen::Vector3d localOffset_,
                                                 dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getJacobian(skel_ptr_->getBodyNode(_bn_idx), localOffset_,
                                  wrt_);
}

Eigen::MatrixXd RobotSystem::getBodyNodeJacobianDot(
    const std::string& name_, Eigen::Vector3d localOffset_,
    dart::dynamics::Frame* wrt_) {
    // return skel_ptr_->getJacobianSpatialDeriv(skel_ptr_->getBodyNode(name_),
    //                                          localOffset_, wrt_);

    return skel_ptr_->getJacobianClassicDeriv(skel_ptr_->getBodyNode(name_),
                                              localOffset_, wrt_);
}

Eigen::MatrixXd RobotSystem::getBodyNodeJacobianDot(
    const int& _bn_idx, Eigen::Vector3d localOffset_,
    dart::dynamics::Frame* wrt_) {
    // return skel_ptr_->getJacobianSpatialDeriv(skel_ptr_->getBodyNode(name_),
    // localOffset_,
    // wrt_);

    return skel_ptr_->getJacobianClassicDeriv(skel_ptr_->getBodyNode(_bn_idx),
                                              localOffset_, wrt_);
}

Eigen::MatrixXd RobotSystem::getBodyNodeBodyJacobian(const int& _bn_idx, Eigen::Vector3d localOffset_ )
{
    return skel_ptr_->getJacobian(skel_ptr_->getBodyNode(_bn_idx), localOffset_);
}

Eigen::MatrixXd RobotSystem::getBodyNodeBodyJacobianDot(const int& _bn_idx,Eigen::Vector3d localOffset_)
{
    // return skel_ptr_->getJacobianSpatialDeriv(skel_ptr_->getBodyNode(_bn_idx),
    //                                           localOffset_);
    return skel_ptr_->getJacobianClassicDeriv(skel_ptr_->getBodyNode(_bn_idx),
                                              localOffset_);                                              
}

Eigen::MatrixXd RobotSystem::getBodyNodeCoMJacobian(
    const std::string& name_, dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getJacobian(skel_ptr_->getBodyNode(name_),
                                  skel_ptr_->getBodyNode(name_)->getLocalCOM(),
                                  wrt_);
}

Eigen::MatrixXd RobotSystem::getBodyNodeCoMJacobian(
    const int& _bn_idx, dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getJacobian(
        skel_ptr_->getBodyNode(_bn_idx),
        skel_ptr_->getBodyNode(_bn_idx)->getLocalCOM(), wrt_);
}

Eigen::MatrixXd RobotSystem::getBodyNodeCoMJacobianDot(
    const std::string& name_, dart::dynamics::Frame* wrt_) {
    // return skel_ptr_->getJacobianSpatialDeriv(skel_ptr_->getBodyNode(name_),
    // skel_ptr_->getBodyNode(name_)->getLocalCOM(),
    // wrt_);

    return skel_ptr_->getJacobianClassicDeriv(
        skel_ptr_->getBodyNode(name_),
        skel_ptr_->getBodyNode(name_)->getLocalCOM(), wrt_);
}

Eigen::MatrixXd RobotSystem::getBodyNodeCoMJacobianDot(
    const int& _bn_idx, dart::dynamics::Frame* wrt_) {

    // return skel_ptr_->getJacobianSpatialDeriv(
    //     skel_ptr_->getBodyNode(_bn_idx),
    //     skel_ptr_->getBodyNode(_bn_idx)->getLocalCOM(),
    //     wrt_);

    return skel_ptr_->getJacobianClassicDeriv(
        skel_ptr_->getBodyNode(_bn_idx),
        skel_ptr_->getBodyNode(_bn_idx)->getLocalCOM(), wrt_);
}

Eigen::MatrixXd RobotSystem::getBodyNodeCoMBodyJacobian(const std::string& name_) {
    return skel_ptr_->getJacobian(
                            skel_ptr_->getBodyNode(name_),
                            skel_ptr_->getBodyNode(name_)->getLocalCOM());
}

Eigen::MatrixXd RobotSystem::getBodyNodeCoMBodyJacobian(const int& _bn_idx) {
    return skel_ptr_->getJacobian(
                            skel_ptr_->getBodyNode(_bn_idx),
                            skel_ptr_->getBodyNode(_bn_idx)->getLocalCOM());
}

Eigen::MatrixXd RobotSystem::getBodyNodeCoMBodyJacobianDot(const std::string& name_) {
    
    Eigen::MatrixXd Jacob = skel_ptr_->getJacobian(
                            skel_ptr_->getBodyNode(name_),
                            skel_ptr_->getBodyNode(name_)->getLocalCOM());
    
    Eigen::MatrixXd SpatialDeriv = skel_ptr_->getJacobianSpatialDeriv(
                            skel_ptr_->getBodyNode(name_),
                            skel_ptr_->getBodyNode(name_)->getLocalCOM());
    Eigen::MatrixXd ClassicDeriv = skel_ptr_->getJacobianClassicDeriv(
                            skel_ptr_->getBodyNode(name_),
                            skel_ptr_->getBodyNode(name_)->getLocalCOM());

    Eigen::VectorXd JacobVec(Eigen::Map<Eigen::VectorXd>(Jacob.data(), Jacob.cols()*Jacob.rows()));
    Eigen::VectorXd SpatialDerivVec(Eigen::Map<Eigen::VectorXd>(SpatialDeriv.data(), SpatialDeriv.cols()*SpatialDeriv.rows()));
    Eigen::VectorXd ClassicDerivVec(Eigen::Map<Eigen::VectorXd>(ClassicDeriv.data(), ClassicDeriv.cols()*ClassicDeriv.rows()));


    my_utils::saveVector(JacobVec, "JacobVec");
    my_utils::saveVector(SpatialDerivVec, "SpatialDerivVec");
    my_utils::saveVector(ClassicDerivVec, "ClassicDerivVec");
        
    return skel_ptr_->getJacobianClassicDeriv(
                            skel_ptr_->getBodyNode(name_),
                            skel_ptr_->getBodyNode(name_)->getLocalCOM());
}

Eigen::MatrixXd RobotSystem::getBodyNodeCoMBodyJacobianDot(const int& _bn_idx) {
    
        Eigen::MatrixXd Jacob = skel_ptr_->getJacobian(
                            skel_ptr_->getBodyNode(_bn_idx),
                            skel_ptr_->getBodyNode(_bn_idx)->getLocalCOM());
    
    Eigen::MatrixXd SpatialDeriv = skel_ptr_->getJacobianSpatialDeriv(
                            skel_ptr_->getBodyNode(_bn_idx),
                            skel_ptr_->getBodyNode(_bn_idx)->getLocalCOM());
    Eigen::MatrixXd ClassicDeriv = skel_ptr_->getJacobianClassicDeriv(
                            skel_ptr_->getBodyNode(_bn_idx),
                            skel_ptr_->getBodyNode(_bn_idx)->getLocalCOM());

    Eigen::VectorXd JacobVec(Eigen::Map<Eigen::VectorXd>(Jacob.data(), Jacob.cols()*Jacob.rows()));
    Eigen::VectorXd SpatialDerivVec(Eigen::Map<Eigen::VectorXd>(SpatialDeriv.data(), SpatialDeriv.cols()*SpatialDeriv.rows()));
    Eigen::VectorXd ClassicDerivVec(Eigen::Map<Eigen::VectorXd>(ClassicDeriv.data(), ClassicDeriv.cols()*ClassicDeriv.rows()));


    my_utils::saveVector(JacobVec, "JacobVec");
    my_utils::saveVector(SpatialDerivVec, "SpatialDerivVec");
    my_utils::saveVector(ClassicDerivVec, "ClassicDerivVec");
    
    return skel_ptr_->getJacobianClassicDeriv(
                            skel_ptr_->getBodyNode(_bn_idx),
                            skel_ptr_->getBodyNode(_bn_idx)->getLocalCOM());
}

int RobotSystem::getJointIdx(const std::string& jointName_) {
    return skel_ptr_->getJoint(jointName_)->getJointIndexInSkeleton();
}

int RobotSystem::getDofIdx(const std::string& dofName_) {
    return skel_ptr_->getDof(dofName_)->getIndexInSkeleton();
}

void RobotSystem::updateSystem(const Eigen::VectorXd& q_,
                               const Eigen::VectorXd& qdot_,
                               bool isUpdatingCentroid) {
    skel_ptr_->setPositions(q_);
    skel_ptr_->setVelocities(qdot_);
    if (isUpdatingCentroid) _updateCentroidFrame(q_, qdot_);
    skel_ptr_->computeForwardKinematics();
}

void RobotSystem::_updateCentroidFrame(const Eigen::VectorXd& q_,
                                       const Eigen::VectorXd& qdot_) {
    Eigen::MatrixXd Jsp = Eigen::MatrixXd::Zero(6, num_dof_);
    Eigen::VectorXd p_gl = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd R_gl = Eigen::MatrixXd::Zero(3, 3);
    Eigen::VectorXd pCoM_g = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd I = Eigen::MatrixXd::Zero(6, 6);
    Eigen::Isometry3d T_lc = Eigen::Isometry3d::Identity();
    Eigen::MatrixXd AdT_lc = Eigen::MatrixXd::Zero(6, 6);
    I_cent_ = Eigen::MatrixXd::Zero(6, 6);
    J_cent_ = Eigen::MatrixXd::Zero(6, num_dof_);
    A_cent_ = Eigen::MatrixXd::Zero(6, num_dof_);
    pCoM_g = skel_ptr_->getCOM();
    for (int i = 0; i < skel_ptr_->getNumBodyNodes(); ++i) {
        dart::dynamics::BodyNodePtr bn = skel_ptr_->getBodyNode(i);
        Jsp = skel_ptr_->getJacobian(bn);
        p_gl = bn->getWorldTransform().translation();
        R_gl = bn->getWorldTransform().linear();
        I = bn->getSpatialInertia();
        T_lc.linear() = R_gl.transpose();
        T_lc.translation() = R_gl.transpose() * (pCoM_g - p_gl);
        AdT_lc = dart::math::getAdTMatrix(T_lc);
        I_cent_ += AdT_lc.transpose() * I * AdT_lc;
        A_cent_ += AdT_lc.transpose() * I * Jsp;
    }
    J_cent_ = I_cent_.inverse() * A_cent_;
}

void RobotSystem::printRobotInfo() {
    std::cout << " ==== Body Node ====" << std::endl;
    for (int i = 0; i < skel_ptr_->getNumBodyNodes(); ++i) {
        dart::dynamics::BodyNodePtr bn = skel_ptr_->getBodyNode(i);
        std::cout << "constexpr int " << bn->getName() << " = "
                  << std::to_string(i) << ";" << std::endl;
    }
    std::cout << " ==== DoF ====" << std::endl;
    for (int i = 0; i < skel_ptr_->getNumDofs(); ++i) {
        dart::dynamics::DegreeOfFreedom* dof = skel_ptr_->getDof(i);
        std::cout << "constexpr int " << dof->getName() << " = "
                  << std::to_string(i) << ";" << std::endl;
    }
    std::cout << " ==== Num ====" << std::endl;
    std::cout << "constexpr int n_bodynode = "
              << std::to_string(skel_ptr_->getNumBodyNodes()) << ";"
              << std::endl;
    std::cout << "constexpr int n_dof = " << std::to_string(num_dof_) << ";"
              << std::endl;
    std::cout << "constexpr int n_vdof = " << std::to_string(num_virtual_dof_)
              << ";" << std::endl;
    std::cout << "constexpr int n_adof = " << std::to_string(num_actuated_dof_)
              << ";" << std::endl;

    // std::cout << " === MASS ===" << std::endl;
    // for (int i = 0; i < skel_ptr_->getNumBodyNodes(); ++i) {
    //     dart::dynamics::BodyNodePtr bn = skel_ptr_->getBodyNode(i);
    //     std::cout << bn->getName() << " : "
    //               << bn->getMass() << "kg" << std::endl;
    // }

    // std::cout << " === LocalCOM ===" << std::endl;
    // for (int i = 0; i < skel_ptr_->getNumBodyNodes(); ++i) {
    //     dart::dynamics::BodyNodePtr bn = skel_ptr_->getBodyNode(i);
    //     std::cout << bn->getName() << " : "
    //               << bn->getLocalCOM().transpose() << std::endl;
    // }

    // dart::dynamics::Inertia;
    // std::cout << " === Moment ===" << std::endl;
    // for (int i = 0; i < skel_ptr_->getNumBodyNodes(); ++i) {
    //     dart::dynamics::BodyNodePtr bn = skel_ptr_->getBodyNode(i);
    //     std::cout << bn->getName() << " : "
    //               << bn->getInertia().getMoment() << "kg*m2" << std::endl;
    // }


    // AL_depth_sensor_link
    int idx_depth_sensor_link [4] = {9,18,27,36};
    // gazebo_AL_leg_optical_frame
    int idx_leg_optical_frame [4] = {10,19,28,37};

    std::cout << " === MASS ===" << std::endl;
    for (int i = 0; i < 4; ++i) {
        dart::dynamics::BodyNodePtr bn = skel_ptr_->getBodyNode(idx_depth_sensor_link[i]);
        std::cout << bn->getName() << " : "
                  << bn->getMass() << "kg" << std::endl;

        bn = skel_ptr_->getBodyNode(idx_leg_optical_frame[i]);
        std::cout << bn->getName() << " : "
                  << bn->getMass() << "kg" << std::endl;
    }

    std::cout << " === LocalCOM ===" << std::endl;
    for (int i = 0; i < 4; ++i) {
        dart::dynamics::BodyNodePtr bn = skel_ptr_->getBodyNode(idx_depth_sensor_link[i]);
        std::cout << bn->getName() << " : "
                  << bn->getLocalCOM().transpose() << std::endl;

        bn = skel_ptr_->getBodyNode(idx_leg_optical_frame[i]);
        std::cout << bn->getName() << " : "
                  << bn->getLocalCOM().transpose() << std::endl;
    }

    std::cout << " === Moment ===" << std::endl;
    for (int i = 0; i < 4; ++i) {
        dart::dynamics::BodyNodePtr bn = skel_ptr_->getBodyNode(idx_depth_sensor_link[i]);
        std::cout << bn->getName() << " : "
                  << bn->getInertia().getMoment() << " kg*m2" << std::endl;

        bn = skel_ptr_->getBodyNode(idx_leg_optical_frame[i]);
        std::cout << bn->getName() << " : "
                  << bn->getInertia().getMoment() << " kg*m2" << std::endl;

    }
    // exit(0);
    std::cout << " === === === === === === === === ===" << std::endl;
}

// test JE
void RobotSystem::setRobotMass(){
    // AL_depth_sensor_link
    int idx_depth_sensor_link [4] = {9,18,27,36};
    // gazebo_AL_leg_optical_frame
    int idx_leg_optical_frame [4] = {10,19,28,37};

    dart::dynamics::BodyNodePtr bn;

    double ratio = 1e-1;
    double mass_ = ratio*1.;
    Eigen::VectorXd com_ = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd eye3_ = ratio*Eigen::MatrixXd::Identity(3,3); 
    
    dart::dynamics::Inertia InertiaSensor(mass_, com_, eye3_);
    dart::dynamics::Inertia InertiaFrame(mass_, com_, eye3_);

    for(int i=0; i<4; ++i)
    {
        bn = skel_ptr_->getBodyNode(idx_depth_sensor_link[i]);
        bn->setInertia(InertiaSensor);

        bn = skel_ptr_->getBodyNode(idx_leg_optical_frame[i]);
        bn->setInertia(InertiaFrame);
    }
}
