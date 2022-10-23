#include <my_robot_system//RobotSystem.hpp>
#include <my_utils/IO/IOUtilities.hpp>
#include <chrono>

RobotSystem::RobotSystem(const RobotSystem& robotsys) 
:RobotSystem(robotsys.b_fixed_base_, robotsys.urdf_file_)  {
}

RobotSystem::RobotSystem(bool b_fixed_base_, 
                        const std::string& file, 
                        int n_pdof): n_pdof_(n_pdof) {
    my_utils::pretty_constructor(1, "Robot Model");
    urdf_file_ = file;

    // set pinocchio model & data
    if(b_fixed_base_){
        pinocchio::urdf::buildModel(urdf_file_, model_);
        n_float_ = 0;
    }else{
        pinocchio::urdf::buildModel(urdf_file_, 
              pinocchio::JointModelFreeFlyer(), model_);
        n_float_ = 6;
    }
    data_ = pinocchio::Data(model_);
    n_q_ = model_.nq;
    n_qdot_ = model_.nv;
    n_adof_ = n_qdot_ - n_float_ - n_pdof_;

    I_cent_ = Eigen::MatrixXd::Zero(6, 6);
    J_cent_ = Eigen::MatrixXd::Zero(6, n_qdot_);
    A_cent_ = Eigen::MatrixXd::Zero(6, n_qdot_);
    H_cent_ = Eigen::MatrixXd::Zero(6, 1);

    q_ = Eigen::VectorXd::Zero(n_q_);
    qdot_ = Eigen::VectorXd::Zero(n_qdot_);
    qddot_ = Eigen::VectorXd::Zero(n_qdot_);
    
    setActuatedJoint();
    _initializeRobotInfo();
    // printRobotInfo();
}

RobotSystem::~RobotSystem() {}

void RobotSystem::setActuatedJoint()  {
    idx_adof_.clear();
    for(int i=0;i<n_adof_;++i)    
        idx_adof_.push_back(n_float_ + i); 
}
void RobotSystem::setActuatedJoint(const int *_idx_adof)  {
    idx_adof_.clear();
    for(int i=0;i<n_adof_;++i)    
        idx_adof_.push_back(_idx_adof[i]);  
}

Eigen::VectorXd RobotSystem::getActiveJointValue(const Eigen::VectorXd& q_full)  {
    Eigen::VectorXd q_a(n_adof_);
    for(int i = 0; i < n_adof_; ++i)        
        q_a[i] = q_full[idx_adof_[i]]; 
    return  q_a;
}

Eigen::VectorXd RobotSystem::GetTorqueLowerLimits() {
    if (b_fixed_base_) return -model_.effortLimit;
    else return -model_.effortLimit.segment(n_float_, n_adof_);
}
Eigen::VectorXd RobotSystem::GetTorqueUpperLimits() {
    if (b_fixed_base_) return model_.effortLimit;
    else return model_.effortLimit.segment(n_float_, n_adof_);
}
Eigen::VectorXd RobotSystem::GetPositionLowerLimits() {
    if (b_fixed_base_) return model_.lowerPositionLimit;
    else return model_.lowerPositionLimit.segment(n_float_, n_adof_);
}
Eigen::VectorXd RobotSystem::GetPositionUpperLimits() {
    if (b_fixed_base_) return model_.upperPositionLimit;
    else return model_.upperPositionLimit.segment(n_float_, n_adof_);
}

Eigen::MatrixXd RobotSystem::getMassMatrix() {
    data_.M.triangularView<Eigen::StrictlyLower>() =
        data_.M.transpose().triangularView<Eigen::StrictlyLower>();
    return data_.M;
}

Eigen::MatrixXd RobotSystem::getInvMassMatrix() {    
    data_.Minv.triangularView<Eigen::StrictlyLower>() =
        data_.Minv.transpose().triangularView<Eigen::StrictlyLower>();
    return data_.Minv;
}

Eigen::VectorXd RobotSystem::getCoriolisGravity() {
    return pinocchio::nonLinearEffects(model_, data_, q_, qdot_);
}

Eigen::VectorXd RobotSystem::getCoriolis() {
    return pinocchio::nonLinearEffects(model_, data_, q_, qdot_) -
         pinocchio::computeGeneralizedGravity(model_, data_, q_);
}

Eigen::VectorXd RobotSystem::getGravity() {
    return pinocchio::computeGeneralizedGravity(model_, data_, q_);
}

Eigen::MatrixXd RobotSystem::getCentroidJacobian() const { return this->J_cent_; }
Eigen::MatrixXd RobotSystem::getCentroidInertiaTimesJacobian() const  { return this->A_cent_; }
Eigen::MatrixXd RobotSystem::getCentroidInertia() const { return this->I_cent_; }
Eigen::VectorXd RobotSystem::getCentroidMomentum() const { return this->H_cent_; }

Eigen::Vector3d RobotSystem::getCoMPosition() { return data_.com[0];}
Eigen::Vector3d RobotSystem::getCoMVelocity() { return data_.vcom[0];}
Eigen::Vector3d RobotSystem::getCoMAcceleration() { return data_.acom[0];}
Eigen::MatrixXd RobotSystem::getCoMJacobian() {
    pinocchio::jacobianCenterOfMass(model_, data_, q_);
    return data_.Jcom;
}

Eigen::Isometry3d RobotSystem::getBodyNodeIsometry(const std::string& name) {
    return this->getBodyNodeIsometry(link_idx_map_[name]); }

Eigen::Isometry3d RobotSystem::getBodyNodeIsometry(const int& link_idx) {
    Eigen::Isometry3d ret;
    const pinocchio::SE3 trans =
        pinocchio::updateFramePlacement(model_, data_, link_idx);
    ret.linear() = trans.rotation();
    ret.translation() = trans.translation();
    return ret;
}

Eigen::Matrix<double, 6, 1> RobotSystem::getBodyNodeSpatialVelocity(const std::string& name) {
    return this->getBodyNodeSpatialVelocity(link_idx_map_[name]); }

Eigen::Matrix<double, 6, 1> RobotSystem::getBodyNodeSpatialVelocity(const int& link_idx) {
  Eigen::Matrix<double, 6, 1> ret = Eigen::Matrix<double, 6, 1>::Zero();
  pinocchio::Motion fv = pinocchio::getFrameVelocity(
      model_, data_, link_idx, pinocchio::LOCAL_WORLD_ALIGNED);
  ret.head<3>() = fv.angular();
  ret.tail<3>() = fv.linear();
  return ret;
}

Eigen::MatrixXd RobotSystem::getBodyNodeJacobian(const std::string& name) {
    return this->getBodyNodeJacobian(link_idx_map_[name]); }


Eigen::MatrixXd RobotSystem::getBodyNodeJacobian(const int& link_idx) {
    // Analytic Jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jac =
      Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, n_qdot_);
  pinocchio::getFrameJacobian(model_, data_, link_idx,
                              pinocchio::LOCAL_WORLD_ALIGNED, jac);
  Eigen::Matrix<double, 6, Eigen::Dynamic> ret =
      Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, n_qdot_);
  ret.topRows(3) = jac.bottomRows(3);
  ret.bottomRows(3) = jac.topRows(3);
  return ret;
}

Eigen::MatrixXd RobotSystem::getBodyNodeJacobianDotQDot(const std::string& name) {
    return this->getBodyNodeJacobianDotQDot(link_idx_map_[name]); }

Eigen::MatrixXd RobotSystem::getBodyNodeJacobianDotQDot(const int& link_idx) {
    // check 
    // pinocchio::Motion fa = pinocchio::getFrameAcceleration(
    //     model_, data_, link_idx, pinocchio::LOCAL_WORLD_ALIGNED);

    // pinocchio::forwardKinematics(model_, data_, q_, qdot_, 0 * qdot_);
    pinocchio::Motion fa = pinocchio::getFrameClassicalAcceleration(
        model_, data_, link_idx, pinocchio::LOCAL_WORLD_ALIGNED);

    Eigen::Matrix<double, 6, 1> ret = Eigen::Matrix<double, 6, 1>::Zero();
    ret.segment(0, 3) = fa.angular();
    ret.segment(3, 3) = fa.linear();

    return ret;
}

Eigen::Matrix<double, 6, 1> RobotSystem::getBodyNodeBodyVelocity(const std::string& name) {
    return this->getBodyNodeBodyVelocity(link_idx_map_[name]);
}
Eigen::Matrix<double, 6, 1> RobotSystem::getBodyNodeBodyVelocity(const int& link_idx) {

}


Eigen::MatrixXd RobotSystem::getBodyNodeBodyJacobian(const std::string& name) {
    return this->getBodyNodeBodyJacobian(link_idx_map_[name]);
}

Eigen::MatrixXd RobotSystem::getBodyNodeBodyJacobian(const int& link_idx) {
  Eigen::Matrix<double, 6, Eigen::Dynamic> jac =
      Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, n_qdot_);
  pinocchio::getFrameJacobian(model_, data_, link_idx, pinocchio::LOCAL, jac);
  Eigen::Matrix<double, 6, Eigen::Dynamic> ret =
      Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, n_qdot_);
  ret.topRows(3) = jac.bottomRows(3);
  ret.bottomRows(3) = jac.topRows(3);
  return ret;
}

Eigen::MatrixXd RobotSystem::getBodyNodeBodyJacobianDotQDot(const std::string& name) {
        return this->getBodyNodeBodyJacobianDotQDot(link_idx_map_[name]);
}

Eigen::MatrixXd RobotSystem::getBodyNodeBodyJacobianDotQDot(const int& link_idx) {
    // pinocchio::forwardKinematics(model_, data_, q_, qdot_, 0 * qdot_);
    pinocchio::Motion fa = pinocchio::getFrameClassicalAcceleration(
        model_, data_, link_idx, pinocchio::LOCAL);

    Eigen::Matrix<double, 6, 1> ret = Eigen::Matrix<double, 6, 1>::Zero();
    ret.segment(0, 3) = fa.angular();
    ret.segment(3, 3) = fa.linear();

    return ret;
}

int RobotSystem::getLinkIdx(const std::string& frame_name) {    
    return link_idx_map_[frame_name]; }

int RobotSystem::getJointIdx(const std::string& jointName) {    
    return joint_idx_map_[jointName]; }

std::string RobotSystem::getLinkName(const int& frame_idx) {    
    return link_idx_map_inv_[frame_idx]; }

std::string RobotSystem::getJointName(const int& joint_idx) {    
    return joint_idx_map_inv_[joint_idx]; }

void RobotSystem::updateSystem(
    const Eigen::Vector3d &base_joint_pos, const Eigen::Quaterniond &base_joint_quat,
    const Eigen::Vector3d &base_joint_lin_vel, const Eigen::Vector3d &base_joint_ang_vel, 
    const Eigen::VectorXd &joint_pos, const Eigen::VectorXd &joint_vel, 
    bool b_update_centroid) {
    // ASSUME FLOATING BASE
    q_.segment(0, 3) = base_joint_pos;
    q_.segment(3, 4) << base_joint_quat.normalized().coeffs(); // x,y,z,w order
    q_.tail(n_q_ - 7) = joint_pos;

    // Eigen::Matrix3d rot_w_basejoint =
    //     base_joint_quat.normalized().toRotationMatrix();
    // qdot_.segment(0, 3) = rot_w_basejoint.transpose() * base_joint_lin_vel;
    // qdot_.segment(3, 3) = rot_w_basejoint.transpose() * base_joint_ang_vel;
    qdot_.segment(0, 3) =  base_joint_lin_vel;
    qdot_.segment(3, 3) = base_joint_ang_vel;
    qdot_.tail(n_qdot_ - n_float_) = joint_vel;

    // update data
    _updateSystemData();
    if (b_update_centroid) _updateCentroidFrame();
}

void RobotSystem::updateSystem(const Eigen::VectorXd &joint_pos,
                                const Eigen::VectorXd &joint_vel,
                                bool b_update_centroid){
    // ASSUME FLOATING BASE
    Eigen::Quaterniond base_joint_quat;
    base_joint_quat.coeffs() = joint_pos.segment(3, 4);
    Eigen::Matrix3d rot_w_basejoint =
        base_joint_quat.normalized().toRotationMatrix();
    Eigen::Vector3d base_joint_lin_vel = joint_vel.segment(0, 3);
    Eigen::Vector3d base_joint_ang_vel = joint_vel.segment(3, 3);

    q_ = joint_pos;
    qdot_ = joint_vel;

    qdot_.segment(0, 3) = rot_w_basejoint.transpose() * base_joint_lin_vel;
    qdot_.segment(3, 3) = rot_w_basejoint.transpose() * base_joint_ang_vel;
    

    // update data
    _updateSystemData();
    if (b_update_centroid) _updateCentroidFrame();
}

void RobotSystem::updateSystem(const Eigen::VectorXd &joint_pos,
                                const Eigen::VectorXd &joint_vel) {
    // ASSUME FIXED BASE
    q_ = joint_pos;
    qdot_ = joint_vel;

    // update data
    _updateSystemData();
}

void RobotSystem::_updateSystemData(){
    pinocchio::crba(model_, data_, q_);
    pinocchio::forwardKinematics(model_, data_, q_, qdot_, 0 * qdot_);
    pinocchio::centerOfMass(model_, data_, q_, qdot_);
    pinocchio::computeJointJacobians(model_, data_, q_);
}

void RobotSystem::_updateCentroidFrame() {

    pinocchio::ccrba(model_, data_, q_, qdot_);

    I_cent_.block<3, 3>(0, 0) = data_.Ig.matrix().block<3, 3>(3, 3);
    I_cent_.block<3, 3>(3, 3) = data_.Ig.matrix().block<3, 3>(0, 0);

    H_cent_.segment(0, 3) = data_.hg.angular();
    H_cent_.segment(3, 3) = data_.hg.linear();

    A_cent_.topRows(3) = data_.Ag.bottomRows(3);
    A_cent_.bottomRows(3) = data_.Ag.topRows(3);

    J_cent_ = I_cent_.inverse() * A_cent_;
}

void RobotSystem::_initializeRobotInfo() {
    // UPDATE link_idx_map_, joint_idx_map_
    for (pinocchio::FrameIndex i(0); // FrameIndex : size_t
        i < static_cast<pinocchio::FrameIndex>(model_.nframes); ++i) {
        if(model_.frames[i].type == pinocchio::FrameType::BODY){
            std::string frame_name = model_.frames[i].name;
            link_idx_map_[frame_name] = model_.getBodyId(frame_name);
            link_idx_map_inv_[model_.getBodyId(frame_name)] = frame_name;             
        }
    }
    for (pinocchio::JointIndex i(0);
        i < static_cast<pinocchio::JointIndex>(model_.njoints); ++i) {
        total_mass_ += model_.inertias[i].mass();
        std::string joint_name = model_.names[i];
        if (joint_name != "universe" && joint_name != "root_joint"){
            joint_idx_map_[joint_name] = model_.getJointId(joint_name); // i - 2; joint map excluding fixed joint
            joint_idx_map_inv_[model_.getJointId(joint_name)] = joint_name; // joint map excluding fixed joint
        }
            
    }
    n_link_ = link_idx_map_.size();
    assert(n_adof_ == joint_idx_map_.size());

}

void RobotSystem::printRobotInfo() {
    
    std::cout << " ==== Body Node ====" << std::endl;
    for (auto &[idx, name] : link_idx_map_inv_) {
        std::cout << "constexpr int " << name  << " = "
                  << std::to_string(idx) << ";" << std::endl;
    }
    std::cout << " ==== DoF ====" << std::endl;
    for (auto &[idx, name] : joint_idx_map_inv_) {
        std::cout << "constexpr int " << name << " = " 
                  << std::to_string(idx) << ";" << std::endl;
    }
    std::cout << " ==== Num ====" << std::endl;
    std::cout << "constexpr int n_bodynode = "
              << std::to_string(n_link_) << ";"
              << std::endl;
    std::cout << "constexpr int n_dof = " << std::to_string(n_qdot_) << ";"
              << std::endl;
    std::cout << "constexpr int n_vdof = " << std::to_string(n_pdof_+n_float_)
              << ";" << std::endl;
    std::cout << "constexpr int n_adof = " << std::to_string(n_adof_)
              << ";" << std::endl;

    std::cout << " ==== Just info ====" << std::endl;
    std::cout << n_link_ <<", "  // 86
            << n_q_ << ", "  // 25 7 + 12 + 6
            << n_qdot_ <<", "  // 24  6 + 12 + 6
            << n_pdof_ << ", "  // 0
            << n_float_ <<", " // 6
            << n_adof_ << std::endl; // 18 = 12 + 6

}


