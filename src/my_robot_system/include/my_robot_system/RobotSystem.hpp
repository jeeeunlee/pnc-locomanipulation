#pragma once

#include <stdio.h>
#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

#include <my_utils/IO/IOUtilities.hpp>

class RobotSystem {
   protected:
    std::string skel_file_name_;
    dart::dynamics::SkeletonPtr skel_ptr_;
    int num_dof_;
    int num_virtual_dof_;
    int num_actuated_dof_;
    int num_body_nodes_;
    std::vector<int> idx_adof_;
    Eigen::MatrixXd I_cent_;
    Eigen::MatrixXd J_cent_;
    Eigen::MatrixXd A_cent_;

    /*
     * Update I_cent_, A_cent_, J_cent_
     * , where
     * centroid_momentum = I_cent_ * centroid_velocity = A_cent_ * qdot
     *           J_cent_ = inv(I_cent_) * A_cent_
     * centroid_velocity = J_cent_ * qdot
     */
    void _updateCentroidFrame(const Eigen::VectorXd& q_,
                              const Eigen::VectorXd& qdot_);

   public:
    RobotSystem(const RobotSystem& robotsys);
    RobotSystem(int numVirtual_, std::string file);
    virtual ~RobotSystem(void);

    void setActuatedJoint(); // DEFAULT : assume the last num_actuated_dof_ is adof
    void setActuatedJoint(const int *_idx_adof);
    void getActuatedJointIdx(std::vector<int> & _idx_adof ) { _idx_adof=idx_adof_; };

    std::string getFileName() { return skel_file_name_; };
    dart::dynamics::SkeletonPtr getSkeleton() { return skel_ptr_; };
    dart::dynamics::BodyNodePtr getBodyNode(const std::string& _link_name) {
        return skel_ptr_->getBodyNode(_link_name);
    }
    dart::dynamics::BodyNodePtr getBodyNode(const int& _bn_idx) {
        return skel_ptr_->getBodyNode(_bn_idx);
    }

    Eigen::VectorXd getQ() { return skel_ptr_->getPositions(); };
    Eigen::VectorXd getQdot() { return skel_ptr_->getVelocities(); };
    Eigen::VectorXd getQddot() { return skel_ptr_->getAccelerations(); };

    Eigen::VectorXd getActiveQ() { return getActiveJointValue(skel_ptr_->getPositions()); };
    Eigen::VectorXd getActiveQdot() { return getActiveJointValue(skel_ptr_->getVelocities()); };
    Eigen::VectorXd getActiveQddot() { return getActiveJointValue(skel_ptr_->getAccelerations()); };

    Eigen::VectorXd getActiveJointValue(const Eigen::VectorXd& q_full);

    // JE test for magneto
    void setRobotMass();

    void printRobotInfo();
    double getRobotMass() { return skel_ptr_->getMass(); }
    int getNumDofs() { return num_dof_; };
    int getNumVirtualDofs() { return num_virtual_dof_; };
    int getNumActuatedDofs() { return num_actuated_dof_; };
    int getNumBodyNodes() { return num_body_nodes_; };

    int getJointIdx(const std::string& jointName_);
    int getDofIdx(const std::string& dofName_);

    Eigen::VectorXd GetTorqueLowerLimits() {
        return skel_ptr_->getForceLowerLimits();
    }
    Eigen::VectorXd GetTorqueUpperLimits() {
        return skel_ptr_->getForceUpperLimits();
    }
    Eigen::VectorXd GetPositionLowerLimits() {
        return skel_ptr_->getPositionLowerLimits();
    }
    Eigen::VectorXd GetPositionUpperLimits() {
        return skel_ptr_->getPositionUpperLimits();
    }

    Eigen::MatrixXd getMassMatrix();
    Eigen::MatrixXd getInvMassMatrix();
    Eigen::VectorXd getGravity();
    Eigen::VectorXd getCoriolis();
    Eigen::VectorXd getCoriolisGravity();

    Eigen::MatrixXd getCentroidJacobian();
    Eigen::MatrixXd getCentroidInertiaTimesJacobian();
    Eigen::MatrixXd getCentroidInertia();
    Eigen::VectorXd getCentroidVelocity();
    Eigen::VectorXd getCentroidMomentum();
    Eigen::Vector3d getCoMPosition(
        dart::dynamics::Frame* wrt_ = dart::dynamics::Frame::World());
    Eigen::Vector3d getCoMVelocity(
        dart::dynamics::Frame* rl_ = dart::dynamics::Frame::World(),
        dart::dynamics::Frame* wrt_ = dart::dynamics::Frame::World());
    Eigen::MatrixXd getCoMJacobian(
        dart::dynamics::Frame* wrt_ = dart::dynamics::Frame::World());
    void updateSystem(const Eigen::VectorXd& q_, const Eigen::VectorXd& qdot_,
                      bool isUpdatingCentroid_ = true);

    Eigen::Isometry3d getBodyNodeIsometry(
        const std::string& name_,
        dart::dynamics::Frame* wrt_ = dart::dynamics::Frame::World());
    Eigen::Isometry3d getBodyNodeCoMIsometry(
        const std::string& name_,
        dart::dynamics::Frame* wrt_ = dart::dynamics::Frame::World());
    Eigen::Vector6d getBodyNodeSpatialVelocity(
        const std::string& name_,
        dart::dynamics::Frame* rl_ = dart::dynamics::Frame::World(),
        dart::dynamics::Frame* wrt_ = dart::dynamics::Frame::World());
    Eigen::Vector6d getBodyNodeCoMSpatialVelocity(
        const std::string& name_,
        dart::dynamics::Frame* rl_ = dart::dynamics::Frame::World(),
        dart::dynamics::Frame* wrt_ = dart::dynamics::Frame::World());
    Eigen::MatrixXd getBodyNodeJacobian(
        const std::string& name_,
        Eigen::Vector3d localOffset_ = Eigen::Vector3d::Zero(3),
        dart::dynamics::Frame* wrt_ = dart::dynamics::Frame::World());
    Eigen::MatrixXd getBodyNodeJacobianDot(
        const std::string& name_,
        Eigen::Vector3d localOffset_ = Eigen::Vector3d::Zero(3),
        dart::dynamics::Frame* wrt_ = dart::dynamics::Frame::World());
    Eigen::MatrixXd getBodyNodeCoMJacobian(
        const std::string& name_,
        dart::dynamics::Frame* wrt_ = dart::dynamics::Frame::World());
    Eigen::MatrixXd getBodyNodeCoMJacobianDot(
        const std::string& name_,
        dart::dynamics::Frame* wrt_ = dart::dynamics::Frame::World());
    Eigen::MatrixXd getBodyNodeCoMBodyJacobian(const std::string& name_);
    Eigen::MatrixXd getBodyNodeCoMBodyJacobianDot(const std::string& name_);

    Eigen::Isometry3d getBodyNodeIsometry(
        const int& _bn_idx,
        dart::dynamics::Frame* wrt_ = dart::dynamics::Frame::World());
    Eigen::Isometry3d getBodyNodeCoMIsometry(
        const int& _bn_idx,
        dart::dynamics::Frame* wrt_ = dart::dynamics::Frame::World());
    Eigen::Vector6d getBodyNodeSpatialVelocity(
        const int& _bn_idx,
        dart::dynamics::Frame* rl_ = dart::dynamics::Frame::World(),
        dart::dynamics::Frame* wrt_ = dart::dynamics::Frame::World());
    Eigen::Vector6d getBodyNodeCoMSpatialVelocity(
        const int& _bn_idx,
        dart::dynamics::Frame* rl_ = dart::dynamics::Frame::World(),
        dart::dynamics::Frame* wrt_ = dart::dynamics::Frame::World());
    Eigen::MatrixXd getBodyNodeJacobian(
        const int& _bn_idx,
        Eigen::Vector3d localOffset_ = Eigen::Vector3d::Zero(3),
        dart::dynamics::Frame* wrt_ = dart::dynamics::Frame::World());
    Eigen::MatrixXd getBodyNodeJacobianDot(
        const int& _bn_idx,
        Eigen::Vector3d localOffset_ = Eigen::Vector3d::Zero(3),
        dart::dynamics::Frame* wrt_ = dart::dynamics::Frame::World());
    Eigen::MatrixXd getBodyNodeBodyJacobian(
        const int& _bn_idx,
        Eigen::Vector3d localOffset_ = Eigen::Vector3d::Zero(3));
    Eigen::MatrixXd getBodyNodeBodyJacobianDot(
        const int& _bn_idx,
        Eigen::Vector3d localOffset_ = Eigen::Vector3d::Zero(3));
    Eigen::MatrixXd getBodyNodeCoMJacobian(
        const int& _bn_idx,
        dart::dynamics::Frame* wrt_ = dart::dynamics::Frame::World());
    Eigen::MatrixXd getBodyNodeCoMJacobianDot(
        const int& _bn_idx,
        dart::dynamics::Frame* wrt_ = dart::dynamics::Frame::World());
    Eigen::MatrixXd getBodyNodeCoMBodyJacobian(const int& _bn_idx);
    Eigen::MatrixXd getBodyNodeCoMBodyJacobianDot(const int& _bn_idx);
};
