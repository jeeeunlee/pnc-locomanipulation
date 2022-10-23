#pragma once

#include <stdio.h>
#include <Eigen/Dense>
#include <string>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>



class RobotSystem {
   protected:
    // pinocchio system
    pinocchio::Model model_;
    pinocchio::Data data_;

    std::string urdf_file_;
    bool b_fixed_base_;
    bool b_print_info_;

    // generalized coordinate configuration
    Eigen::VectorXd q_;
    Eigen::VectorXd qdot_;
    Eigen::VectorXd qddot_;

    // robot info
    double total_mass_;
    int n_q_;
    int n_qdot_;
    int n_adof_; // n_qdot_ - (n_pdof_ +n_float_)
    int n_pdof_;
    int n_float_;
    int n_link_;
    std::vector<int> idx_adof_;
    std::map<std::string, size_t> link_idx_map_;
    std::map<std::string, size_t> joint_idx_map_;
    std::map<size_t, std::string> link_idx_map_inv_;
    std::map<size_t, std::string> joint_idx_map_inv_; 
    
    /* CEMNTROIDAL DYNAMICS QUANTITIES
     * Update I_cent_, A_cent_, J_cent_, H_cent_
     * , where
     * centroid_momentum = I_cent_ * centroid_velocity = A_cent_ * qdot
     *           J_cent_ = inv(I_cent_) * A_cent_
     * centroid_velocity = J_cent_ * qdot
     */
    Eigen::MatrixXd I_cent_;
    Eigen::MatrixXd J_cent_;
    Eigen::MatrixXd A_cent_;   
    Eigen::VectorXd H_cent_;
    

   public:
    RobotSystem(const RobotSystem& robotsys); // copier
    RobotSystem(bool b_fixed_base, const std::string& file, int n_pdof=0);
    virtual ~RobotSystem(void);

    void printRobotInfo();    

    // update floating base system
    void updateSystem(const Eigen::Vector3d &base_joint_pos,
                    const Eigen::Quaterniond &base_joint_quat,
                    const Eigen::Vector3d &base_joint_lin_vel,
                    const Eigen::Vector3d &base_joint_ang_vel, 
                    const Eigen::VectorXd &joint_pos,
                    const Eigen::VectorXd &joint_vel, 
                    bool b_update_centroid);
    void updateSystem(const Eigen::VectorXd &joint_pos,
                    const Eigen::VectorXd &joint_vel,
                    bool b_update_centroid);
    // update fixed base system
    void updateSystem(const Eigen::VectorXd &joint_pos,
                    const Eigen::VectorXd &joint_vel);

    void setActuatedJoint(); // DEFAULT : assume the last num_actuated_dof_ is adof
    void setActuatedJoint(const int *_idx_adof);
    void getActuatedJointIdx(std::vector<int> & _idx_adof ) { _idx_adof=idx_adof_; };
    Eigen::VectorXd getActiveJointValue(const Eigen::VectorXd& q_full);

    Eigen::VectorXd getQ() { return q_; };
    Eigen::VectorXd getQdot() { return qdot_; };
    Eigen::VectorXd getQddot() { return qddot_; };

    Eigen::VectorXd getActiveQ() { return getActiveJointValue(q_); };
    Eigen::VectorXd getActiveQdot() { return getActiveJointValue(qdot_); };
    Eigen::VectorXd getActiveQddot() { return getActiveJointValue(qddot_); };    

    
    double getRobotMass() { return total_mass_; }
    int getNumDofs() { return n_qdot_; };
    int getNumActuatedDofs() { return n_adof_; };
    int getNumBodyNodes() { return n_link_; };

    int getJointIdx(const std::string& joint_name);
    int getLinkIdx(const std::string& frame_name);
    std::string getLinkName(const int& frame_idx);
    std::string getJointName(const int& joint_idx); 

    Eigen::VectorXd GetTorqueLowerLimits();
    Eigen::VectorXd GetTorqueUpperLimits();
    Eigen::VectorXd GetPositionLowerLimits();
    Eigen::VectorXd GetPositionUpperLimits();

    Eigen::MatrixXd getMassMatrix();
    Eigen::MatrixXd getInvMassMatrix();
    Eigen::VectorXd getCoriolisGravity();
    Eigen::VectorXd getGravity();
    Eigen::VectorXd getCoriolis();

    Eigen::MatrixXd getCentroidJacobian() const;
    Eigen::MatrixXd getCentroidInertiaTimesJacobian() const;
    Eigen::MatrixXd getCentroidInertia() const;
    Eigen::VectorXd getCentroidMomentum() const;

    Eigen::Vector3d getCoMPosition();
    Eigen::Vector3d getCoMVelocity();
    Eigen::Vector3d getCoMAcceleration();
    Eigen::MatrixXd getCoMJacobian();

    Eigen::Isometry3d getBodyNodeIsometry(const std::string& name_);
    Eigen::Matrix<double, 6, 1> getBodyNodeSpatialVelocity(const std::string& name_);    
    Eigen::MatrixXd getBodyNodeJacobian(const std::string& name_);
    Eigen::MatrixXd getBodyNodeJacobianDotQDot(const std::string& name_);
    Eigen::Matrix<double, 6, 1> getBodyNodeBodyVelocity(const std::string& name_);
    Eigen::MatrixXd getBodyNodeBodyJacobian(const std::string& name_);
    Eigen::MatrixXd getBodyNodeBodyJacobianDotQDot(const std::string& name_);

    Eigen::Isometry3d getBodyNodeIsometry(const int& _bn_idx);
    Eigen::Matrix<double, 6, 1> getBodyNodeSpatialVelocity(const int& _bn_idx);    
    Eigen::MatrixXd getBodyNodeJacobian(const int& _bn_idx);
    Eigen::MatrixXd getBodyNodeJacobianDotQDot(const int& _bn_idx);
    Eigen::Matrix<double, 6, 1> getBodyNodeBodyVelocity(const int& _bn_idx);
    Eigen::MatrixXd getBodyNodeBodyJacobian(const int& _bn_idx);
    Eigen::MatrixXd getBodyNodeBodyJacobianDotQDot(const int& _bn_idx);


  private:
    void _initializeRobotInfo();
    void _updateCentroidFrame();
    void _updateSystemData();
};
