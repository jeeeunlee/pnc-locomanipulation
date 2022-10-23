#pragma once

#include <../my_utils/Configuration.h>
#include <Eigen/Dense>

class ANYmalStateProvider;
class RobotSystem;
class ANYmalSensorData;

class ANYmalStateEstimator {
   public:
    ANYmalStateEstimator(RobotSystem* robot);
    ~ANYmalStateEstimator();

    void Initialization(ANYmalSensorData*);
    void Update(ANYmalSensorData*);

   protected:
    ANYmalStateProvider* sp_;
    RobotSystem* robot_;

    Eigen::Vector3d curr_base_pos_;
    Eigen::Quaterniond curr_base_quat_;
    Eigen::Vector3d curr_base_lin_vel_;
    Eigen::Vector3d curr_base_ang_vel_;

    Eigen::VectorXd curr_vjoint_pos_;
    Eigen::VectorXd curr_vjoint_vel_;
    Eigen::VectorXd curr_joint_pos_;
    Eigen::VectorXd curr_joint_vel_;

    Eigen::VectorXd prev_tau_cmd_;

    void _JointUpdate(ANYmalSensorData* data);
    void _ConfigurationAndModelUpdate();
    void _FootContactUpdate(ANYmalSensorData* data);
};
