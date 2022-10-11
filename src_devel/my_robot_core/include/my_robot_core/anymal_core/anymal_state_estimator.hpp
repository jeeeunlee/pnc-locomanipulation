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

    Eigen::VectorXd curr_config_;
    Eigen::VectorXd curr_qdot_;
    Eigen::VectorXd prev_tau_cmd_;

    void _JointUpdate(ANYmalSensorData* data);
    void _ConfigurationAndModelUpdate();
    void _FootContactUpdate(ANYmalSensorData* data);
};
