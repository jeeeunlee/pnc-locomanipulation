#pragma once

#include <../my_utils/Configuration.h>
#include <Eigen/Dense>

class MagnetoStateProvider;
class RobotSystem;
class MagnetoSensorData;

class MagnetoStateEstimator {
   public:
    MagnetoStateEstimator(RobotSystem* robot);
    ~MagnetoStateEstimator();

    void Initialization(MagnetoSensorData*);
    void Update(MagnetoSensorData*);

   protected:
    MagnetoStateProvider* sp_;
    RobotSystem* robot_;

    Eigen::VectorXd curr_config_;
    Eigen::VectorXd curr_qdot_;
    Eigen::VectorXd prev_tau_cmd_;

    void _JointUpdate(MagnetoSensorData* data);
    void _ConfigurationAndModelUpdate();
    void _FootContactUpdate(MagnetoSensorData* data);
};
