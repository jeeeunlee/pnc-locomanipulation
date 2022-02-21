#pragma once

#include <my_robot_core/magneto_core/magneto_state_provider.hpp>
#include <my_robot_core/state_estimator.hpp>
#include <my_utils/Math/low_pass_filter.h>

class MagnetoWbcSpecContainer;
class MagnetoReferenceGeneratorContainer;

class SlipObserverData {
    public:
    SlipObserverData() {

    }
    ~SlipObserverData() {}

};

class SlipObserver : public StateEstimator {
  public:
    SlipObserver( MagnetoWbcSpecContainer* ws_container,
                RobotSystem* _robot);
    ~SlipObserver();
    void evaluate();
    void initialization(const YAML::Node& node);

    void checkVelocity();   
    void checkVelocityFoot(int foot_idx);
     
    void checkForce();
    Eigen::VectorXd computeGRFDesired(const Eigen::VectorXd& tau);

    void weightShaping();

    void initContact();
    void initParams();
    void updateContact();
  
  public:
    int weight_shaping_activated_;
    double lin_vel_thres_;
    

  protected:
    MagnetoStateProvider* sp_;
    MagnetoWbcSpecContainer* ws_container_;

    double t_updated_;
    bool b_swing_phase_;
    int swing_foot_link_idx_;
    
    std::map<int, bool> b_foot_contact_map_;
    std::map<int, int> dim_grf_map_;
    std::map<int, Eigen::VectorXd> foot_vel_map_;
    std::map<int, Eigen::VectorXd> foot_acc_map_;

    std::map<int, Eigen::VectorXd> grf_act_map_;
    std::map<int, Eigen::VectorXd> grf_des_map_;

    Eigen::MatrixXd Sa_;

    Eigen::MatrixXd M_;
    Eigen::MatrixXd Minv_;
    Eigen::MatrixXd grav_;
    Eigen::MatrixXd coriolis_;

    Eigen::VectorXd q_;
    Eigen::VectorXd qdot_;
    Eigen::VectorXd qddot_;  

    std::array<LowPassFilter2*, Magneto::n_leg> lpf2_container_;
    double lpf_vel_cutoff_;
    
};