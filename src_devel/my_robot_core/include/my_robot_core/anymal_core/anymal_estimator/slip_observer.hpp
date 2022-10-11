#pragma once

#include <my_robot_core/anymal_core/anymal_state_provider.hpp>
#include <my_robot_core/state_estimator.hpp>
#include <my_utils/Math/low_pass_filter.h>
#include <my_utils/Math/simple_kalman_filter.h>
#include <deque>

class ANYmalWbcSpecContainer;
class ANYmalReferenceGeneratorContainer;

class SlipObserverData {
    public:
    SlipObserverData() {

    }
    ~SlipObserverData() {}

};

class SlipObserver : public StateEstimator {
  public:
    SlipObserver( ANYmalWbcSpecContainer* ws_container,
                RobotSystem* _robot);
    ~SlipObserver();
    void evaluate();
    void initialization(const YAML::Node& node);

    void checkVelocity();   
    void checkVelocityFoot(int foot_idx);
    bool estimateParameters();
     
    void checkForce();
    Eigen::VectorXd computeGRFDesired(const Eigen::VectorXd& tau);

    void weightShaping();

    void initContact();
    void initParams();
    void updateContact();
  
  public:
    int weight_shaping_activated_;
    int online_param_estimation_activated_;
    double lin_vel_thres_;
    

  protected:
    ANYmalStateProvider* sp_;
    ANYmalWbcSpecContainer* ws_container_;

    double t_updated_;
    bool b_swing_phase_;
    int swing_foot_idx_;
    
    std::map<int, bool> b_foot_contact_map_;
    std::map<int, int> dim_grf_map_;
    std::map<int, Eigen::VectorXd> foot_vel_map_;
    std::map<int, Eigen::VectorXd> foot_acc_map_;

    std::map<int, Eigen::VectorXd> grf_act_map_;
    std::map<int, Eigen::VectorXd> grf_des_map_;
    std::map<int, Eigen::VectorXd> grf_des_map2_;

    Eigen::MatrixXd Sa_;

    Eigen::MatrixXd M_;
    Eigen::MatrixXd Minv_;
    Eigen::MatrixXd grav_;
    Eigen::MatrixXd coriolis_;

    Eigen::VectorXd q_;
    Eigen::VectorXd qdot_;
    Eigen::VectorXd qddot_;  

    std::array<LowPassFilter2*, ANYmal::n_leg> lpf2_container_;    
    double lpf_vel_cutoff_;

    // parameter estimation
    int time_sampling_period_;
    std::array<std::deque<Eigen::VectorXd>, ANYmal::n_leg> stacked_grf_map_;
    std::array<SimpleKalmanFilter*, ANYmal::n_leg> kf_container_;
    SimpleSystemParam* kf_sys_;


    
};