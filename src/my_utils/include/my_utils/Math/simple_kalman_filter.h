
#ifndef SIMPLE_KALMAN_FILTER
#define SIMPLE_KALMAN_FILTER

#include <Eigen/Dense>
#include "my_utils/Math/pseudo_inverse.hpp"

class SimpleSystemParam{
    // x(k+1) = F*x(k) + w(k), w(k)~N(0,Q)
    // z(k) = H*x(k) + v(k), v(k)~N(0,R)
    public:
        Eigen::MatrixXd F;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd H;
        Eigen::MatrixXd R;
        SystemParam(){}
        ~SystemParam(){}
};

class SimpleKalmanFilter{
public:
	SimpleKalmanFilter() { b_initialize= false;}
	~SimpleKalmanFilter() {}
	
	void initialize(const Eigen::VectorXd& x0,
                    const Eigen::MatrixXd& P0) {
        x_hat_ = x0;
        P_hat_ = P0; 
        n_ = x0.size();
        b_initialize = true;  
    }

    void propagate(const Eigen::VectorXd& z, Eigen::VectorXd& xhat, void* system_data = NULL){
        if(system_data) sys_param_ = static_cast<SimpleSystemParam*>(system_data);

        predict();
        update(z);
        xhat = x_hat_;
    }

    void predict() {
        x_pre_ =  sys_param_->F * x_hat_;
        P_pre_ =  sys_param_->F * P_hat_ * sys_param_->F.transpose() + sys_param_->Q;
    }

    void update(const Eigen::VectorXd & z) {
        S_ = sys_param_->H * P_pre_ * sys_param_->H.transpose() + sys_param_->R;
        my_utils::pseudoInverse(S_, 0.001, S_inv_);
        K_ = P_pre_ * sys_param_->H.transpose()*S_inv_;
        y_ = z - sys_param_->H * x_pre_;
        
        P_hat_ = (Eigen::MatrixXd::Identity(n_) - K_*sys_param_->H)*P_pre_;
        x_hat_ = x_pre_ + K_*y_;
    }


public:
	bool b_initialize;

private:
    SimpleSystemParam* sys_param_;
    int n_; // x dim

    Eigen::VectordXd x_hat_;
    Eigen::VectordXd x_pre_;
    Eigen::VectordXd y_; // observation error
    

    Eigen::MatrixXd P_hat_;
    Eigen::MatrixXd P_pre_;
    Eigen::MatrixXd S_; // innovation
    Eigen::MatrixXd S_inv_;
    Eigen::MatrixXd K_; // kalman gain
    
};

#endif