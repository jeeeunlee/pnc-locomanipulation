
#ifndef SIMPLE_KALMAN_FILTER
#define SIMPLE_KALMAN_FILTER

#include <Eigen/Dense>

class SimpleSystemParam{
    // x(k+1) = F*x(k) + w(k), w(k)~N(0,Q)
    // z(k) = H*x(k) + v(k), v(k)~N(0,Q)
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
	SimpleKalmanFilter(){
        b_initialized = false;
    }
	~SimpleKalmanFilter() {}
	
	void initialize(int idim, double cutoff) {

    }

    void propagate(){

    }

	Eigen::VectorXd update(const Eigen::VectorXd & z, void* system_data = NULL){
        if(!b_initialized){
            std::cout<<"SimpleKalmanFilter didn't initialized"<< std::endl;
            initialize(s_in.size(), 30.);
        }
        if (system_data) sys_param_ = static_cast<SimpleSystemParam*>(system_data);

        return filtered_val_0;
    }
public:
	

private:
    SimpleSystemParam* sys_param_;
    bool b_initialized;

    Eigen::Vector3d cDen; // denominator coeff
    Eigen::Vector3d cNum; // numerator coeff

    Eigen::VectorXd filtered_val_0;
    Eigen::VectorXd filtered_val_1;
    Eigen::VectorXd filtered_val_2;

    Eigen::VectorXd input_val_0;
    Eigen::VectorXd input_val_1;
    Eigen::VectorXd input_val_2;
};

#endif