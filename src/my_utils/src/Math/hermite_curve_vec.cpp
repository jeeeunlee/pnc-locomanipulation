#include <my_utils/Math/hermite_curve_vec.hpp>

// Constructor
HermiteCurveVec::HermiteCurveVec(){}
// Destructor
HermiteCurveVec::~HermiteCurveVec(){}

HermiteCurveVec::HermiteCurveVec(const Eigen::VectorXd& start_pos, const Eigen::VectorXd& start_vel, 
			const Eigen::VectorXd& end_pos, const Eigen::VectorXd& end_vel, const double & duration){
	initialize(start_pos, start_vel, end_pos, end_vel, duration);
}

void HermiteCurveVec::initialize(const Eigen::VectorXd & start_pos, const Eigen::VectorXd & start_vel, 
								const Eigen::VectorXd & end_pos, const Eigen::VectorXd & end_vel, const double & duration){
	// Clear and 	create N hermite curves with the specified boundary conditions
	curves.clear();
	p1 = start_pos;	v1 = start_vel;
	p2 = end_pos;	v2 = end_vel;
	t_dur = duration;

	for(int i = 0; i < start_pos.size(); i++){
		curves.push_back(HermiteCurve(start_pos[i], start_vel[i], end_pos[i], end_vel[i], t_dur));
	}
	output = Eigen::VectorXd::Zero(start_pos.size());
}

// Evaluation functions
Eigen::VectorXd HermiteCurveVec::evaluate(const double & t_in){
	for(int i = 0; i < p1.size(); i++){
		output[i] = curves[i].evaluate(t_in);
	}
	return output;
}

Eigen::VectorXd HermiteCurveVec::evaluateFirstDerivative(const double & t_in){
	for(int i = 0; i < p1.size(); i++){
		output[i] = curves[i].evaluateFirstDerivative(t_in);
	}
	return output;
}

Eigen::VectorXd HermiteCurveVec::evaluateSecondDerivative(const double & t_in){
	for(int i = 0; i < p1.size(); i++){
		output[i] = curves[i].evaluateSecondDerivative(t_in);
	}
	return output;
}
