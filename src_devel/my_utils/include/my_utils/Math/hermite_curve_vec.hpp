#ifndef ALM_HERMITE_CURVE_VEC_H
#define ALM_HERMITE_CURVE_VEC_H

#include <vector>
#include <Eigen/Dense>
#include <my_utils/Math/hermite_curve.hpp>

// vector version of hermite curve interpolation

class HermiteCurveVec{
public:
	HermiteCurveVec();
	HermiteCurveVec(const Eigen::VectorXd & start_pos, const Eigen::VectorXd & start_vel, 
				   const Eigen::VectorXd & end_pos, const Eigen::VectorXd & end_vel, const double & duration);
	~HermiteCurveVec();
	
	void initialize(const Eigen::VectorXd & start_pos, const Eigen::VectorXd & start_vel, 
					const Eigen::VectorXd & end_pos, const Eigen::VectorXd & end_vel, const double & duration);
	Eigen::VectorXd evaluate(const double & t_in);
	Eigen::VectorXd evaluateFirstDerivative(const double & t_in);
	Eigen::VectorXd evaluateSecondDerivative(const double & t_in);

private:
	Eigen::VectorXd p1;
	Eigen::VectorXd v1;
	Eigen::VectorXd p2;
	Eigen::VectorXd v2;

	double t_dur;

	std::vector<HermiteCurve> curves;
 	Eigen::VectorXd output;
};

#endif
