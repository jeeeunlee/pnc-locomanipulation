#ifndef ALM_HERMITE_CURVE_H
#define ALM_HERMITE_CURVE_H

#include <iostream>
#include <math.h>
#include <algorithm>

// returns a hermite interpolation (cubic) of the boundary conditions for a given s \in [0,1].

class HermiteCurve{
public:
	HermiteCurve();
	HermiteCurve(const double & start_pos, const double & start_vel, 
				 const double & end_pos, const double & end_vel, const double & duration);
	~HermiteCurve();
	double evaluate(const double & t_in);
	double evaluateFirstDerivative(const double & t_in);
	double evaluateSecondDerivative(const double & t_in);

private:
	double p1;
	double v1;
	double p2;
	double v2;

	double t_dur;

	double s_;

	// by default clamps within 0 and 1.
	double clamp(const double & t_in, double lo = 0.0, double hi = 1.0);

};

#endif