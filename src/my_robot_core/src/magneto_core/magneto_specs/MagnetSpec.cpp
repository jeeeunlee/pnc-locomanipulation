#include <my_robot_core/magneto_core/magneto_specs/MagnetSpec.hpp>

double MagnetSpec::getMagneticForce() { 
  // differ from onoff
  // contact distance
  if(onoff_){
    return computeFm( fm_ );
  }else{
    return computeFm( fm_*residual_ratio_ );
  }
}

Eigen::MatrixXd MagnetSpec::getJacobian() {
  return robot_->getBodyNodeBodyJacobian(link_idx_);
}


double MagnetSpec::computeFm(double f0, double d, double d0){
  // f0 : magnetic force when contact(d=0)
  // d : contact distance
  // d0 : contact distance criteria
  double r = d0 / (d+d0);
  return r*r*f0;
}
