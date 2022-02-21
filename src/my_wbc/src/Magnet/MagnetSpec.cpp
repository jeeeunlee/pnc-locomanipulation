#include <my_wbc/Magnet/MagnetSpec.hpp>


Eigen::VectorXd MagnetSpec::getMagneticForce() { 
  // differ from onoff
  // contact distance
  Eigen::VectorXd Fm = Eigen::VectorXd::Zero(contact_->getDim());
  if(onoff_){
    Fm[contact_->getFzIndex()] = computeFm( fm_ );
  }else{
    Fm[contact_->getFzIndex()] = computeFm( fm_*residual_ratio_ );
  }
  return Fm;
}

Eigen::MatrixXd MagnetSpec::getJacobian() {
  contact_->updateContactSpec();
  contact_->getContactJacobian(J_);
  return J_;
}

Eigen::VectorXd MagnetSpec::getJmFm() {
  // return JmFm
  contact_->updateContactSpec();
  contact_->getContactJacobian(J_);
  JmFm_ = J_.transpose() * (-getMagneticForce());
  return JmFm;
}

double MagnetSpec::computeFm(double f0){
  // contact_distance_ <- setContactDistance
  // 0.02 : contact distance criteria
  return computeFm(f0, contact_distance_, 0.02);
} 

double MagnetSpec::computeFm(double f0, double d, double d0) {
  // f0 : magnetic force when contact(d=0)
  // d : contact distance
  // d0 : contact distance criteria
  double r = d0 / (d+d0);
  return r*r*f0;
}
