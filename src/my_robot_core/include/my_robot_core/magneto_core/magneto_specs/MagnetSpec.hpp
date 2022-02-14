#pragma once

#include <stdio.h>
#include <Eigen/Dense>
#include <iostream>

#include <my_wbc/Contact/ContactSpec.hpp>
// #include <my_robot_system/RobotSystem.hpp>
#include <my_utils/IO/IOUtilities.hpp>

class MagnetSpec {
  public:
    MagnetSpec(ContactSpec* _contact, 
              double _fm, double _res_ratio){
      contact_ = _contact;
      fm_=_fm;
      residual_ratio_ = _res_ratio;   

      contact_distance_ = 0.0; // assume in contact  
      onoff_ = true;   
    }
    ~MagnetSpec() {}

    int getLinkIdx() {return contact_->getLinkIdx();}
    void setContactDistance(double cd) { contact_distance_ = cd; }
    void setResidualRatio(double rr) { residual_ratio_ = rr; }
    void setMagneticForce(double fm) { fm_ = fm; }
    void setMagnetOnoff(bool _onoff) { onoff_ = _onoff; }
    bool getOnOff() {return onoff_;}    

    Eigen::VectorXd getMagneticForce();    
    Eigen::MatrixXd getJacobian();

  protected:
    ContactSpec* contact_;
    double fm_;
    double residual_ratio_;
    double contact_distance_;
    bool onoff_;

  private:
    double computeFm(double f0, double d, double d0);
    double computeFm(double f0);

    Eigen::MatrixXd J_;
    
};