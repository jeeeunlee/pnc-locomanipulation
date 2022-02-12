#pragma once

#include <stdio.h>
#include <Eigen/Dense>
#include <iostream>

#include <my_robot_system/RobotSystem.hpp>
#include <my_utils/IO/IOUtilities.hpp>

class MagnetSpec {
  public:
    MagnetSpec(RobotSystem* _robot, int _link_idx, double _fm, double _res_ratio){
        robot_ = _robot;        
        link_idx_ = _link_idx;  
        fm_=_fm;
        residual_ratio_ = _res_ratio;   

        contact_distance_ = 0.0; // assume in contact  
        onoff_ = true;   
    }
    ~MagnetSpec() {}

    int getLinkIdx() {return link_idx_;}

    void setContactDistance(double cd) { contact_distance_ = cd; }
    void setResidualRatio(double rr) { residual_ratio_ = rr; }
    void setMagneticForce(double fm) { fm_ = fm; }
    void setMagnetOnoff(bool _onoff) { onoff_ = _onoff; }

    bool getOnOff() {return onoff_;}    
    double getMagneticForce();    
    Eigen::MatrixXd getJacobian();

  private:
    double computeFm(double f0, double d=contact_distance_, double d0=0.02);

  protected:
    RobotSystem* robot_;
    int link_idx_;
    double fm_;
    double residual_ratio_;
    double contact_distance_;
    bool onoff_;
};