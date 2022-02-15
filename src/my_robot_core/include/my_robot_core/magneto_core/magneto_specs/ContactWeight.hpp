#pragma once

#include <stdio.h>
#include <Eigen/Dense>
#include <iostream>

#include <my_wbc/Contact/ContactSpec.hpp>
// #include <my_robot_system/RobotSystem.hpp>
#include <my_utils/IO/IOUtilities.hpp>

class ContactWeight {
    // weight regarding contact
    // W_xddot_ (contact acceleartion)
    // W_rf_ (contact reaction force)
    // W_rf_ : (rfx, rfy & rfz are dealt differently)
  public:
    ContactWeight(ContactSpec* _contact){
      contact_ = _contact;
      dim_contact_ = contact_->getDim();
      linear_dim_contact_ = 3;
      idx_rf_z_ = contact_->getFzIndex();

      W_xddot_ = Eigen::VectorXd::Zero(dim_contact_);
      W_rf_ = Eigen::VectorXd::Zero(dim_contact_);
    }
    ~ContactWeight() {}

    Eigen::VectorXd getWxddot() { return W_xddot_;}
    Eigen::VectorXd getWrf() { return W_rf_;}

    void reshapeWeightRF(double alpha) {
      if(alpha > 1e-3)
        w_rf_xy_ *= alpha;
        w_rf_z_ /= alpha;      
      updateWrf();      
    }
    void reshapeWeightXddot(double alpha) {
      if(alpha > 1e-3)
        w_xddot_xy_ *= alpha;  
      updateWXddot();      
    }

    void setWeightRF(double val_xy, double val_z) { 
                      w_rf_r_ = val_xy;
                      w_rf_xy_ = val_xy;
                       w_rf_z_ = val_z;
                      updateWrf();  }
    void setWeightXddot(double val, double val_z) {                        
                        w_xddot_r_ = val;
                        w_xddot_xy_ = val; 
                        w_xddot_z_ = val_z;
                        updateWXddot();  }
    
  protected:
    ContactSpec* contact_;
    int dim_contact_;
    int linear_dim_contact_;
    int idx_rf_z_;

    double w_rf_r_;
    double w_rf_xy_;
    double w_rf_z_;
    double w_xddot_r_;
    double w_xddot_xy_;
    double w_xddot_z_;

    Eigen::VectorXd W_xddot_;
    Eigen::VectorXd W_rf_;

    void updateWXddot(){
      W_xddot_ = Eigen::VectorXd::Constant(dim_contact_,w_xddot_r_);
      W_xddot_.tail(linear_dim_contact_) = 
        Eigen::VectorXd::Constant(linear_dim_contact_,w_xddot_xy_);
      W_xddot_[idx_rf_z_] = w_xddot_z_;
    }
    void updateWrf() {
      W_rf_ = Eigen::VectorXd::Constant(dim_contact_,w_rf_r_);
      W_rf_.tail(linear_dim_contact_) = 
        Eigen::VectorXd::Constant(linear_dim_contact_,w_rf_xy_);
      W_rf_[idx_rf_z_] = w_rf_z_;
    }

};