#pragma once

#include <stdio.h>
#include <Eigen/Dense>
#include <iostream>

#include <my_robot_system/RobotSystem.hpp>
#include <my_utils/IO/IOUtilities.hpp>

//! Contact Frame's z-axis should correspond normal vector to the ground

enum ContactSpecType {POINT, SURFACE, FIXED, SURFACE_BODYFRAME, POINT_BODYFRAME};
class ContactSpec {
  public:
    ContactSpec(RobotSystem* _robot, int _dim, int _link_idx=-1, double _mu=0.0) {
        robot_ = _robot;
        dim_contact_ = _dim;
        b_set_contact_ = false;
        idx_Fz_ = dim_contact_ - 1;
        Jc_ = Eigen::MatrixXd::Zero(dim_contact_, robot_->getNumDofs());
        JcDotQdot_ = Eigen::VectorXd::Zero(dim_contact_);
        
        link_idx_ = _link_idx;        
        setFrictionCoeff(_mu);
        max_Fz_ = 500.;
    }

    virtual ~ContactSpec() {}

    void unsetContact() { b_set_contact_ = false; }
    void getContactJacobian(Eigen::MatrixXd& Jc) { Jc = Jc_; }
    void getJcDotQdot(Eigen::VectorXd& JcDotQdot) { JcDotQdot = JcDotQdot_; }
    void getJcQdot(Eigen::VectorXd& JcQdot) { JcQdot = JcQdot_; }

    int getDim() { return dim_contact_; }
    int getLinkIdx() { return link_idx_; } 
    int getFzIndex() { return idx_Fz_; }   

    double getFrictionCoeff() { return mu_*sqrt(2.0); } 
    void setFrictionCoeff(double _mu) { mu_ = _mu/sqrt(2.0); }
    void setMaxFz(double max_fz) { max_Fz_ = max_fz; }    

    bool updateContactSpec() {
        _UpdateJc();
        _UpdateJcDotQdot();
        _UpdateJcQdot();
        _UpdateUf();
        _UpdateInequalityVector();
        b_set_contact_ = true;
        return true;
    }

    virtual int getDimRFConstratint() { return Uf_.rows(); }
    void getRFConstraintMtx(Eigen::MatrixXd& Uf) { Uf = Uf_; }
    void getRFConstraintVec(Eigen::VectorXd& ieq_vec) { ieq_vec = ieq_vec_; }
    

  protected:
    virtual bool _UpdateJc() = 0;
    virtual bool _UpdateJcDotQdot() = 0;
    virtual bool _UpdateJcQdot() = 0;
    virtual bool _UpdateUf() = 0;
    virtual bool _UpdateInequalityVector() = 0;

    RobotSystem* robot_;
    Eigen::MatrixXd Jc_;
    Eigen::VectorXd JcDotQdot_;
    Eigen::VectorXd JcQdot_;
    Eigen::MatrixXd Uf_;
    Eigen::VectorXd ieq_vec_;

    int dim_contact_;
    int idx_Fz_;
    bool b_set_contact_;

    double mu_;
    double max_Fz_;
    int link_idx_;


};
