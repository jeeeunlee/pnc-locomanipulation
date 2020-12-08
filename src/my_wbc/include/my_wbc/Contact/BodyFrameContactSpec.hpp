#pragma once

#include "ContactSpec.hpp"

class RobotSystem;

class BodyFramePointContactSpec : public ContactSpec {
   public:
    BodyFramePointContactSpec(RobotSystem* robot, int _link_idx, double _mu);
    virtual ~BodyFramePointContactSpec();

    int getLinkIdx() { return link_idx_; }
    void setMaxFz(double max_fz) { max_Fz_ = max_fz; }
    void setFrictionCoeff(double _mu) { mu_ = _mu/sqrt(2.0); }

   protected:
    double mu_;
    double max_Fz_;
    int link_idx_;

    virtual bool _UpdateJc();
    virtual bool _UpdateJcDotQdot();
    virtual bool _UpdateJcQdot();
    virtual bool _UpdateUf();
    virtual bool _UpdateInequalityVector();
    void _setU(double x, double y, double mu, Eigen::MatrixXd& U);
};

class BodyFrameSurfaceContactSpec : public ContactSpec {
   public:
    BodyFrameSurfaceContactSpec(RobotSystem* robot, int _link_idx, double _x, double _y,
                       double _mu);
    virtual ~BodyFrameSurfaceContactSpec();

    int getLinkIdx() { return link_idx_; }
    void setMaxFz(double max_fz) { max_Fz_ = max_fz; }
    void setFrictionCoeff(double _mu) { mu_ = _mu/sqrt(2.0); }

   protected:
    double mu_;
    double max_Fz_;
    int link_idx_;
    double x_;
    double y_;

    virtual bool _UpdateJc();
    virtual bool _UpdateJcDotQdot();
    virtual bool _UpdateJcQdot();
    virtual bool _UpdateUf();
    virtual bool _UpdateInequalityVector();
    void _setU(double x, double y, double mu, Eigen::MatrixXd& U);
};