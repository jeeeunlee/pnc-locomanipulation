#pragma once

#include "ContactSpec.hpp"

class RobotSystem;

class BodyFramePointContactSpec : public ContactSpec {
   public:
    BodyFramePointContactSpec(RobotSystem* robot, int _link_idx, double _mu);
    virtual ~BodyFramePointContactSpec();

   protected:
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

   protected:
    double x_;
    double y_;

    virtual bool _UpdateJc();
    virtual bool _UpdateJcDotQdot();
    virtual bool _UpdateJcQdot();
    virtual bool _UpdateUf();
    virtual bool _UpdateInequalityVector();
    void _setU(double x, double y, double mu, Eigen::MatrixXd& U);
};