#pragma once

#include "ContactSpec.hpp"

class RobotSystem;

class PointContactSpec : public ContactSpec {
   public:
    PointContactSpec(RobotSystem* robot, int _link_idx, double _mu);
    virtual ~PointContactSpec();   

   protected:
    virtual bool _UpdateJc();
    virtual bool _UpdateJcDotQdot();
    virtual bool _UpdateJcQdot();
    virtual bool _UpdateUf();
    virtual bool _UpdateInequalityVector();
};

class SurfaceContactSpec : public ContactSpec {
   public:
    SurfaceContactSpec(RobotSystem* robot, int _link_idx, double _x, double _y,
                       double _mu);
    virtual ~SurfaceContactSpec();
    
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

class FixedBodyContactSpec : public ContactSpec {
   public:
    FixedBodyContactSpec(RobotSystem* _robot);
    virtual ~FixedBodyContactSpec();

   protected:
    virtual bool _UpdateJc();
    virtual bool _UpdateJcDotQdot();
    virtual bool _UpdateJcQdot();
    virtual bool _UpdateUf();
    virtual bool _UpdateInequalityVector();
};
