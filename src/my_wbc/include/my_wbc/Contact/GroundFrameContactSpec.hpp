#pragma once

#include "ContactSpec.hpp"

class RobotSystem;

class GroundFramePointContactSpec : public ContactSpec {
   public:
    GroundFramePointContactSpec(RobotSystem* robot, int _link_idx, double _mu);
    virtual ~GroundFramePointContactSpec();

    void getRFConstraintMtx(Eigen::MatrixXd& Uf);
    void getRFConstraintVec(Eigen::VectorXd& ieq_vec);


   protected:
    virtual bool _UpdateJc();
    virtual bool _UpdateJcDotQdot();
    virtual bool _UpdateJcQdot();
    virtual bool _UpdateUf();
    virtual bool _UpdateInequalityVector();
    void _setU(double mu, Eigen::MatrixXd& U);
    void _setIeqVec(double max_Fz, Eigen::VectorXd& ieq_vec);
};