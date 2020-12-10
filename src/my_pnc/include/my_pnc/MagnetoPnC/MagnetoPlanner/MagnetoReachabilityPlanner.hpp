#pragma once

#include <Eigen/Dense>
#include <my_pnc/Constraint/Constraint.hpp>

class RobotSystem;
class ContactSpec;

enum MagnetoJointType { COXA, FEMUR, TIBIA, FOOT1, FOOT2, FOOT3 };


class MagnetoReachabilityPlanner {
   public:
      MagnetoReachabilityPlanner(RobotSystem* robot);
      ~MagnetoReachabilityPlanner();
      
      void _setDesiredFootPosition(MotionCommand _motion_command);
      void computeGoal(MotionCommand &_motion_command);

   protected:
      RobotSystem* robot_;
      RobotSystem* robot_planner_;

      std::vector<Constraint*> constraint_list;

   private:      
      void _InitCostFunction();
      void _InitConstraints(const std::vector<int> _link_idx_list);
      void _DeleteConstraints();

      void _UpdateConfiguration(const Eigen::VectorXd& q);
      void _UpdateConstraints();
      void _BuildConstraints();
      void _BuildNSConstraints();
      void _UpdateDelQ();

      void _computeCostWeight(Eigen::MatrixXd& _Aprime,
                              Eigen::VectorXd& _bprime);

      bool _checkJoint(int joint_idx, 
                     MagnetoJointType joint_type);

      int num_joint_dof_;

      Eigen::MatrixXd A_;
      Eigen::VectorXd b_;

      Eigen::MatrixXd cJacobian_;
      Eigen::MatrixXd cJacobNull_;
      Eigen::MatrixXd cJacobInv_;
      Eigen::VectorXd ceq_;

      Eigen::VectorXd q_;
      Eigen::VectorXd dotq_;
      Eigen::VectorXd delq_;
};