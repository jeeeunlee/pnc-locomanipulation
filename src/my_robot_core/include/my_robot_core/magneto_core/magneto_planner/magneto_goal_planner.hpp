#pragma once

#include <Eigen/Dense>
#include <my_robot_core/magneto_core/magneto_planner/constraints/constraint.hpp>


class RobotSystem;

enum MagnetoJointType { COXA, FEMUR, TIBIA, FOOT1, FOOT2, FOOT3 };


class MagnetoGoalPlanner {
   public:
      MagnetoGoalPlanner(RobotSystem* robot);
      ~MagnetoGoalPlanner();      
     
      void computeGoal(MotionCommand &_motion_command);
      void getGoalConfiguration(Eigen::VectorXd& _q_goal);
      void getGoalComPosition(Eigen::Vector3d& _pc_goal);

   protected:
      RobotSystem* robot_;
      RobotSystem* robot_planner_;

      std::vector<Constraint*> constraint_list;

   private:      
      void _InitCostFunction();
      void _InitConstraints();
      void _DeleteConstraints();

      void _setDesiredFootPosition(const POSE_DATA &foot_pose_data,
                                    int moving_foot_idx);
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
      Eigen::VectorXd q_goal_;

      Eigen::Vector3d pc_goal_;
};