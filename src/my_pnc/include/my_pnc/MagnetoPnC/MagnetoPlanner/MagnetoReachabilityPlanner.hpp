#pragma once

#include <Eigen/Dense>
#include <my_pnc/Constraint/Constraint.hpp>
#include <my_wbc/Contact/BodyFrameContactSpec.hpp>

class RobotSystem;
class ContactSpec;   


class MagnetoReachabilityContact {
   public:
      MagnetoReachabilityContact(RobotSystem* robot_planner);
      ~MagnetoReachabilityContact();

      void update(const Eigen::VectorXd& q,
                  const Eigen::VectorXd& dotq);

      void clearContacts();
      void addContacts(ContactSpec* contact);
      void setWbqpd();

   private:
      void _deleteContacts();
      void _updateContacts(const Eigen::VectorXd& q,
                           const Eigen::VectorXd& dotq);
      void _buildContactJacobian(Eigen::MatrixXd& Jc);
      void _buildContactJcDotQdot(Eigen::VectorXd&  Jdotqdot);

      Eigen::MatrixXd getPseudoInverse(const Eigen::MatrixXd& Mat);
      Eigen::MatrixXd getNullSpaceMatrix(const Eigen::MatrixXd& Mat);

   private:       
      RobotSystem* robot_planner_;
      std::vector<ContactSpec*> contact_list_;

      // updated variables
      Eigen::MatrixXd Jc_;
      Eigen::VectorXd Jcdotqdot_;

      Eigen::MatrixXd M_;
      Eigen::MatrixXd MInv_;
      Eigen::VectorXd cori_grav_;

      Eigen::MatrixXd AInv_;
      Eigen::MatrixXd AMat_;
      Eigen::MatrixXd Jc_bar_T_;
      Eigen::MatrixXd Nc_T_;
      Eigen::MatrixXd Q_;      

      // contact
      int dim_joint_;
      int dim_contact_;
      bool is_update_centroid_;      

      // wbqpd
      bool b_wbqpd_set;
      WbqpdParam* wbqpd_param_;
      WbqpdResult* wbqpd_result_;
      WBQPD* wbqpd_;

};


class MagnetoReachabilityNode {
   public:
      MagnetoReachabilityNode(MagnetoReachabilityContact* contact,
                              const Eigen::VectorXd& q,
                              const Eigen::VectorXd& dotq);
      ~MagnetoReachabilityNode();

      bool FindNextNode(const Eigen::VectorXd& ddq_des);

   private:
      Eigen::VectorXd q_;
      Eigen::VectorXd dotq_;
      MagnetoReachabilityContact* contact_state_;
};

class MagnetoReachabilityEdge {
   public:
      MagnetoReachabilityEdge(MagnetoReachabilityNode* src_node,
                              MagnetoReachabilityNode* dst_node,
                              const Eigen::VectorXd& trq_atv);
      ~MagnetoReachabilityEdge();

   private:
      MagnetoReachabilityNode* src_node_; // source
      MagnetoReachabilityNode* dst_node_; // destiny
      Eigen::VectorXd trq_atv_;
};

class MagnetoReachabilityPlanner {
   public:
      MagnetoReachabilityPlanner(RobotSystem* robot, double fric_coeff);
      ~MagnetoReachabilityPlanner();
      
      void initTorqueLimit(const Eigen::VectorXd& tau_min, 
                          const Eigen::VectorXd& tau_max);
      void setMovingFoot(int moving_foot);
      void compute(const Eigen::VectorXd& q_goal);

   private:      
      void _setInitGoal(const Eigen::VectorXd& q_goal,
                        const Eigen::VectorXd& qdot_goal);  

   protected:
      RobotSystem* robot_;
      RobotSystem* robot_planner_;

      ContactSpec* alfoot_contact_;
      ContactSpec* blfoot_contact_;
      ContactSpec* arfoot_contact_;
      ContactSpec* brfoot_contact_;
      std::vector<ContactSpec*> full_contact_list_;
      
      MagnetoReachabilityContact* full_contact_state_;
      MagnetoReachabilityContact* swing_contact_state_;

      // nodes and edges
      // MagnetoReachabilityNode 

   private:
      // constant
      Eigen::MatrixXd Sa_;
      Eigen::MatrixXd Sv_;      
      Eigen::VectorXd q_zero_dof_;
      bool is_update_centroid_;

      Eigen::VectorXd tau_min_;
      Eigen::VectorXd tau_max_;

      // boundary conditions
      Eigen::VectorXd q_init_;
      Eigen::VectorXd dotq_init_;
      Eigen::VectorXd q_goal_;
      Eigen::VectorXd dotq_goal_;

           
      // state variables
      Eigen::VectorXd q_;
      Eigen::VectorXd dotq_;
      Eigen::VectorXd delq_;

      // contact
      double mu_;
      int dim_joint_;
      int dim_contact_;
      int moving_foot_idx_;

      
};