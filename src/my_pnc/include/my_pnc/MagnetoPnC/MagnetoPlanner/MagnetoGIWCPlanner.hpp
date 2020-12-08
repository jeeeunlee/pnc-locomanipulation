#pragma once

#include <../my_utils/Configuration.h>
#include <Eigen/Dense>



class RobotSystem;
class ContactSpec;


class MagnetoGIWCPlanner {
   public:
        MagnetoGIWCPlanner(RobotSystem* robot);
        ~MagnetoGIWCPlanner();        

        void setFmOverM(double fm_over_m) { fm_over_m_ = fm_over_m; };
        void setContact(const std::vector<ContactSpec*>& contact_list, 
                        const std::vector<ContactSpec*>& real_contact_list, 
                        ContactSpecType contact_type);
        void setContact(const std::vector<ContactSpec*> & contact_list, ContactSpecType contact_type);
        bool findOptimalCOM(const Eigen::Vector3d& com_pos_ini, Eigen::Vector3d& com_pos_des);
        void getFeasibleCOM(std::vector<std::pair<double, Eigen::Vector3d>>& _feasible_com_list );
        
     //    (const Eigen::Vector3d& com_pos_ini_, 
     //                           std::vector<std::pair<double, Eigen::Vector3d>>& com_pos_list);

   protected:
        void _updateConvexHull();   
        
        void _buildAstanceMatrix(const std::vector<ContactSpec*> & contact_list);
        void _buildAstanceMatrixPoint(const std::vector<ContactSpec*> & contact_list);
        void _buildAstanceMatrixSurface(const std::vector<ContactSpec*> & contact_list);
        void _buildContactPlane(const std::vector<ContactSpec*> & contact_list);
        void _buildContactJacobianNullspace(const std::vector<ContactSpec*> & contact_list);
        void _buildHyperPlane(const::Eigen::MatrixXd& gravity, Eigen::MatrixXd& abc, Eigen::VectorXd& d);        
        void _buildHyperPlane(Eigen::MatrixXd& abc, Eigen::VectorXd& d);     
        void _buildCoMJacobian();

        void _buildProjectionMatrix(const Eigen::MatrixXd& J, Eigen::MatrixXd& N);
        void _buildJacobianFromContacts(const std::vector<ContactSpec*> & contact_list, Eigen::MatrixXd& Jc);
        
        void _ineqMat2Uf(Eigen::MatrixXd &Uf);  

        double _getMinDistance(const Eigen::VectorXd& pos, const Eigen::MatrixXd& abc, const Eigen::VectorXd& d);
        void _getRandomSample(const Eigen::Vector3d& com_pos_ini, Eigen::Vector3d& com_pos_sample);
        void _enforceJointLimit(const Eigen::VectorXd& q_curr, Eigen::VectorXd& dq);

        Eigen::MatrixXd _null(const Eigen::MatrixXd& input);
        Eigen::MatrixXd _pinv(const Eigen::MatrixXd& input);

        RobotSystem* robot_;
        RobotSystem* robot_temp_;

        Eigen::MatrixXd Uf_;
        Eigen::MatrixXd Ust_;
        Eigen::MatrixXd Astance_;
        Eigen::VectorXd Fm_over_m_;
        Eigen::MatrixXd Jcom_;
        Eigen::MatrixXd Nc_; // contact Jacobian Null Space   
      
        std::vector<std::pair<double, Eigen::Vector3d>> Feasible_com_list_;
        
        Eigen::MatrixXd abc_contact_;
        Eigen::VectorXd d_contact_;

        double fm_over_m_;
        int n_contact_; // # of contacts 
        int n_dof_ ;
        std::vector<int> adof_idx_;    
        std::vector<int> link_idx_;    
        ContactSpecType contact_type_;
//    private:
};