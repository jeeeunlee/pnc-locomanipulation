#ifndef WHOLE_BODY_CONTROLLER
#define WHOLE_BODY_CONTROLLER

#include <my_utils/IO/IOUtilities.hpp>
#include <my_utils/Math/MathUtilities.hpp>
#include <my_utils/Math/pseudo_inverse.hpp>
#include <my_wbc/Task/Task.hpp>
#include <my_wbc/Contact/ContactSpec.hpp>

// Assume first 6 (or 3 in 2D case) joints are for the representation of 
// a floating base.
class WBC{
    public:
        WBC(const std::vector<bool> & act_list):
            num_act_joint_(0),
            num_passive_(0)
    {
        act_list_.clear();
        num_qdot_ = act_list.size();

        for (int i(0); i < num_qdot_; ++i) {
            if (act_list[i]) act_list_.push_back(i);
        }        
        for(int i(0); i<num_qdot_; ++i){
            if(act_list[i] == true) ++num_act_joint_;
            else ++num_passive_;
        }
        
        Sa_ = Eigen::MatrixXd::Zero(num_act_joint_, num_qdot_);
        Sv_ = Eigen::MatrixXd::Zero(num_passive_, num_qdot_);
        Sf_ = Eigen::MatrixXd::Zero(6, num_qdot_);

        // Set virtual & actuated selection matrix
        int j(0);
        int k(0);
        for(int i(0); i <num_qdot_; ++i){
            if(act_list[i] == true){
                Sa_(j, i) = 1.;
                ++j;
            }
            else{
                Sv_(k, i) = 1.;
                ++k;
            }
        }        
        Sf_.block(0, 0, 6, 6).setIdentity();
    }
        virtual ~WBC(){}

        virtual void updateSetting(const Eigen::MatrixXd & A,
                const Eigen::MatrixXd & Ainv,
                const Eigen::VectorXd & cori,
                const Eigen::VectorXd & grav,
                void* extra_setting = NULL) = 0;

        virtual void makeTorque(const std::vector<Task*> & task_list,
                const std::vector<ContactSpec*> & contact_list,
                Eigen::VectorXd & cmd,
                void* extra_input = NULL) =0;

    protected:
        // full rank fat matrix only
        void _WeightedInverse(const Eigen::MatrixXd & J,
                const Eigen::MatrixXd & Winv,
                Eigen::MatrixXd & Jinv){
            Eigen::MatrixXd lambda(J* Winv * J.transpose());
            Eigen::MatrixXd lambda_inv;
            my_utils::pseudoInverse(lambda, 0.0001, lambda_inv);
            Jinv = Winv * J.transpose() * lambda_inv;
        }

        std::vector<int> act_list_;

        int num_qdot_;
        int num_act_joint_;
        int num_passive_;

        Eigen::MatrixXd Sa_; // Actuated joint
        Eigen::MatrixXd Sv_; // Virtual joint
        Eigen::MatrixXd Sf_; //floating base

        Eigen::MatrixXd A_;
        Eigen::MatrixXd Ainv_;
        Eigen::VectorXd cori_;
        Eigen::VectorXd grav_;

        bool b_updatesetting_;

};

#endif
