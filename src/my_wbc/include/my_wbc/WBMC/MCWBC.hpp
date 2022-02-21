#ifndef MAGNETIC_CONTACT_WHOLE_BODY_CONTROLLER
#define MAGNETIC_CONTACT_WHOLE_BODY_CONTROLLER

#include <my_utils/IO/IOUtilities.hpp>
#include <my_utils/Math/MathUtilities.hpp>
#include <my_utils/Math/pseudo_inverse.hpp>
#include <Goldfarb/QuadProg++.hh>

#include <my_wbc/Task/Task.hpp>
#include <my_wbc/Contact/ContactSpec.hpp>
#include <my_wbc/Magnet/MagnetSpec.hpp>

// MC-WBC (Magnetic Contact - Whole Body Control)

class MCWBC_ExtraData{
    public:
        // Output
        Eigen::VectorXd opt_result_;
        Eigen::VectorXd qddot_;
        Eigen::VectorXd Fr_;
        // Input
        Eigen::VectorXd W_qddot_;
        Eigen::VectorXd W_rf_;
        Eigen::VectorXd W_xddot_;
        MCWBC_ExtraData(){}
        ~MCWBC_ExtraData(){}
};

class MFWBCC{
    public:
        MFWBCC(const std::vector<bool> & act_list) {
            num_qdot_ = act_list.size();
            act_list_.clear(); // act jnt idx container
            for(int i(0); i<num_qdot_; ++i){
                if(act_list[i]) act_list_.push_back(i);
            }
            num_act_joint_ = act_list_.size();
            num_passive_ = num_qdot_ - num_act_joint_;

            // Set virtual & actuated selection matrix
            Sa_ = Eigen::MatrixXd::Zero(num_act_joint_, num_qdot_);
            Sv_ = Eigen::MatrixXd::Zero(num_passive_, num_qdot_);            
            int j(0), k(0);            
            for(int i(0); i <num_qdot_; ++i){
                if(act_list[i]){
                    Sa_(j, i) = 1.;
                    ++j;
                }
                else{
                    Sv_(k, i) = 1.;
                    ++k;
                }
            }
        }

        virtual ~MFWBCC() {}

        void setTorqueLimits(const Eigen::VectorXd &_tau_min,
                const Eigen::VectorXd &_tau_max) {
            tau_min_ = _tau_min;
            tau_max_ = _tau_max;
        }
        virtual void updateSetting(const Eigen::MatrixXd & A,
                                const Eigen::MatrixXd & Ainv,
                                const Eigen::VectorXd & cori,
                                const Eigen::VectorXd & grav,
                                void* extra_setting = NULL) {
            A_ = A;
            Ainv_ = Ainv;
            cori_ = cori;
            grav_ = grav;
            b_updatesetting_ = true;
        }
        virtual void makeTorqueGivenRef(const Eigen::VectorXd & ref_cmd,
                const std::vector<ContactSpec*> & contact_list,
                const std::vector<MagnetSpec*> & magnet_list,
                Eigen::VectorXd & cmd,
                void* extra_input = NULL) = 0;

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

    protected:
        std::vector<int> act_list_; // Actuated joint idx
        int num_qdot_;
        int num_act_joint_;
        int num_passive_;

        Eigen::MatrixXd Sa_; // Actuated joint
        Eigen::MatrixXd Sv_; // Virtual joint
        Eigen::VectorXd tau_min_;
        Eigen::VectorXd tau_max_;

        Eigen::MatrixXd A_;
        Eigen::MatrixXd Ainv_;
        Eigen::VectorXd cori_;
        Eigen::VectorXd grav_;

        bool b_updatesetting_;
        MCWBC_ExtraData* data_;

        // function in makeTorqueGivenRef 
        // Setup the followings: 
        virtual void _BuildContactMtxVect(const std::vector<ContactSpec*> & contact_list) = 0;
        virtual void _BuildMagnetMtxVect(const std::vector<MagnetSpec*> &magnet_list) = 0;
        virtual void _Build_Equality_Constraint() = 0;
        virtual void _Build_Inequality_Constraint() = 0;
        virtual void _GetSolution(Eigen::VectorXd & cmd) = 0;

};

#endif
