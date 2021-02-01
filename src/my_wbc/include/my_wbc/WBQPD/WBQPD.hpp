#pragma once
// Whole Body Quadratic Programming Dynamics

#include <my_utils/IO/IOUtilities.hpp>
#include <my_utils/Math/pseudo_inverse.hpp>
#include "Goldfarb/QuadProg++.hh"

/*
let contact dynamics at the moment be described as:
    ddq = A*tau + a0
    Fc = B*tau + b0
s.t.
    Sp*tau = 0
    tau_l < Sa*tau < tau_u
    U*(Fc) < ui0

Let the cost function for the problem be formulated as:
min 0.5*(ddq-ddq_des)'Wq(ddq-ddq_des) + 0.5*(Fc)'Wf(Fc)

Then problem is in the form:
tau = argmin(tau) 0.5*(A*tau+a0-ddq_des)'Wq(~) + 0.5*(B*tau + b0)'Wf(~)
s.t.
    tau_l < tau < tau_u
    U*(B*tau + b0) < ui0
*/

struct WbqpdParam{
    // Dynamics Parameter at the moment
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::VectorXd a0;
    Eigen::VectorXd b0;
    Eigen::VectorXd ddq_des;

    // Weight Parameter 
    Eigen::VectorXd Wq;
    Eigen::VectorXd Wf;
};

struct WbqpdResult{
    bool b_reachable;
    Eigen::VectorXd tau;
    // Eigen::VectorXd Fc;
    // Eigen::VectorXd ddq;
};

class WBQPD{
    public:
        WBQPD(const Eigen::MatrixXd& Sa, 
            const Eigen::MatrixXd& Sv);
        ~WBQPD();
        void updateSetting(void* param=NULL);
        double computeTorque(void* result);
        bool computeDdotq(Eigen::VectorXd& tau,
                            Eigen::VectorXd& ddotq);

        void setTorqueLimit(const Eigen::VectorXd& tau_l,
                            const Eigen::VectorXd& tau_u);
        void setFrictionCone(const Eigen::MatrixXd& U,
                            const Eigen::VectorXd& u0);

    private:
        void _updateOptParam();    
        void _updateCostParam();
        void _updateEqualityParam();
        void _updateInequalityParam();

    protected:
        bool b_updatedparam_;
        bool b_torque_limit_;

        Eigen::MatrixXd Sa_;
        Eigen::MatrixXd Sv_;

        Eigen::VectorXd tau_l_;
        Eigen::VectorXd tau_u_;

        Eigen::MatrixXd U_;
        Eigen::VectorXd u0_;

        int dim_opt_; // n_dof
        int dim_eq_cstr_; // equality constraints
        int dim_ieq_cstr_; // inequality constraints
        int dim_fric_ieq_cstr_; // friction constraints
        int dim_trqact_ieq_cstr_; // active torque limit constraints

        Eigen::MatrixXd Gmat_;
        Eigen::VectorXd gvec_;
        Eigen::MatrixXd Ceq_;
        Eigen::VectorXd deq_;
        Eigen::MatrixXd Cieq_;
        Eigen::VectorXd dieq_;

        WbqpdParam* param_;
        WbqpdResult* result_;

        // --------------------------
        //  Optimization parameters
        // --------------------------
        GolDIdnani::GVect<double> x;
        // Cost
        GolDIdnani::GMatr<double> G;
        GolDIdnani::GVect<double> g0;

        // Equality
        GolDIdnani::GMatr<double> CE;
        GolDIdnani::GVect<double> ce0;

        // Inequality
        GolDIdnani::GMatr<double> CI;
        GolDIdnani::GVect<double> ci0;
};