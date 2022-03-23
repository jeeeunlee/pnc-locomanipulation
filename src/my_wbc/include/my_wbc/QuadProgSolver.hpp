#pragma once
// Quadratic Programming Solver Utilities

#include <my_utils/IO/IOUtilities.hpp>
#include "Goldfarb/QuadProg++.hh"


class QuadProgSolver{
    public:
    QuadProgSolver();
    ~QuadProgSolver() {};

    void setProblem(const Eigen::MatrixXd& _H,
                    const Eigen::VectorXd& _f,
                    const Eigen::MatrixXd& _Aieq,
                    const Eigen::VectorXd& _bieq,
                    const Eigen::MatrixXd& _Aeq,
                    const Eigen::VectorXd& _beq);
    void setProblem(const Eigen::MatrixXd& _H,
                    const Eigen::VectorXd& _f,
                    const Eigen::MatrixXd& _Aieq,
                    const Eigen::VectorXd& _bieq);
    void solveProblem(Eigen::VectorXd& _x);


protected:
    bool b_initialized;
    int n; // dim_opt_
    int m; // dim_ieq_cstr_
    int p; // dim_eq_cstr_
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