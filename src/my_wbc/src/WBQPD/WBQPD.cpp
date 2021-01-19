// Whole Body Quadratic Programming Dynamics

#include <Eigen/LU>
#include <Eigen/SVD>

#include <my_wbc/WBQPD/WBQPD.hpp>

WBQPD::WBQPD(const Eigen::MatrixXd& Sa, 
            const Eigen::MatrixXd& Sv) {
    Sa_ = Sa;
    Sv_ = Sv;
    b_updatedparam_ = false;
    b_torque_limit_ = false;
}

WBQPD::~WBQPD() {
    delete param_;
    delete result_;
}

void WBQPD::updateSetting(void* param){
    b_updatedparam_ = true;
    if (param) 
        param_ = static_cast<WbqpdParam*>(param);
}

void WBQPD::_updateOptParam() {
    dim_opt_ = param_->A.cols();
    dim_eq_cstr_ = Sv_.rows();    
    dim_ieq_cstr_ = dim_fric_ieq_cstr_;
    if(b_torque_limit_)
        dim_ieq_cstr_ += 2*dim_trqact_ieq_cstr_; // lower & upper limit

    _updateCostParam();
    _updateEqualityParam();    
    _updateInequalityParam();

    // cost
    // 0.5 x'*G*x + g0'*x
    G.resize(dim_opt_, dim_opt_);
    g0.resize(dim_opt_);    
    for (int i(0); i < dim_opt_; ++i) {
        for (int j(0); j < dim_opt_; ++j) {
            G[i][j] = Gmat_(i,j);
        }
        g0[i] = gvec_[i];
    }

    // equality
    CE.resize(dim_opt_, dim_eq_cstr_);
    ce0.resize(dim_eq_cstr_);
    for (int i(0); i < dim_eq_cstr_; ++i) {
        for (int j(0); j < dim_opt_; ++j) {
            CE[j][i] = Ceq_(i,j);
        }
        ce0[i] = 0.;
    }

    // inequality
    // CI'*x + ci0 >= 0    
    // Cieq*x + dieq >= 0
    CI.resize(dim_opt_, dim_ieq_cstr_);
    ci0.resize(dim_ieq_cstr_);  
    for (int i(0); i < dim_ieq_cstr_; ++i) {
        for (int j(0); j < dim_opt_; ++j) {
            CI[j][i] = Cieq_(i, j);
        }
        ci0[i] = dieq_[i];
    } 
}


void WBQPD::_updateCostParam() {
    // 0.5 x'*G*x + g0'*x
    Gmat_ = param_->A.transpose() * param_->Wq.asDiagonal() * param_->A
            + param_->B.transpose() * param_->Wf.asDiagonal() * param_->B ;
    gvec_ = param_->A.transpose() * param_->Wq.asDiagonal() * (param_->a0 - param_->ddq_des)
            +  param_->B.transpose() * param_->Wf.asDiagonal() * param_->b0;    
}

void WBQPD::_updateEqualityParam() {
    Ceq_ = Sv_;
    deq_ = Eigen::VectorXd::Zero(dim_eq_cstr_);
}

void WBQPD::_updateInequalityParam() {
    // Cieq*x + dieq >= 0
    Cieq_ = Eigen::MatrixXd::Zero(dim_ieq_cstr_, dim_opt_);
    dieq_ = Eigen::VectorXd::Zero(dim_ieq_cstr_);
    int row_idx(0);
    Cieq_.block(row_idx, 0, dim_fric_ieq_cstr_, dim_opt_) = U_*param_->B;
    dieq_.head(dim_fric_ieq_cstr_) = U_*param_->b0 - u0_;

    row_idx+=dim_fric_ieq_cstr_;
    if(b_torque_limit_) {        
        Cieq_.block(row_idx, 0, dim_trqact_ieq_cstr_, dim_opt_) = Sa_;
        dieq_.segment(row_idx, dim_trqact_ieq_cstr_) = -tau_l_;
        row_idx+=dim_trqact_ieq_cstr_;
        Cieq_.block(row_idx, 0, dim_trqact_ieq_cstr_, dim_opt_) = -Sa_;
        dieq_.segment(row_idx, dim_trqact_ieq_cstr_) = tau_u_;
    }
}

void WBQPD::computeTorque(void* result){
    if (result) 
        result_ = static_cast<WbqpdResult*>(result);

    // update G, g0, CE, ce0, CI, ci0
    if(b_updatedparam_) _updateOptParam();

    // solve QP, x=tau
    double f = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);

    if(f == std::numeric_limits<double>::infinity())  {
        std::cout << "Infeasible Solution f: " << f << std::endl;
        std::cout << "x: " << x << std::endl;
        result_->b_reachable = false;
        // exit(0.0);
    }
    else{
        result_->b_reachable = true;
        result_->tau = Eigen::VectorXd::Zero(dim_opt_);
        for (int i(0); i < dim_opt_; ++i) result_->tau[i] = x[i];
    }
}

bool WBQPD::computeDdotq(Eigen::VectorXd& tau,
                        Eigen::VectorXd& ddotq) {
    // check the tau is feasible and compute ddotq

    // set passive torque to be zero
    Eigen::VectorXd tau_a = Sa_ * tau;
    tau = Sa_.transpose() * tau_a;

    
    Eigen::VectorXd Uf_Fc = U_*( param_->B * tau + param_->b0);

    // check joint limit
    bool b_joint_limit = true;
    for(int i(0); i<dim_trqzct_ieq_cstr_; ++i) {
        if(tau_a[i] < tau_l_[i])
            b_joint_limit = false;
        if(tau_a[i] > tau_u_[i])
            b_joint_limit = false;
    }
    
    // check friction constraint
    bool b_friction_constraint = true;
    Eigen::
    for(int i(0); i<dim_fric_ieq_cstr_; ++i) {
        if(Uf_Fc[i] < u0_[i])
            b_friction_constraint = false;
    }

    if(b_joint_limit && b_friction_constraint){
        ddotq = param_->A * tau + param_->a0;
        return true;
    }
    else{
        ddotq = Eigen::VectorXd::Zero(dim_opt_);
        return false;
    }
    

}

void WBQPD::setTorqueLimit(const Eigen::VectorXd& tau_l,
                            const Eigen::VectorXd& tau_u){
    b_torque_limit_ = true;
    tau_l_ = tau_l;
    tau_u_ = tau_u;    
    dim_trqact_ieq_cstr_ = tau_l_.size();
}

void WBQPD::setFrictionCone(const Eigen::MatrixXd& U,
                            const Eigen::VectorXd& u0){
    U_ = U;
    u0_ = u0;
    dim_fric_ieq_cstr_ = u0_.size();
}