// Whole Body Quadratic Programming Dynamics

#include <Eigen/LU>
#include <Eigen/SVD>

#include <my_wbc/WBQPD/WBQPD.hpp>

WBQPD::WBQPD(const Eigen::MatrixXd& Sa, 
            const Eigen::MatrixXd& Sv) {
    Sa_ = Sa;
    Sv_ = Sv;

    dim_opt_ = Sa_.rows();
    dim_eq_cstr_ = Sa_.cols(); 

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

// int dim_opt_; // n_adof
// int dim_eq_cstr_; // equality constraints
// int dim_ieq_cstr_; // inequality constraints
// int dim_fric_ieq_cstr_; // friction constraints
// int dim_trqact_ieq_cstr_; // active torque limit constraints

void WBQPD::_updateOptParam() {

    std::cout<<"hi 1 " << std::endl;

    dim_opt_ = Sa_.rows();
    dim_eq_cstr_ = Sa_.cols();    
    dim_ieq_cstr_ = dim_fric_ieq_cstr_;
    if(b_torque_limit_)
        dim_ieq_cstr_ += 2*dim_trqact_ieq_cstr_; // lower & upper limit

    _updateCostParam(); std::cout<<"hi 2 " << std::endl;
    _updateEqualityParam(); std::cout<<"hi 3 " << std::endl;
    _updateInequalityParam(); std::cout<<"hi 4 " << std::endl;

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
    std::cout<<"hi 5 " << std::endl;

    // equality
    CE.resize(dim_opt_, dim_eq_cstr_);
    ce0.resize(dim_eq_cstr_);
    for (int i(0); i < dim_eq_cstr_; ++i) {
        for (int j(0); j < dim_opt_; ++j) {
            CE[j][i] = Ceq_(i,j);
        }
        ce0[i] = 0.;
    }
    std::cout<<"hi 6 " << std::endl;

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

    std::cout<<"hi 7 " << std::endl; 
}


void WBQPD::_updateCostParam() {
    // 0.5 x'*G*x + g0'*x = 0.5*(Ax+a0-dq_des)*Wq*(Ax+a0-ddq_des)
    Gmat_ = Sa_ * param_->B.transpose() * param_->Wf.asDiagonal() * param_->B * Sa_.transpose();
    gvec_ = Sa_ * param_->B.transpose() * param_->Wf.asDiagonal() * param_->b0;    
}

void WBQPD::_updateEqualityParam() {
    // Ceq_*x + deq_ = 0
    Ceq_ = param_->A * Sa_.transpose();
    deq_ = param_->a0 - param_->ddq_des;
}

void WBQPD::_updateInequalityParam() {
    // Cieq*x + dieq >= 0
    Cieq_ = Eigen::MatrixXd::Zero(dim_ieq_cstr_, dim_opt_);
    dieq_ = Eigen::VectorXd::Zero(dim_ieq_cstr_);
    int row_idx(0);
    Cieq_.block(row_idx, 0, dim_fric_ieq_cstr_, dim_opt_) 
                            = U_*param_->B * Sa_.transpose();
    dieq_.head(dim_fric_ieq_cstr_) = U_*param_->b0 - u0_;

    row_idx+=dim_fric_ieq_cstr_;
    if(b_torque_limit_) {        
        Cieq_.block(row_idx, 0, dim_trqact_ieq_cstr_, dim_opt_)
            = Eigen::VectorXd::Identity(dim_opt_, dim_opt_);
        dieq_.segment(row_idx, dim_trqact_ieq_cstr_) = -tau_l_;
        row_idx+=dim_trqact_ieq_cstr_;
        Cieq_.block(row_idx, 0, dim_trqact_ieq_cstr_, dim_opt_) 
            = - Eigen::VectorXd::Identity(dim_opt_, dim_opt_);
        dieq_.segment(row_idx, dim_trqact_ieq_cstr_) = tau_u_;
    }
}

double WBQPD::computeTorque(void* result){
    if (result) 
        result_ = static_cast<WbqpdResult*>(result);

    Eigen::MatrixXd Aprime = param_->A * Sa_.transpose(); // N X Na
    Eigen::MatrixXd Bprime = param_->B * Sa_.transpose();

    // Analyze Aprime
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      Aprime, Eigen::ComputeThinU | Eigen::ComputeFullV);

    int const nrows(svd.singularValues().rows()); //12
    int const Vrows(svd.matrixV().rows()); //12
    int const Vcols(svd.matrixV().cols()); //12
    int const Urows(svd.matrixU().rows()); //30
    int const Ucols(svd.matrixU().cols()); //12

    // std::cout<<"singular values of A prime are :  " << std::endl << svd.singularValues() << std::endl;
    // std::cout<<"nrows = " << nrows << ", Vrows=" << Vrows << ", Vcols=" << Vcols << ", Urows=" << Urows << ", Ucols=" << Ucols << std::endl;
  
    // set threshold = 100 => maxSingVal > threshold * minSingVal 
    // if Aprime has full column rank, tau_a is obtained and we can only determine the feasibility
    // else assume Aprime has 1 dimension of nullity (which is usual)    

    // pseudo Inverse Aprime, null Aprime
    Eigen::MatrixXd AprimePinv;
    Eigen::VectorXd AprimeNull;
    double sv_threshold = 100.;
    if(svd.singularValues().coeff(0) > svd.singularValues().coeff(nrows-1) * sv_threshold ) {
        // formulate 1d constrained optimization problem
        // Aprime*tau_a + a0 = ddq_des => tau_a = AprimePinv*(ddq_des - a0) + AprimeNull*z
        Eigen::MatrixXd MatSigmaInv = Eigen::MatrixXd::Zero(nrows, nrows);
        for(int ii(0); ii<nrows; ++ii) {
            if(svd.singularValues().coeff(ii) > 1.)
                MatSigmaInv(ii, ii) = 1./svd.singularValues().coeff(ii);
            else
                MatSigmaInv(ii, ii) = 0.;            
        }

        AprimePinv = svd.matrixV() * MatSigmaInv * svd.matrixU().transpose();
        AprimeNull = svd.matrixV().col(Vcols-1); // last column vector

        Eigen::VectorXd x1,x2;
        // tau_a = AprimePinv*(ddq_des - a0) + AprimeNull*alpha = alpha * x2 + x1 
        x1 = AprimePinv*(param_->ddq_des - param_->a0);
        x2 = AprimeNull;
        double minEq, maxEq;
        bool iq = _checkInequaility(x1,x2,minEq,maxEq);
        std::cout << minEq << " < alpha < " << maxEq << std::endl;
    }
    else { // check feasibility
        
    }

    result_->b_reachable = true;
    result_->tau = Eigen::VectorXd::Zero(dim_eq_cstr_);
    result_->ddq = Eigen::VectorXd::Zero(dim_eq_cstr_);


    return 0.0;
}

bool WBQPD::_checkInequaility(const Eigen::VectorXd& x1,
                            const Eigen::VectorXd& x2,
                            double& minV,
                            double& maxV){

    // Cieq*tau_a + dieq >= 0
    dim_ieq_cstr_ = dim_fric_ieq_cstr_;
    if(b_torque_limit_)
        dim_ieq_cstr_ += 2*dim_trqact_ieq_cstr_; // lower & upper limit

    _updateInequalityParam(); 

    // tau_a = alpha * x2 + x1 
    // Cieq(alpha*x2 + x1) + dieq = alpha*(Cieq*x2) + Cieq*x1+dieq >0
    //                            = alpha*y2 + y1 > 0
    // return true if alpha=0 satisfy the inequaility
    Eigen::VectorXd y1,y2;
    y2 = Cieq_*x2;
    y1 = Cieq_*x1 + dieq_;
    bool is_ieq_hold = true;

    minV = -100;
    maxV = 100;
    
    // std::cout << "------------------------" << std::endl;
    for(int ii(0); ii<dim_ieq_cstr_; ++ii){
        
        double temp, tol = 1e-5;
        if(y2(ii)<-tol){
            temp = - y1(ii) / y2(ii);
            maxV = maxV < temp ? maxV : temp;           
        } else if (y2(ii)>tol) {
            temp = - y1(ii) / y2(ii);
            minV = minV > temp ? minV : temp; // update with the tighter constraint
        } else {
            /* code */
        }

        // std::cout << " alpha * " << y2(ii) << " + " << y1(ii) << ">0 ,  temp=" << temp <<", min =" << minV << ", max = " << maxV << std::endl;

            
        if(y1(ii) < 0) is_ieq_hold = false;
    }

    if(maxV < minV) is_ieq_hold=false;

    return is_ieq_hold;    

}

// double WBQPD::computeTorque(void* result){
//     if (result) 
//         result_ = static_cast<WbqpdResult*>(result);

//     // update G, g0, CE, ce0, CI, ci0
//     if(b_updatedparam_) _updateOptParam();

//     // solve QP, x=tau    
//     double f = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
//     std::cout<<"hi 8 " << std::endl;

//     if(f == std::numeric_limits<double>::infinity())  {
//         std::cout << "Infeasible Solution f: " << f << std::endl;
//         std::cout << "x: " << x << std::endl;
//         result_->b_reachable = false;
//         // exit(0.0);
//     }
//     else{
//         result_->b_reachable = true;
//         Eigen::VectorXd tau_a = Eigen::VectorXd::Zero(dim_opt_);
//         for (int i(0); i < dim_opt_; ++i) tau_a[i] = x[i];

//         result_->tau = Sa_.transpose() * tau_a;
//         result_->ddq = param_->A * result_->tau + param_->a0;
//     }

//     std::cout<<"hi 9 " << std::endl;
//     return f;
// }

bool WBQPD::computeDdotq(Eigen::VectorXd& tau,
                        Eigen::VectorXd& ddotq) {
    // check the tau is feasible and compute ddotq

    // set passive torque to be zero
    Eigen::VectorXd tau_a = Sa_ * tau;
    tau = Sa_.transpose() * tau_a;

    
    Eigen::VectorXd Uf_Fc = U_*( param_->B * tau + param_->b0);

    // check joint limit
    bool b_joint_limit = true;
    for(int i(0); i<dim_trqact_ieq_cstr_; ++i) {
        if(tau_a[i] < tau_l_[i])
            b_joint_limit = false;
        if(tau_a[i] > tau_u_[i])
            b_joint_limit = false;
    }
    
    // check friction constraint
    bool b_friction_constraint = true;
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