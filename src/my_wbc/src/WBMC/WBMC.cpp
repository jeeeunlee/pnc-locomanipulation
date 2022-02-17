#include <Eigen/LU>
#include <Eigen/SVD>

#include <my_wbc/WBMC/WBMC.hpp>
#include <my_utils/IO/IOUtilities.hpp>

WBMC::WBMC(const std::vector<bool>& act_list)
    : WBC(act_list) {
    my_utils::pretty_constructor(3, "WBMC");
    Sf_ = Eigen::MatrixXd::Zero(6, num_qdot_);
    Sf_.block(0, 0, 6, 6).setIdentity();

    act_list_.clear();
    for (int i(0); i < num_qdot_; ++i) {
        if (act_list[i]) act_list_.push_back(i);
    }
    qddot_ = Eigen::VectorXd::Zero(num_qdot_);
    // dynacore::pretty_print(Sv_, std::cout, "Sv");
}

void WBMC::updateSetting(const Eigen::MatrixXd& A, const Eigen::MatrixXd& Ainv,
                         const Eigen::VectorXd& cori,
                         const Eigen::VectorXd& grav, void* extra_setting) {
    A_ = A;
    Ainv_ = Ainv;
    cori_ = cori;
    grav_ = grav;
    b_updatesetting_ = true;

    // dynacore::pretty_print(grav_, std::cout, "grav");
    // dynacore::pretty_print(cori_, std::cout, "cori");
    // dynacore::pretty_print(A_, std::cout, "A");
}

void WBMC::setTorqueLimits(const Eigen::VectorXd &_tau_min,
                            const Eigen::VectorXd &_tau_max) {
    tau_min_ = _tau_min;
    tau_max_ = _tau_max;
}

void WBMC::makeTorqueGivenRef(const Eigen::VectorXd& des_jacc_cmd,
                           const std::vector<ContactSpec*>& contact_list,
                           Eigen::VectorXd& cmd, void* extra_input) {
    if (!b_updatesetting_) {
        printf("[Warning] WBMC setting is not done\n");
    }
    if (extra_input) data_ = static_cast<WBMC_ExtraData*>(extra_input);

    // Internal Constraint Check
    Nci_ = Eigen::MatrixXd::Identity(num_qdot_, num_qdot_);

    for (int i(0); i < num_act_joint_; ++i) {
        qddot_[act_list_[i]] = des_jacc_cmd[i];
    }

    // Contact Jacobian & Uf & Fr_ieq
    _BuildContactMtxVect(contact_list);

    // Dimension Setting
    dim_opt_ = num_qdot_ + 2 * dim_rf_;  // (delta_qddot, Fr, xddot_c)
    dim_eq_cstr_ = num_passive_ + dim_rf_;
    dim_ieq_cstr_ = 2 * num_act_joint_ + Uf_.rows();

    _Build_Equality_Constraint(); 
    _Build_Inequality_Constraint(); 
    _OptimizationPreparation(Aeq_, beq_, Cieq_, dieq_); 

    double f = solve_quadprog(G, g0, CE, ce0, CI, ci0, z);

    if(f == std::numeric_limits<double>::infinity())  {
        std::cout << "Infeasible Solution f: " << f << std::endl;
        std::cout << "x: " << z << std::endl;
        // exit(0.0);
    }


    _GetSolution(cmd);
    // std::cout << "f: " << f << std::endl;
    // std::cout << "x: " << z << std::endl;
    // std::cout << "cmd: "<<cmd<<std::endl;

    // if(f > 1.e5){
    //   std::cout << "f: " << f << std::endl;
    //   std::cout << "x: " << z << std::endl;
    // std::cout << "cmd: "<<cmd<<std::endl;

    //   printf("G:\n");
    //   std::cout<<G<<std::endl;
    //   printf("g0:\n");
    //   std::cout<<g0<<std::endl;

    //   printf("CE:\n");
    //   std::cout<<CE<<std::endl;
    //   printf("ce0:\n");
    //   std::cout<<ce0<<std::endl;

    //   printf("CI:\n");
    //   std::cout<<CI<<std::endl;
    //   printf("ci0:\n");
    //   std::cout<<ci0<<std::endl;
    // }
}

void WBMC::_Build_Inequality_Constraint() {
    Cieq_ = Eigen::MatrixXd::Zero(dim_ieq_cstr_, dim_opt_);
    dieq_ = Eigen::VectorXd::Zero(dim_ieq_cstr_);
    int row_idx(0);

    // friction cone
    Cieq_.block(row_idx, num_qdot_, Uf_.rows(), dim_rf_) = Uf_;
    dieq_.head(Uf_.rows()) = Fr_ieq_;
    row_idx += Uf_.rows();

    // tau min
    Cieq_.block(row_idx, 0, num_act_joint_, num_qdot_) = Sa_ * A_;
    Cieq_.block(row_idx, num_qdot_, num_act_joint_, dim_rf_) =
        -Sa_ * Jc_.transpose();
    dieq_.segment(row_idx, num_act_joint_) =
        tau_min_ - Sa_ * (cori_ + grav_ + A_ * qddot_);
    row_idx += num_act_joint_;

    // tau max
    Cieq_.block(row_idx, 0, num_act_joint_, num_qdot_) = -Sa_ * A_;
    Cieq_.block(row_idx, num_qdot_, num_act_joint_, dim_rf_) =
        Sa_ * Jc_.transpose();
    dieq_.segment(row_idx, num_act_joint_) =
        -tau_max_ + Sa_ * (cori_ + grav_ + A_ * qddot_);

    // my_utils::pretty_print(Cieq_, std::cout, "C ieq");
    // my_utils::pretty_print(dieq_, std::cout, "d ieq");
}

void WBMC::_Build_Equality_Constraint() {
    Aeq_ = Eigen::MatrixXd::Zero(dim_eq_cstr_, dim_opt_);
    beq_ = Eigen::VectorXd::Zero(dim_eq_cstr_);

    // passive joint
    Aeq_.block(0, 0, num_passive_, num_qdot_) = Sv_ * A_;
    Aeq_.block(0, num_qdot_, num_passive_, dim_rf_) = -Sv_ * Jc_.transpose();
    beq_.head(num_passive_) = -Sv_ * ( A_ * qddot_ + cori_ + grav_ );

    // xddot
    Aeq_.block(num_passive_, 0, dim_rf_, num_qdot_) = Jc_;
    Aeq_.bottomRightCorner(dim_rf_, dim_rf_) =
        -Eigen::MatrixXd::Identity(dim_rf_, dim_rf_);
    beq_.tail(dim_rf_) = -Jc_ * qddot_ - JcDotQdot_;

    // my_utils::pretty_print(Aeq_, std::cout, "Aeq");
    // my_utils::pretty_print(beq_, std::cout, "beq");
}

void WBMC::_BuildContactMtxVect(const std::vector<ContactSpec*>& contact_list) {
    ContactSpec* contact = contact_list[0];
    contact->getContactJacobian(Jc_);
    contact->getJcDotQdot(JcDotQdot_);
    contact->getJcQdot(JcQdot_);    
    contact->getRFConstraintMtx(Uf_);
    contact->getRFConstraintVec(Fr_ieq_);


    Eigen::VectorXd dFr_by_magnet ;
    dFr_by_magnet = - Uf_*(data_->F_magnetic_.head(Uf_.cols()));    
    Fr_ieq_ += dFr_by_magnet;

    Eigen::MatrixXd Jc_i, Uf_i;
    Eigen::VectorXd Fr_ieq_i, JcDotQdot_i, JcQdot_i;

    dim_rf_ = Jc_.rows();
    int num_rows_Uf = Uf_.rows();
    int num_cols_Uf = Uf_.cols();
    for (int i(1); i < contact_list.size(); ++i) {
        contact = contact_list[i];
        contact->getContactJacobian(Jc_i);
        contact->getJcDotQdot(JcDotQdot_i);
        contact->getJcQdot(JcQdot_i);
        contact->getRFConstraintMtx(Uf_i);
        contact->getRFConstraintVec(Fr_ieq_i);

        dFr_by_magnet = - Uf_i*(data_->F_magnetic_.segment(num_cols_Uf, Uf_i.cols()));  
        Fr_ieq_i += dFr_by_magnet;

        Jc_.conservativeResize(dim_rf_ + Jc_i.rows(), num_qdot_);
        Jc_.block(dim_rf_, 0, Jc_i.rows(), num_qdot_) = Jc_i;

        
        JcQdot_.conservativeResize(dim_rf_ + Jc_i.rows());
        JcQdot_.tail(Jc_i.rows()) = JcQdot_i;

        JcDotQdot_.conservativeResize(dim_rf_ + Jc_i.rows());
        JcDotQdot_.tail(Jc_i.rows()) = JcDotQdot_i;

        Uf_.conservativeResize(num_rows_Uf + Uf_i.rows(),
                               dim_rf_ + Uf_i.cols());
        (Uf_.topRightCorner(num_rows_Uf, Uf_i.cols())).setZero();
        (Uf_.bottomLeftCorner(Uf_i.rows(), dim_rf_)).setZero();
        Uf_.block(num_rows_Uf, dim_rf_, Uf_i.rows(), Uf_i.cols()) = Uf_i;

        Fr_ieq_.conservativeResize(num_rows_Uf + Uf_i.rows());
        Fr_ieq_.tail(Uf_i.rows()) = Fr_ieq_i;

        dim_rf_ += Jc_i.rows();
        num_rows_Uf += Uf_i.rows();
        num_cols_Uf += Uf_i.cols();
    }
    // my_utils::pretty_print(Jc_, std::cout, "Jc");
    // my_utils::pretty_print(Uf_, std::cout, "Uf");
    // my_utils::pretty_print(JcDotQdot_, std::cout, "JcDotQdot");
    // my_utils::pretty_print(Fr_ieq_, std::cout, "Fr_ieq");
}

void WBMC::_OptimizationPreparation(const Eigen::MatrixXd& Aeq,
                                    const Eigen::VectorXd& beq,
                                    const Eigen::MatrixXd& Cieq,
                                    const Eigen::VectorXd& dieq) {

    G.resize(dim_opt_, dim_opt_);
    g0.resize(dim_opt_);
    CE.resize(dim_opt_, dim_eq_cstr_);
    ce0.resize(dim_eq_cstr_);
    CI.resize(dim_opt_, dim_ieq_cstr_);
    ci0.resize(dim_ieq_cstr_);

    for (int i(0); i < dim_opt_; ++i) {
        for (int j(0); j < dim_opt_; ++j) {
            G[i][j] = 0.;
        }
        g0[i] = 0.;
    }
    // Set Cost
    for (int i(0); i < num_qdot_; ++i) {
        G[i][i] = data_->W_qddot_[i];
    }
    int idx_offset = num_qdot_;
    for (int i(0); i < dim_rf_; ++i) {
        G[i + idx_offset][i + idx_offset] = data_->W_rf_[i];
    }
    idx_offset += dim_rf_;
    for (int i(0); i < dim_rf_; ++i) {
        G[i + idx_offset][i + idx_offset] = data_->W_xddot_[i];
    }
    for (int i(0); i < dim_eq_cstr_; ++i) {
        for (int j(0); j < dim_opt_; ++j) {
            CE[j][i] = Aeq(i, j);
        }
        ce0[i] = -beq[i];
    }
    for (int i(0); i < dim_ieq_cstr_; ++i) {
        for (int j(0); j < dim_opt_; ++j) {
            CI[j][i] = Cieq(i, j);
        }
        ci0[i] = -dieq[i];
    }

    // printf("G:\n");
    // std::cout << G << std::endl;
    // printf("g0:\n");
    // std::cout << g0 << std::endl;
    // printf("CE:\n");
    // std::cout << CE << std::endl;
    // printf("ce0:\n");
    // std::cout << ce0 << std::endl;
    // printf("CI:\n");
    // std::cout << CI << std::endl;
    // printf("ci0:\n");
    // std::cout << ci0 << std::endl;

    // std::ofstream fout;
    // fout.open(THIS_COM "A.txt");
    // fout<<CE<<std::endl;
    // fout.close();
    // fout.open(THIS_COM "B.txt");
    // fout<<ce0<<std::endl;
    // fout.close();
    // fout.open(THIS_COM "C.txt");
    // fout<<CI<<std::endl;
    // fout.close();
    // fout.open(THIS_COM "D.txt");
    // fout<<ci0<<std::endl;
    // fout.close();
}

void WBMC::_GetSolution(Eigen::VectorXd& cmd) {
    Eigen::VectorXd delta_qddot(num_qdot_);
    for (int i(0); i < num_qdot_; ++i) delta_qddot[i] = z[i];
    data_->Fr_ = Eigen::VectorXd(dim_rf_);
    for (int i(0); i < dim_rf_; ++i) data_->Fr_[i] = z[i + num_qdot_];
    Eigen::VectorXd delta_xddot(dim_rf_);
    for (int i = 0; i < dim_rf_; ++i)
        delta_xddot[i] = z[i + num_qdot_ + dim_rf_];

    Eigen::VectorXd tau = A_ * (qddot_ + delta_qddot) + cori_ + grav_ -
                          Jc_.transpose() * (data_->Fr_);

    data_->qddot_ = qddot_ + delta_qddot;
    cmd = Sa_ * tau;

    
    //0112 my_utils::saveVector(data_->Fr_, "Fr_WBMC");
    
    // my_utils::pretty_print(qddot_, std::cout, "qddot_");
    // my_utils::pretty_print(delta_qddot, std::cout, "delta_qddot");
    // my_utils::pretty_print(data_->Fr_, std::cout, "Fr");
    // my_utils::pretty_print(tau, std::cout, "total tau");
    // Eigen::VectorXd x_check = Jc_ * (qddot_ + delta_qddot) + JcDotQdot_;
    // my_utils::pretty_print(x_check, std::cout, "x check");
    // my_utils::pretty_print(delta_xddot, std::cout, "delta xddot");

    // Eigen::VectorXd xdot_check = JcQdot_ + (Jc_ * (qddot_ + delta_qddot) + JcDotQdot_)* 0.001;
    // my_utils::pretty_print(xdot_check, std::cout, "xdot check");
}

