#include <Eigen/LU>
#include <Eigen/SVD>

#include <my_wbc/WBMC/MRWBCC.hpp>

// MRWBMC (Minimum Reaction - Whole Body Climbing Control)
// force variable to be minimized: Fr=Fx+Fm (reaction force)

MRWBCC::MRWBCC(const std::vector<bool>& act_list)
    : MCWBC(act_list) {
    my_utils::pretty_constructor(3, "MRWBCC - Min Reaction Whole Body Climbing Control");    
    qddot_ref_ = Eigen::VectorXd::Zero(num_qdot_);
}

void MRWBCC::makeTorqueGivenRef(const Eigen::VectorXd& ref_cmd,
                    const std::vector<ContactSpec*>& contact_list,
                    const std::vector<MagnetSpec*> &magnet_list,
                    Eigen::VectorXd& cmd, void* extra_input) {
    if (!b_updatesetting_) {
        printf("[Warning] MRWBCC setting is not done\n");
    }
    if (extra_input) data_ = static_cast<MCWBC_ExtraData*>(extra_input);

    // assume ref_cmd is given by qddot_ref_
    if(ref_cmd.size() == num_act_joint_){
        for (int i(0); i < num_act_joint_; ++i)
            qddot_ref_[act_list_[i]] = ref_cmd[i];
    } else if(ref_cmd.size() == num_qdot_) {
        for (int i(0); i < num_qdot_; ++i) 
            qddot_ref_[i] = ref_cmd[i];         
    } else {
        printf("[Warning] qddot reference dimension incorrect \n");
    }
    // Magnet Jacobian & force : Jm_, Fm_
    _BuildMagnetMtxVect(magnet_list);

    // Contact Jacobian & friction condition : 
    // dim_rf_ & Uf_ & Fr_ieq  & Jc_, JcDotQdot, JcQdot_
    _BuildContactMtxVect(contact_list);

    // Dimension Setting
    dim_opt_ = num_qdot_ + 2 * dim_rf_;  // (delta_qddot, Fr, xddot_c)
    dim_eq_cstr_ = num_passive_ + dim_rf_;
    dim_ieq_cstr_ = 2 * num_act_joint_ + Uf_.rows();

    _Build_Equality_Constraint(); 
    _Build_Inequality_Constraint(); 
    _OptimizationPreparation(); // A,B,C,D -> G, g0, CE, ce0, CI, ci0

    double f = solve_quadprog(G, g0, CE, ce0, CI, ci0, z);
    // std::cout<<" solve_quadprog done"<< std::endl;

    if(f == std::numeric_limits<double>::infinity())  {
        std::cout << "Infeasible Solution f: " << f << std::endl;
        std::cout << "x: " << z << std::endl;
        // exit(0.0);
    }
    // else if(f > 1.e5){
    //     std::cout << "f: " << f << std::endl;
    //     std::cout << "x: " << z << std::endl;
    //     std::cout << "cmd: "<<cmd<<std::endl; 
    // }

    _GetSolution(cmd);
    // _SaveDebug();  
}

void MRWBCC::_BuildContactMtxVect(const std::vector<ContactSpec*>& contact_list) {
    Jc_ = Eigen::MatrixXd::Zero(0,0);
    JcDotQdot_ = Eigen::VectorXd::Zero(0);  
    JcQdot_ = Eigen::VectorXd::Zero(0);  
    Uf_ = Eigen::MatrixXd::Zero(0,0);
    Fr_ieq_ = Eigen::VectorXd::Zero(0); 
    dim_rf_ = 0;

    Eigen::MatrixXd Jc_i, Uf_i;
    Eigen::VectorXd Fr_ieq_i, JcDotQdot_i, JcQdot_i, Fm_i;

    for (auto &contact: contact_list) {
        contact->getContactJacobian(Jc_i);
        contact->getJcDotQdot(JcDotQdot_i);
        contact->getJcQdot(JcQdot_i);
        contact->getRFConstraintMtx(Uf_i);
        contact->getRFConstraintVec(Fr_ieq_i);

        // (Fr = Fc+Fm, where Uf*Fc >fieq)
        // Uf(Fr-Fm)> fieq : Uf*Fr > fieq + Uf*Fm
        Fm_i = - Magnet_map_[contact->getLinkIdx()]->getMagneticForce();
        Fr_ieq_i += Uf_i* Fm_i;

        Jc_ = my_utils::vStack(Jc_, Jc_i);
        JcDotQdot_ = my_utils::vStack(JcDotQdot_, JcDotQdot_i);
        JcQdot_ = my_utils::vStack(JcQdot_, JcQdot_i);
        Uf_ = my_utils::dStack(Uf_, Uf_i);
        Fr_ieq_ = my_utils::vStack(Fr_ieq_, Fr_ieq_i);
        dim_rf_ += contact->getDim();
                
        // erase contact link
        Magnet_map_.erase(contact->getLinkIdx());
    }

    // residual magnetic force
    JrmFrm_ = Eigen::VectorXd::Zero(num_qdot_);
    for(auto &[link, magnet]: Magnet_map_)
        JrmFrm_ += magnet->getJmFm();

}

void MRWBCC::_BuildMagnetMtxVect(const std::vector<MagnetSpec*> &magnet_list) {
    Magnet_map_.clear();
    for (auto &magnet : magnet_list) {
        Magnet_map_[magnet->getLinkIdx()] = magnet;
    }
}

void MRWBCC::_Build_Inequality_Constraint() {
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
        tau_min_ - Sa_ * (cori_ + grav_ + A_ * qddot_ref_ - JrmFrm_);
    row_idx += num_act_joint_;

    // tau max
    Cieq_.block(row_idx, 0, num_act_joint_, num_qdot_) = -Sa_ * A_;
    Cieq_.block(row_idx, num_qdot_, num_act_joint_, dim_rf_) =
        Sa_ * Jc_.transpose();
    dieq_.segment(row_idx, num_act_joint_) =
        -tau_max_ + Sa_ * (cori_ + grav_ + A_ * qddot_ref_ - JrmFrm_);

    // my_utils::pretty_print(Cieq_, std::cout, "C ieq");
    // my_utils::pretty_print(dieq_, std::cout, "d ieq");
}

void MRWBCC::_Build_Equality_Constraint() {
    Aeq_ = Eigen::MatrixXd::Zero(dim_eq_cstr_, dim_opt_);
    beq_ = Eigen::VectorXd::Zero(dim_eq_cstr_);

    // passive joint
    Aeq_.block(0, 0, num_passive_, num_qdot_) = Sv_ * A_;
    Aeq_.block(0, num_qdot_, num_passive_, dim_rf_) = -Sv_ * Jc_.transpose();
    beq_.head(num_passive_) = -Sv_ * ( A_ * qddot_ref_ + cori_ + grav_  - JrmFrm_);

    // xddot
    Aeq_.block(num_passive_, 0, dim_rf_, num_qdot_) = Jc_;
    Aeq_.bottomRightCorner(dim_rf_, dim_rf_) =
        -Eigen::MatrixXd::Identity(dim_rf_, dim_rf_);
    beq_.tail(dim_rf_) = -Jc_ * qddot_ref_ - JcDotQdot_;

    // my_utils::pretty_print(Aeq_, std::cout, "Aeq");
    // my_utils::pretty_print(beq_, std::cout, "beq");
}


void MRWBCC::_OptimizationPreparation() {

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
            CE[j][i] = Aeq_(i, j);
        }
        ce0[i] = -beq_[i];
    }
    for (int i(0); i < dim_ieq_cstr_; ++i) {
        for (int j(0); j < dim_opt_; ++j) {
            CI[j][i] = Cieq_(i, j);
        }
        ci0[i] = -dieq_[i];
    }
}

void MRWBCC::_GetSolution(Eigen::VectorXd& cmd) {
    delta_qddot_ = Eigen::VectorXd::Zero(num_qdot_);
    Fc_ = Eigen::VectorXd(dim_rf_);
    xc_ddot_ = Eigen::VectorXd::Zero(dim_rf_);

    for (int i(0); i < num_qdot_; ++i) delta_qddot_[i] = z[i];
    for (int i(0); i < dim_rf_; ++i) Fc_[i] = z[i + num_qdot_];    
    for (int i = 0; i < dim_rf_; ++i)
        xc_ddot_[i] = z[i + num_qdot_ + dim_rf_];

    tau_cmd_ = A_ * (qddot_ref_ + delta_qddot_) + cori_ + grav_ 
                - Jc_.transpose() * (Fc_) - JrmFrm_;

    data_->qddot_ = qddot_ref_ + delta_qddot_;
    data_->Fr_ = Fc_;
    cmd = Sa_ * tau_cmd_; 
}


void MRWBCC::_SaveDebug(){
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
    static int numcount = 0;   

    std::ofstream fout;
    fout.open(THIS_COM "experiment_data/DEBUG/MRWBCC/W.txt");
    Eigen::VectorXd Weights(dim_opt_);
    Weights << data_->W_qddot_, data_->W_rf_, data_->W_xddot_;
    fout<<Weights<<std::endl;
    fout.close();
    
    fout.open(THIS_COM "experiment_data/DEBUG/MRWBCC/A.txt");
    fout<<Aeq_<<std::endl;
    fout.close();
    fout.open(THIS_COM "experiment_data/DEBUG/MRWBCC/B.txt");
    fout<<beq_<<std::endl;
    fout.close();

    fout.open(THIS_COM "experiment_data/DEBUG/MRWBCC/C.txt");
    fout<<Cieq_<<std::endl;
    fout.close();
    fout.open(THIS_COM "experiment_data/DEBUG/MRWBCC/D.txt");
    fout<<dieq_<<std::endl;
    fout.close();

    fout.open(THIS_COM "experiment_data/DEBUG/MRWBCC/dqddot.txt");
    fout<<delta_qddot_<<std::endl;
    fout.close();
    fout.open(THIS_COM "experiment_data/DEBUG/MRWBCC/Fc.txt");
    fout<<Fc_<<std::endl;
    fout.close();
    fout.open(THIS_COM "experiment_data/DEBUG/MRWBCC/xddot.txt");
    fout<<xc_ddot_<<std::endl;
    fout.close();
    fout.open(THIS_COM "experiment_data/DEBUG/MRWBCC/tau_cmd.txt");
    fout<<tau_cmd_<<std::endl;
    fout.close();      

    if(numcount++ > 5)
        exit(0);

}