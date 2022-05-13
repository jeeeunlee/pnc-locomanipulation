#include <my_robot_system/RobotSystem.hpp>
#include <my_wbc/QuadProgSolver.hpp>
#include <my_robot_core/magneto_core/magneto_planner/magneto_com_planner.hpp>

// #include <my_robot_core/magneto_core/magneto_interface.hpp>
#include <my_robot_core/magneto_core/magneto_state_provider.hpp>
#include <my_wbc/Contact/ContactSpec.hpp>
#include <my_wbc/Contact/BasicContactSpec.hpp>
#include <my_wbc/Contact/BodyFrameContactSpec.hpp>
#include <my_wbc/Magnet/MagnetSpec.hpp>

#include <my_utils/IO/IOUtilities.hpp>
#include <my_utils/Math/MathUtilities.hpp>
#include "my_utils/Math/pseudo_inverse.hpp"
#include <time.h>

MagnetoCoMPlanner::MagnetoCoMPlanner(RobotSystem* robot) {
    my_utils::pretty_constructor(2, "Magneto CoM Hermite Spline Parameter Planner");

    // robot system
    robot_ = robot;
    sp_ = MagnetoStateProvider::getStateProvider(robot_);
    qp_solver_= new QuadProgSolver();

    mass_ = robot_->getRobotMass();
    grav_.setZero();
    grav_(2) = -9.81;
    zero_vel_.setZero();
    Ldot_ = zero_vel_;


    initialized = false;
    Tt_given_ = 0.0;
}

void MagnetoCoMPlanner::replanCentroidalMotionPreSwing(
                                        const std::array<ContactSpec*, Magneto::n_leg>& f_contacts,
                                        const std::array<MagnetSpec*, Magneto::n_leg>& f_mag){

    // all the process almost same except these happen during the 

    // set p_init_, p_goal_, swing_foot_dpos_
    // _setConfigurations(pcom_goal,_motion_command);
    // set matrices
    _buildFrictionCone(f_contacts);
    _buildWeightMatrices(f_contacts);
    _buildFm(f_mag);
    _buildPfRf();
    _buildCentroidalSystemMatrices();

    // 
    // _setPeriods( _motion_command.get_motion_periods() );
    std::cout<<"Tf_="<<Tf_<<", Tt1_="<<Tt1_<<", Ts_="<<Ts_<<", Tt2="<<Tt2_<<std::endl;
    std::cout<<"T1_="<<T1_<<", T2_="<<T2_<<", T3_="<<T3_<<std::endl;

    // solve problem
    _solveQuadProg(); // get dir_com_swing_, alpha, beta

    p_swing_init_ = p_goal_ + ( 0.5*beta_*T3_*T3_ + beta_*T2_*T3_ 
                                + 0.5*alpha_*T2_*T2_ )*dir_com_swing_;
    v_swing_init_ = (-beta_*T3_ -alpha_*T2_)*dir_com_swing_;
    acc_swing_ = alpha_*dir_com_swing_;
    
}

void MagnetoCoMPlanner::replanCentroidalMotionSwing(
                                        const std::array<ContactSpec*, Magneto::n_leg>& f_contacts,
                                        const std::array<MagnetSpec*, Magneto::n_leg>& f_mag,
                                        double passed_time){
    if(acc_swing_.norm() > 1e-5){
        std::cout<<"acc_swing_ ="  <<acc_swing_.transpose() <<std::endl;
        return;
    }

    // set p_init_, p_goal_, swing_foot_dpos_
    // _setConfigurations(pcom_goal,_motion_command);
    // set matrices
    _buildFrictionCone(f_contacts);
    _buildWeightMatrices(f_contacts);
    _buildFm(f_mag);
    _buildPfRf();
    _buildCentroidalSystemMatrices();

    // motion period 
    // _setPeriods( _motion_command.get_motion_periods() );
    T2_ = Ts_ - passed_time;
    std::cout<<"Tf_="<<Tf_<<", Tt1_="<<Tt1_<<", Ts_="<<Ts_<<", Tt2="<<Tt2_<<std::endl;
    std::cout<<"T1_="<<T1_<<", T2_="<<T2_<<", T3_="<<T3_<<std::endl;

    _solveQuadProgReplan();

    acc_swing_ = alpha_*dir_com_swing_;

}

void MagnetoCoMPlanner::planCentroidalMotion(const Eigen::Vector3d& pcom_goal,
                                        MotionCommand &_motion_command,
                                        const std::array<ContactSpec*, Magneto::n_leg>& f_contacts,
                                        const std::array<MagnetSpec*, Magneto::n_leg>& f_mag){
    clock_t start = clock();
    
    // set p_com init & goal, p_swing init & goal
    _setConfigurations(pcom_goal,_motion_command);

    // set matrices
    _buildFrictionCone(f_contacts);
    _buildWeightMatrices(f_contacts);
    _buildFm(f_mag);
    _buildPfRf();
    _buildCentroidalSystemMatrices();

    // motion period 
    _setPeriods( _motion_command.get_motion_periods() );
    std::cout<<"Tf_="<<Tf_<<", Tt1_="<<Tt1_<<", Ts_="<<Ts_<<", Tt2="<<Tt2_<<std::endl;
    std::cout<<"T1_="<<T1_<<", T2_="<<T2_<<", T3_="<<T3_<<std::endl;

    // solve problem
    _solveQuadProg(); // get dir_com_swing_, alpha, beta

    p_swing_init_ = p_goal_ + ( 0.5*beta_*T3_*T3_ + beta_*T2_*T3_ 
                                + 0.5*alpha_*T2_*T2_ )*dir_com_swing_;
    v_swing_init_ = (-beta_*T3_ -alpha_*T2_)*dir_com_swing_;
    acc_swing_ = alpha_*dir_com_swing_;
    
    clock_t end = clock();
    double cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;    
    // std::cout<<" computation time =  "<< cpu_time_used<<std::endl;
    // my_utils::saveValue(cpu_time_used, "cpu_time_used" );
    // std::cout<<"########################################################"<<std::endl;
}

ComMotionCommand MagnetoCoMPlanner::getFullSupportCoMCmdReplaned(double passed_time) {
    Eigen::Vector3d pa = sp_->com_pos_des; 
    Eigen::Vector3d va = sp_->com_vel_des;

    return ComMotionCommand( pa, va, p_swing_init_, v_swing_init_, Tf_-passed_time );
}

ComMotionCommand MagnetoCoMPlanner::getSwingCoMCmdReplaned(double passed_time) {
    Eigen::Vector3d pa = sp_->com_pos_des; 
    Eigen::Vector3d va = sp_->com_vel_des;

    return ComMotionCommand( pa, va, acc_swing_, Ts_-passed_time );
}

ComMotionCommand MagnetoCoMPlanner::getFullSupportCoMCmd() {
    Eigen::Vector3d pa = robot_ ->getCoMPosition();  
    Eigen::Vector3d va = zero_vel_;

    return ComMotionCommand( pa, va, p_swing_init_, v_swing_init_, Tf_ );
}

ComMotionCommand MagnetoCoMPlanner::getSwingStartCoMCmd() {
    // constant acc
    Eigen::Vector3d pa = sp_->com_pos_des;
    Eigen::Vector3d va = sp_->com_vel_des;

    return ComMotionCommand( pa, va, acc_swing_, Tt1_ );
}

ComMotionCommand MagnetoCoMPlanner::getSwingCoMCmd() {
    // constant acc
    Eigen::Vector3d pa = sp_->com_pos_des;
    Eigen::Vector3d va = sp_->com_vel_des;

    return ComMotionCommand( pa, va, acc_swing_, Ts_ );
}

ComMotionCommand MagnetoCoMPlanner::getSwingEndCoMCmd() {
    Eigen::Vector3d pa = sp_->com_pos_des;
    Eigen::Vector3d va = sp_->com_vel_des;

    return ComMotionCommand( pa, va, p_goal_, zero_vel_, Tt2_ );

    // Eigen::Vector3d p_mid = 0.5*(pa + p_goal_);    
    // return ComMotionCommand( pa, va, p_mid, zero_vel_, Tt2_ );
}

void MagnetoCoMPlanner::_setConfigurations(const Eigen::Vector3d& pcom_goal,
                                            MotionCommand &_motion_command){
    // set init & goal configurations
    p_init_ = robot_->getCoMPosition();  
    p_goal_ = pcom_goal;

    // my_utils::pretty_print(p_init_, std::cout, "p_init_");
    // my_utils::pretty_print(p_goal_, std::cout, "p_goal_");

    // next foot configuration
    swing_foot_link_idx_ = -1;
    SWING_DATA md;  
    if( _motion_command.get_foot_motion(md) ) {
        swing_foot_link_idx_ = MagnetoFoot::LinkIdx[md.foot_idx];     
        swing_foot_dpos_ = md.dpose.pos;        
        if(md.dpose.is_baseframe){
            Eigen::MatrixXd Rwb = robot_->getBodyNodeIsometry(MagnetoBodyNode::base_link).linear();
            swing_foot_dpos_ = Rwb*swing_foot_dpos_;
        }
    }
    else swing_foot_dpos_ = Eigen::VectorXd::Zero(3);
}

void MagnetoCoMPlanner::_setPeriods(const Eigen::VectorXd& periods){
    if(periods.size()==1) {        
        Tf_ = periods(0); Tt1_ = Tt_given_; Ts_ = periods(0); Tt2_ = Tt_given_;
    }
    else if(periods.size()==2){
        Tf_ = periods(0); Tt1_ = Tt_given_; Ts_ = periods(1); Tt2_ = Tt_given_;
    }
    else if(periods.size()==3){
        Tf_ = periods(0); Tt1_ = periods(1); Ts_ = periods(2); Tt2_ = periods(1); 
    }
    else if(periods.size()==4){
        Tf_ = periods(0); Tt1_ = periods(1); Ts_ = periods(2); Tt2_ = periods(3); 
    }
    else{
        Tf_ = 0.1; Tt1_ = 0.1; Ts_ = 0.1; Tt2_ = 0.1;
    }

    T1_ = Tf_;
    T2_ = Tt1_ + Ts_;
    T3_ = Tt2_;
}

void MagnetoCoMPlanner::_buildCentroidalSystemMatrices(){
    int dim_f = Rc_.cols();
    Ac_ = Eigen::MatrixXd::Zero(6, dim_f);
    Ac_.topRows(3) = Pc_-my_utils::skew(p_goal_)*Rc_;
    Ac_.bottomRows(3) = Rc_;

    Cc_ = Eigen::VectorXd::Zero(6);
    Cc_.head(3) = Ldot_ + (my_utils::skew(p_goal_)*Rc_-Pc_)*Fmc_;
    Cc_.tail(3) = -mass_*grav_ - Rc_*Fmc_;

    dim_f = Rf_.cols();
    Af_ = Eigen::MatrixXd::Zero(6, dim_f);
    Af_.topRows(3) = Pf_-my_utils::skew(p_goal_)*Rf_;
    Af_.bottomRows(3) = Rf_;

    Cf_ = Eigen::VectorXd::Zero(6);
    Cf_.head(3) = Ldot_ + (my_utils::skew(p_goal_)*Rf_-Pf_)*Fmf_;
    Cf_.tail(3) = -mass_*grav_ - Rf_*Fmf_;    
}


void MagnetoCoMPlanner::_buildFm(
    const std::array<MagnetSpec*, Magneto::n_leg>& f_mag){
    Fmf_ = Eigen::VectorXd::Zero(0);
    Fmc_ = Eigen::VectorXd::Zero(0);
    for(auto &mag : f_mag) {
        Fmf_= my_utils::vStack(Fmf_, -mag->getMagneticForce3d());
        if(mag->getLinkIdx() !=  swing_foot_link_idx_){
            Fmc_ = my_utils::vStack(Fmc_, -mag->getMagneticForce3d());
        }
    }    
}

void MagnetoCoMPlanner::_buildFrictionCone(
    const std::array<ContactSpec*, Magneto::n_leg>& f_contacts){
    Df_ = Eigen::MatrixXd::Zero(0,0);
    Dc_ = Eigen::MatrixXd::Zero(0,0);
    df_ = Eigen::VectorXd::Zero(0);
    dc_ = Eigen::VectorXd::Zero(0);

    Eigen::MatrixXd Di;
    Eigen::VectorXd di;
    // Di*Fc >= di
    for(auto &contact : f_contacts) {
        ((BodyFramePointContactSpec*)contact)->getRFConstraintMtx(Di);
        ((BodyFramePointContactSpec*)contact)->getRFConstraintVec(di);
        // exclude maximum fz
        Di = Di.topRows(5);
        di = di.head(5);
        Di(1, 0) = 1.;
        Df_ = my_utils::dStack(Df_, Di);
        df_ = my_utils::vStack(df_, di);
        if(contact->getLinkIdx() != swing_foot_link_idx_){
            Dc_ = my_utils::dStack(Dc_, Di);
            dc_ = my_utils::vStack(dc_, di);
        }
    }
}

void MagnetoCoMPlanner::_buildWeightMatrices(
    const std::array<ContactSpec*, Magneto::n_leg>& f_contacts){
    invWf_ = Eigen::MatrixXd::Zero(0,0);
    invWc_ = Eigen::MatrixXd::Zero(0,0);

    double mu;
    Eigen::VectorXd invwi;
    // Di*Fc >= di
    for(auto &contact : f_contacts) {
        mu = contact->getFrictionCoeff();
        // wi << 1., 1., mu^2;
        invwi = Eigen::VectorXd::Zero(3);
        // invwi << 1., 1., 1./mu/mu;
        // invwi << mu*mu, mu*mu, 1.;
        invwi << mu*mu, mu*mu, 1.;
        invWf_ = my_utils::dStack(invWf_, invwi.asDiagonal());
        if(contact->getLinkIdx() != swing_foot_link_idx_){
            invWc_ = my_utils::dStack(invWc_, invwi.asDiagonal());
        }
    }
}

void MagnetoCoMPlanner::_buildPfRf() {
    Pf_ = Eigen::MatrixXd::Zero(0,0);
    Rf_ = Eigen::MatrixXd::Zero(0,0);
    Pc_ = Eigen::MatrixXd::Zero(0,0);
    Rc_ = Eigen::MatrixXd::Zero(0,0);

    Eigen::MatrixXd PRi, Ri;
    Eigen::VectorXd pi;
    for(int i(0); i<Magneto::n_leg; ++i) {        
        pi = robot_->getBodyNodeIsometry(
            MagnetoFoot::LinkIdx[i]).translation();
        Ri = robot_->getBodyNodeIsometry(
            MagnetoFoot::LinkIdx[i]).linear();            
        PRi = my_utils::skew(pi)*Ri;
        
        if(MagnetoFoot::LinkIdx[i] != swing_foot_link_idx_){
            Pf_ = my_utils::hStack(Pf_, PRi);
            Rf_ = my_utils::hStack(Rf_, Ri);
            Pc_ = my_utils::hStack(Pc_, PRi);
            Rc_ = my_utils::hStack(Rc_, Ri);
        }
        else{
            // next desired foot position
            pi += swing_foot_dpos_;
            PRi = my_utils::skew(pi)*Ri;
            Pf_ = my_utils::hStack(Pf_, PRi);
            Rf_ = my_utils::hStack(Rf_, Ri);
        }
    }
}

void MagnetoCoMPlanner::_getSwingConditionReplan(Eigen::MatrixXd& DD,
                                                Eigen::VectorXd& dd) {
    // Centroidal Dynamics
    // A*Fc = B*alpha*dir + C
    // solve Fc with weighted inverse matirx
    // Fc = invA*(B*delP + C)

    Eigen::MatrixXd AWA = Af_*invWf_*Af_.transpose();
    Eigen::MatrixXd AWAinv;
    my_utils::pseudoInverse(AWA, 0.0001, AWAinv);

    Eigen::MatrixXd invA = invWf_*Af_.transpose()*AWAinv;
    Eigen::VectorXd ddf = - Df_*invA*Cf_ + df_;

    Eigen::MatrixXd invA1 = invA.leftCols<3>();
    Eigen::MatrixXd invA2 = invA.rightCols<3>();
    
    Eigen::MatrixXd DD1, DD2;
    // centroidal dynamics soln applied friction cone
    // D( 0.5*m*(t)^2*invA1*skew(g) + m*invA2)*alpha*pdir >= - D*invA*C + d    
    // when t=0
    // DD1*alpha*dir >= ddf
    DD1 = Df_* mass_*invA2;
    // when t=Ts
    // DD2*alpha*dir >= ddf
    DD2 = Df_*(0.5*mass_*(T2_*T2_)*invA1*my_utils::skew(grav_) + mass_*invA2);

    DD = my_utils::vStack(DD1,DD2);
    dd = my_utils::vStack(ddf,ddf);    
}


void MagnetoCoMPlanner::_getEndTransitionCondition(Eigen::MatrixXd& DD,
                                                   Eigen::VectorXd& dd) {
    // Centroidal Dynamics
    // A*Fc = B*beta*dir + C
    // solve Fc with weighted inverse matirx
    // Fc = invA*(S(s,sddot)*delP + C)

    Eigen::MatrixXd AWA = Af_*invWf_*Af_.transpose();
    Eigen::MatrixXd AWAinv;
    my_utils::pseudoInverse(AWA, 0.0001, AWAinv);

    Eigen::MatrixXd invA = invWf_*Af_.transpose()*AWAinv;
    Eigen::VectorXd ddf = - Df_*invA*Cf_ + df_;

    Eigen::MatrixXd invA1 = invA.leftCols<3>();
    Eigen::MatrixXd invA2 = invA.rightCols<3>();
    
    Eigen::MatrixXd DD1, DD2;
    // centroidal dynamics soln applied friction cone
    // D( 0.5*m*(Tt-t)^2*invA1*skew(g) + m*invA2)*beta*pdir + D*invA*C - d >=0    
    // when t=0
    // DD1*beta*dir - ddf >=0
    DD1 = Df_*( 0.5*mass_*(T3_*T3_)*invA1*my_utils::skew(grav_) + mass_*invA2);
    // when t=Tt
    // DD2*beta*dir - ddf >=0
    DD2 = Df_*mass_*invA2;

    DD = my_utils::vStack(DD1,DD2);
    dd = my_utils::vStack(ddf,ddf);
}


void MagnetoCoMPlanner::_getSwingConditionGivenRatio(double ratio,
                                                    Eigen::MatrixXd& DD,
                                                    Eigen::VectorXd& dd){
    // alpha = ratio*beta

    Eigen::MatrixXd AWA = Ac_*invWc_*Ac_.transpose();
    Eigen::MatrixXd AWAinv;
    my_utils::pseudoInverse(AWA, 0.0001, AWAinv);

    Eigen::MatrixXd invA = invWc_*Ac_.transpose()*AWAinv;
    Eigen::VectorXd ddc = - Dc_*invA*Cc_ + dc_;

    Eigen::MatrixXd invA1 = invA.leftCols<3>();
    Eigen::MatrixXd invA2 = invA.rightCols<3>();

    Eigen::MatrixXd DDa, DDb;
    // @ t=0    
    DDa = _computeSwingDDa(0., invA1, invA2);
    DDb = _computeSwingDDb(0., invA1);
    // @ t=T2
    DDa = my_utils::vStack(DDa, _computeSwingDDa(T2_, invA1, invA2));
    DDb = my_utils::vStack(DDb, _computeSwingDDb(T2_, invA1));
    dd = my_utils::vStack(ddc, ddc);

    // @ t=tstar (local maxima)
    double t_star = T2_ + T3_/2./ratio;
    if(t_star > 0. && t_star < T2_){
        DDa = my_utils::vStack(DDa, _computeSwingDDa(t_star, invA1, invA2));
        DDb = my_utils::vStack(DDb, _computeSwingDDb(t_star, invA1));
        dd = my_utils::vStack(dd, ddc);
    }
    
    DD = DDa*ratio+DDb;
}

Eigen::MatrixXd MagnetoCoMPlanner::_computeSwingDDa(double t,
                                            const Eigen::MatrixXd& invA1,
                                            const Eigen::MatrixXd& invA2){
    // Da =  D*( 0.5*m*(T2-t)^2*invA1*skew(g) + m*invA2 );
    return Dc_*( 0.5*mass_*(T2_-t)*(T2_-t)*invA1*my_utils::skew(grav_) + mass_*invA2);
}

Eigen::MatrixXd MagnetoCoMPlanner::_computeSwingDDb(double t,
                                            const Eigen::MatrixXd& invA1){
    // Db = D*( 0.5*m*T3*(T3+2*T2-t)*invA1*skew(g) );
    return Dc_*( 0.5*mass_*T3_*(T3_+2*T2_-t)*invA1*my_utils::skew(grav_) );
}

void MagnetoCoMPlanner::_solveQuadProgReplan(){
    /* -- 1. solve alpha for swing phase --*/
    Eigen::MatrixXd DDs;
    Eigen::VectorXd dds;

    _getSwingConditionReplan(DDs, dds);
    // solve quad prob for x = alpha*dir
    // min 0.5*x'*x
    // s.t. -DDs*x <= -dds
    std::cout<<"########################################################"<<std::endl;
    Eigen::VectorXd x;
    qp_solver_->setProblem(Eigen::MatrixXd::Identity(3,3),
                        Eigen::VectorXd::Zero(3),-DDs,-dds);
    qp_solver_->solveProblem(x);

    alpha_ = x.norm();

    if(std::fabs(alpha_) > 1e-5) dir_com_swing_ = x/alpha_;
    else dir_com_swing_ = Eigen::VectorXd::Zero(3);
   
    std::cout<<" dir_com_swing= " << dir_com_swing_.transpose() << std::endl;
    std::cout<<" alpha= " <<alpha_ << std::endl;
    if(alpha_>0.5) alpha_ = 0.5; 
    std::cout<<"########################################################"<<std::endl;

    /* --  2. get Tt2 --*/


}


void MagnetoCoMPlanner::_solveQuadProg(){
    Eigen::MatrixXd DDf, DDs, DD;
    Eigen::VectorXd ddf, dds, dd;

    // Phase 3
    _getEndTransitionCondition(DDf, ddf);

    // Phase 2
    // find ratio minimizing the maximum distance from com goal along the spline
    // alpha=ratio*beta
    double a = 0.5*T2_*T2_;
    double c = -0.5*T3_*T3_;
    double b = T3_*(T2_+T3_);
    double ratio = (-b - std::sqrt(b*b-4*a*c)) / 2/a;
    // std::cout<<"ratio = " <<ratio<<std::endl;
    _getSwingConditionGivenRatio(ratio, DDs, dds);

    // Phase 2 & 3
    DD = my_utils::vStack(DDf, DDs);
    dd = my_utils::vStack(ddf, dds);
    // my_utils::pretty_print(DDs,std::cout,"DD");
    // my_utils::pretty_print(dds,std::cout,"dd");

    std::cout<<"########################################################"<<std::endl;
    // solve quad prob for x = beta*dir
    // min 0.5*x'*x
    // s.t. -DD*x <= -dd
    Eigen::VectorXd x;
    qp_solver_->setProblem(Eigen::MatrixXd::Identity(3,3),
                        Eigen::VectorXd::Zero(3),-DD,-dd);
    qp_solver_->solveProblem(x);

    // set result
    if(ratio < 0) beta_ = - x.norm();
    else beta_ = x.norm();

    if(std::fabs(beta_) > 1e-5) dir_com_swing_ = x/beta_;
    else dir_com_swing_ = Eigen::VectorXd::Zero(3);

    alpha_ = beta_*ratio;
    
    std::cout<<" dir_com_swing= " << dir_com_swing_.transpose() << std::endl;
    std::cout<<" alpha= " <<alpha_ << ", beta= " <<beta_ << std::endl;
    if(alpha_>1.2) alpha_ = 1.2; 
    beta_ = alpha_/ratio;
    std::cout<<"########################################################"<<std::endl;
}
