#include <my_robot_core/magneto_core/magneto_wbc_controller/containers/wbc_spec_container.hpp>
#include <my_robot_core/magneto_core/magneto_estimator/slip_observer.hpp>
#include <my_robot_core/magneto_core/magneto_control_architecture/magneto_control_architecture_set.hpp>
#include <my_utils/Math/pseudo_inverse.hpp>

SlipObserver::SlipObserver( MagnetoWbcSpecContainer* ws_container,
              RobotSystem* _robot) : StateEstimator(_robot) {
  my_utils::pretty_constructor(2, "StateEstimator: SlipObserver");

  // Set Pointer to wbc spec / reference generator container
  ws_container_ = ws_container;

  // Get State Provider
  sp_ = MagnetoStateProvider::getStateProvider(robot_);

  // set parameters  
  initParams();
  initContact();

  t_updated_ = sp_->curr_time;
  b_swing_phase_ = false;
}

SlipObserver::~SlipObserver() {}

void SlipObserver::initialization(const YAML::Node& node){

}

void SlipObserver::initParams(){
    Sa_ = Eigen::MatrixXd::Zero(Magneto::n_adof, Magneto::n_dof);
    for(int i=0; i< Magneto::n_adof; ++i){ Sa_(i, Magneto::idx_adof[i]) = 1.; }
}

void SlipObserver::initContact(){
    int dim_grf;
    for ( auto &[foot_idx, contact] : ws_container_->foot_contact_map_)
    {
        b_foot_contact_map_[foot_idx] =  true;
        dim_grf = contact->getDim();

        dim_grf_map_[foot_idx] = dim_grf;
        foot_vel_map_[foot_idx] = Eigen::VectorXd::Zero(dim_grf); 
        foot_acc_map_[foot_idx] = Eigen::VectorXd::Zero(dim_grf);         
        grf_des_map_[foot_idx] = Eigen::VectorXd::Zero(dim_grf);        
    }   
}

void SlipObserver::updateContact(){
    // one time update
    if( t_updated_ < sp_->curr_time ) {
        t_updated_ = sp_->curr_time;
        b_swing_phase_ = (sp_->curr_state == MAGNETO_STATES::SWING);

        // update kinematics
        q_ = robot_->getQ();
        qdot_ = robot_->getQdot();
        qddot_ = robot_->getQddot();  

        // update dynamics
        Minv_ = robot_->getInvMassMatrix();
        grav_ = robot_->getGravity();
        coriolis_ = robot_->getCoriolis();       

        // update contact
        initContact();
        swing_foot_link_idx_ = sp_->curr_motion_command.get_moving_foot();        
        for ( auto &[foot_idx, contact] : ws_container_->foot_contact_map_){
            contact->updateContactSpec();
            if( b_swing_phase_ && swing_foot_link_idx_ ==
                ((BodyFrameSurfaceContactSpec*)(contact))->getLinkIdx() ) {
                    b_foot_contact_map_[foot_idx] = false; }            
        }

        // update sensed GRF value
        grf_act_map_[MagnetoFoot::AL] = sp_->al_rf;
        grf_act_map_[MagnetoFoot::BL] = sp_->bl_rf;
        grf_act_map_[MagnetoFoot::AR] = sp_->ar_rf;
        grf_act_map_[MagnetoFoot::BR] = sp_->br_rf;
    }
}

void SlipObserver::checkVelocity(){
    updateContact();
    checkVelocityFoot(MagnetoFoot::AL);
    checkVelocityFoot(MagnetoFoot::BL);
    checkVelocityFoot(MagnetoFoot::AR);
    checkVelocityFoot(MagnetoFoot::BR);
}

void SlipObserver::checkVelocityFoot(int foot_idx) {
  // if velocity at contact > 50mm/s significant amount?
  // TODO : add threshold in climb_param.yaml
    updateContact();

    auto contact = ws_container_->foot_contact_map_[foot_idx];
    
    Eigen::MatrixXd Jc;
    Eigen::VectorXd JcDotQdot;

    contact->getContactJacobian(Jc);
    contact->getJcDotQdot(JcDotQdot);

    Eigen::VectorXd xcdot = Jc*qdot_;
    Eigen::VectorXd xcddot = Jc*qddot_ + JcDotQdot;
    
    // double linear_velocity_threshold = 0.05;
    // if( foot_idx==swing_foot_link_idx_ && 
    //     xcdot.tail(3).norm() > linear_velocity_threshold){
    //     std::cout<<" foot [" << foot_idx << "] is moving at : ";
    //     std::cout<< "vel: "<< xcdot.transpose() << std::endl;
    //     std::cout<< "acc: "<< xcddot.transpose() << std::endl;
    // }

    foot_vel_map_[foot_idx] = xcdot;
    foot_acc_map_[foot_idx] = xcddot;

    // data saving
    std::string foot_vel_name = MagnetoFootNames[foot_idx] + "_vel";
    std::string foot_acc_name = MagnetoFootNames[foot_idx] + "_acc";
    my_utils::saveVector(xcdot, foot_vel_name);    
    my_utils::saveVector(xcddot, foot_acc_name);
}

void SlipObserver::checkForce() {

    // std::cout<<" checkForce " << std::endl;
    // update grf_act_map_
    updateContact();    

    // compute desired value
    Eigen::VectorXd tau = sp_->tau_cmd_prev;
    Eigen::VectorXd grf_des_stacked = computeGRFDesired(tau);
    
    int dim_grf_stacked = 0;
    int dim_grf=0;
    for ( auto &[foot_idx, b_contact] : b_foot_contact_map_) {
        // desired force (estimated from observation)
        if(b_contact){
            dim_grf = dim_grf_map_[foot_idx];
            grf_des_map_[foot_idx] = grf_des_stacked.segment(dim_grf_stacked, dim_grf);
            dim_grf_stacked += dim_grf;         }    
    }

    // DATA SAVING
    std::string filename;
    Eigen::VectorXd grf_tmp;
    for( auto &[foot_idx, b_contact] : b_foot_contact_map_ ) {
        filename = MagnetoFootNames[foot_idx] + "_grf_act_des";
        dim_grf = dim_grf_map_[foot_idx];
        grf_tmp = Eigen::VectorXd::Zero(2*dim_grf);
        grf_tmp.head(dim_grf) = grf_act_map_[foot_idx];
        grf_tmp.tail(dim_grf) = grf_des_map_[foot_idx];
        my_utils::saveVector(grf_tmp, filename);
    }
}

void SlipObserver::weightShaping() {
    // reshape the weight w.r.t grf ("ws_container_->W_rf_")
    // climbing_param.yaml
    // w_rf = 1.0, w_rf_z = 0.01 / w_rf_z_nocontact = 0.5
    // 
    // if there exists a foot in slippery, we would want to 
    // increase the normal force & decrease the tangential force
    // should decrease w_rf_z, increase w_rf
    
    double linear_velocity_threshold = 0.05;
    double slip_level = 1.0;
    
    for( auto& [foot_idx, xcdot] : foot_vel_map_) {
        if( !b_swing_phase_ && foot_idx!=swing_foot_link_idx_ ) { 
            // detect slip
            slip_level = xcdot.tail(3).norm() / linear_velocity_threshold;
            if( slip_level > 1.0 ){         
                std::cout<<" foot [" << foot_idx << "] is sliding at : ";
                std::cout<< "vel: "<< xcdot.transpose() << std::endl;
                std::cout<< "slip_level (alpha) = "<< slip_level << std::endl;  

                ws_container_->reshape_weight_param( slip_level,
                                        ws_container_->footIdx2FootLink(foot_idx),
                                        ws_container_->W_rf_ );

            }
        }
    }
}


Eigen::VectorXd SlipObserver::computeGRFDesired(const Eigen::VectorXd& tau) {    
    
    Eigen::MatrixXd Jc_i, Jc, Js;
    Eigen::VectorXd JcDotQdot_i, JcDotQdot;
    Eigen::VectorXd fa = Eigen::VectorXd::Zero(6); // swing foot adhesive force

    int dim_grf= 0;
    for ( auto &[foot_idx, contact] : ws_container_->foot_contact_map_ ) {       
        // foot in contact
        if( b_foot_contact_map_[foot_idx] ) {
            contact->getContactJacobian(Jc_i);
            contact->getJcDotQdot(JcDotQdot_i);

            // stack Matrices & Vectors
            if(dim_grf==0){
                Jc = Jc_i;
                JcDotQdot = JcDotQdot_i;                
            }else{
                Jc.conservativeResize(dim_grf + Jc_i.rows(), Magneto::n_dof);
                Jc.block(dim_grf, 0, Jc_i.rows(), Magneto::n_dof) = Jc_i;            
                JcDotQdot.conservativeResize(dim_grf + Jc_i.rows());
                JcDotQdot.tail(Jc_i.rows()) = JcDotQdot_i;                
            }
            dim_grf += Jc_i.rows();            
        } // swing foot
        else {
            contact->getContactJacobian(Js);
            int fz_idx = ((BodyFrameSurfaceContactSpec*)(contact))->getFzIndex();    
            fa[fz_idx] =  ws_container_->residual_force_[foot_idx];
        }
    }

    Eigen::MatrixXd Ainv;
    Eigen::MatrixXd A = Jc*Minv_*Jc.transpose();
    my_utils::pseudoInverse(A, 0.0001, Ainv);
    
    Eigen::VectorXd grf_des = Eigen::VectorXd::Zero(dim_grf);
    // f_c = inv(Jc *Minv_ * JcT) * ( S'tau + Jc Minv_ JsT fa - Jc ddq - Jc Minv_ (c+g) )
    if(b_swing_phase_) {
        grf_des = Ainv * ( Jc*Minv_*( Sa_.transpose()*tau  + Js.transpose()*fa )
                        - Jc*qddot_ - Jc*Minv_*(coriolis_ + grav_) );
    }else {
        grf_des = Ainv * ( Jc*Minv_*Sa_.transpose()*tau
                        - Jc*qddot_ - Jc*Minv_*(coriolis_ + grav_) );    }

    return grf_des;
}

// void SlipObserver::checkJointConfiguration(int foot_idx) {
//     // joint desired vs actual joint
// }

void SlipObserver::evaluate() {

}

