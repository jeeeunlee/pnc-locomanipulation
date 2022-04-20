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

  // lpf2_container_
  for(int i(0); i<Magneto::n_leg; ++i){
    lpf2_container_[i] = new LowPassFilter2();    
  }

  // set parameters  
  initParams();
  initContact();

  t_updated_ = sp_->curr_time;
  b_swing_phase_ = false;
}

SlipObserver::~SlipObserver() { 
    for( auto &lpf2 : lpf2_container_)
        delete lpf2;
}

void SlipObserver::initialization(const YAML::Node& node) {
    try { 
        my_utils::readParameter(node,"slip_velocity_threshold", lin_vel_thres_);  
        my_utils::readParameter(node,"weight_shaping", weight_shaping_activated_); 
        my_utils::readParameter(node,"lpf_vel_cutoff", lpf_vel_cutoff_);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                << __FILE__ << "]" << std::endl
                << std::endl;
        exit(0);
    }  

    for(int i(0); i<Magneto::n_leg; ++i){
        lpf2_container_[i]->initialize(
            ws_container_->feet_contacts_[i]->getDim(), lpf_vel_cutoff_); 
    }
}

void SlipObserver::initParams(){
    Sa_ = Eigen::MatrixXd::Zero(Magneto::n_adof, Magneto::n_dof);
    for(int i=0; i< Magneto::n_adof; ++i){ Sa_(i, Magneto::idx_adof[i]) = 1.; }
    weight_shaping_activated_=0;
    lin_vel_thres_=0.02;
}

void SlipObserver::initContact(){
    int dim_grf;
    for( int foot_idx(0); foot_idx<Magneto::n_leg; foot_idx++ )
    {
        b_foot_contact_map_[foot_idx] =  true;
        dim_grf = ws_container_->feet_contacts_[foot_idx]->getDim();

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
        swing_foot_idx_ = sp_->curr_motion_command.get_moving_foot();        
        for( int foot_idx(0); foot_idx<Magneto::n_leg; foot_idx++ ){
            ws_container_->feet_contacts_[foot_idx]->updateContactSpec();
            if( b_swing_phase_ && swing_foot_idx_ == foot_idx ) {
                    b_foot_contact_map_[foot_idx] = false; }            
        }

        // update sensed GRF value
        grf_act_map_[MagnetoFoot::AL] = sp_->al_rf;
        grf_act_map_[MagnetoFoot::BL] = sp_->bl_rf;
        grf_act_map_[MagnetoFoot::AR] = sp_->ar_rf;
        grf_act_map_[MagnetoFoot::BR] = sp_->br_rf;

        grf_des_map2_[MagnetoFoot::AL] = sp_->al_rf_des;
        grf_des_map2_[MagnetoFoot::BL] = sp_->bl_rf_des;
        grf_des_map2_[MagnetoFoot::AR] = sp_->ar_rf_des;
        grf_des_map2_[MagnetoFoot::BR] = sp_->br_rf_des;
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

    auto contact = ws_container_->feet_contacts_[foot_idx];
    
    Eigen::MatrixXd Jc;
    Eigen::VectorXd JcDotQdot;

    contact->getContactJacobian(Jc);
    contact->getJcDotQdot(JcDotQdot);

    Eigen::VectorXd xcdot = Jc*qdot_;
    Eigen::VectorXd xcddot = Jc*qddot_ + JcDotQdot;

    Eigen::VectorXd xcdot_filtered = lpf2_container_[foot_idx]->update(xcdot);
    
    foot_vel_map_[foot_idx] = xcdot_filtered; // xcdot;
    foot_acc_map_[foot_idx] = xcddot;

    // data saving
    std::string foot_vel_name = MagnetoFoot::Names[foot_idx] + "_vel";
    // my_utils::saveVector(xcdot, foot_vel_name);    

    foot_vel_name = MagnetoFoot::Names[foot_idx] + "_vel_filtered";
    my_utils::saveVector(xcdot_filtered, foot_vel_name);    
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
            dim_grf_stacked += dim_grf;         
        } else{
            grf_des_map_[foot_idx] = Eigen::VectorXd::Zero(6);
        }    
    }

    // DATA SAVING
    std::string filename;
    Eigen::VectorXd grf_act_des;
    for( auto &[foot_idx, b_contact] : b_foot_contact_map_ ) {
        filename = MagnetoFoot::Names[foot_idx] + "_grf_act_des";
        dim_grf = dim_grf_map_[foot_idx];
        grf_act_des = Eigen::VectorXd::Zero(2*dim_grf);
        grf_act_des.head(dim_grf) = grf_act_map_[foot_idx];
        // grf_act_des.tail(dim_grf) = grf_des_map_[foot_idx];
        grf_act_des.tail(dim_grf) = grf_des_map2_[foot_idx];       
        // my_utils::pretty_print(grf_act_des, std::cout, "grf_act_des"); 
        my_utils::saveVector(grf_act_des, filename);
    }
}

void SlipObserver::weightShaping() {
    // reshape the weight w.r.t grf ("ws_container_->W_rf_")
    // climbing_param.yaml
    // w_rf = 1.0, w_rf_z = 0.001 / w_rf_z_nocontact = 0.05
    // 
    // if there exists a foot in slippery, we would want to 
    // increase the normal force & decrease the tangential force
    // should decrease w_rf_z, increase w_rf
    
    if(weight_shaping_activated_==0) return;

    double slip_level = 1.0;    
    for( auto& [foot_idx, xcdot] : foot_vel_map_) {
        if( foot_idx!=foot_idx ) { //!b_swing_phase_ && 
            // detect slip
            slip_level = xcdot.tail(3).norm() / lin_vel_thres_;
            if( slip_level > 1.0 ) {
                // std::cout<< " slip detected at foot[" << foot_idx << "], under swing foot[";
                // std::cout<<foot_idx<<"], phase="<<sp_->curr_state<<std::endl;
                
                ws_container_->reshape_weight_param( slip_level,  
                                MagnetoFoot::LinkIdx[foot_idx], 
                                MagnetoFoot::LinkIdx[swing_foot_idx_] );
            }            
        }
    }
}


Eigen::VectorXd SlipObserver::computeGRFDesired(const Eigen::VectorXd& tau) {    
    
    Eigen::MatrixXd Jc_i, Jc, Js;
    Eigen::VectorXd JcDotQdot_i, JcDotQdot;
    Eigen::VectorXd fa = Eigen::VectorXd::Zero(6); // swing foot adhesive force

    int dim_grf= 0;
    for( int foot_idx(0); foot_idx<Magneto::n_leg; foot_idx++ ) {       
        // foot in contact
        if( b_foot_contact_map_[foot_idx] ) {
            ws_container_->feet_contacts_[foot_idx]->getContactJacobian(Jc_i);
            ws_container_->feet_contacts_[foot_idx]->getJcDotQdot(JcDotQdot_i);

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
            ws_container_->feet_contacts_[foot_idx]->getContactJacobian(Js);
            int fz_idx = ws_container_->feet_contacts_[foot_idx]->getFzIndex();    
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

