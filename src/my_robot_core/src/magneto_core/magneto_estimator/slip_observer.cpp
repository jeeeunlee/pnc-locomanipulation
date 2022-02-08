#include <my_robot_core/magneto_core/magneto_wbc_controller/containers/wbc_spec_container.hpp>
#include <my_robot_core/magneto_core/magneto_estimator/slip_observer.hpp>
#include <my_utils/Math/pseudo_inverse.hpp>

SlipObserver::SlipObserver( MagnetoWbcSpecContainer* ws_container,
              RobotSystem* _robot) : StateEstimator(_robot) {
  my_utils::pretty_constructor(2, "StateEstimator: SlipObserver");

  // Set Pointer to wbc spec / reference generator container
  ws_container_ = ws_container;

  // Get State Provider
  sp_ = MagnetoStateProvider::getStateProvider(robot_);

}

SlipObserver::~SlipObserver() {}

void SlipObserver::checkVelocity(int foot_idx) {

  // if velocity at contact > 50mm/s significant amount?
  // TODO : add threshold in climb_param.yaml

    auto contact = ws_container_->foot_contact_map_[foot_idx];
    
    Eigen::MatrixXd Jc;
    Eigen::VectorXd JcDotQdot;

    contact->updateContactSpec();
    contact->getContactJacobian(Jc);
    contact->getJcDotQdot(JcDotQdot);

    Eigen::VectorXd q = robot_->getQ();
    Eigen::VectorXd qdot = robot_->getQdot();
    Eigen::VectorXd qddot = robot_->getQddot();  

    Eigen::VectorXd xcdot = Jc*qdot;
    Eigen::VectorXd xcddot = Jc*qddot + JcDotQdot;
    
    double linear_velocity_threshold = 0.05;
    if( xcdot.tail(3).norm() > linear_velocity_threshold){
        std::cout<<" foot [" << foot_idx << "] is moving at : ";
        std::cout<< "vel: "<< xcdot.transpose() << std::endl;
        std::cout<< "acc: "<< xcddot.transpose() << std::endl;
    }

    std::string foot_vel_name = "foot_vel_"+ std::to_string(foot_idx);
    my_utils::saveVector(xcdot,foot_vel_name);    
}

Eigen::VectorXd SlipObserver::computeForceDesired(const Eigen::VectorXd& tau) {    
    
    Eigen::MatrixXd Jc_i, Jc, Js;
    Eigen::VectorXd JcDotQdot_i, JcDotQdot;
    Eigen::VectorXd fa = Eigen::VectorXd::Zero(6); // swing foot adhesive force

    int swingfootlinkidx = sp_->curr_motion_command.get_moving_foot();
    int dim_rf= 0;
    bool swing_phase = false;
    for ( auto &[foot_idx, contact] : ws_container_->foot_contact_map_) {        
        contact->updateContactSpec();
        // foot in contact
        if( ((BodyFrameSurfaceContactSpec*)(contact))->getLinkIdx()
                                                != swingfootlinkidx ) {
            contact->getContactJacobian(Jc_i);
            contact->getJcDotQdot(JcDotQdot_i);

            // stack Matrices & Vectors
            if(dim_rf==0){
                Jc = Jc_i;
                JcDotQdot = JcDotQdot_i;                
            }else{
                Jc.conservativeResize(dim_rf + Jc_i.rows(), Magneto::n_dof);
                Jc.block(dim_rf, 0, Jc_i.rows(), Magneto::n_dof) = Jc_i;            
                JcDotQdot.conservativeResize(dim_rf + Jc_i.rows());
                JcDotQdot.tail(Jc_i.rows()) = JcDotQdot_i;                
            }
            dim_rf += Jc_i.rows();
        } // swing foot
        else {
            swing_phase = true;
            contact->getContactJacobian(Js);
            int fz_idx = ((BodyFrameSurfaceContactSpec*)(contact))->getFzIndex();    
            fa[fz_idx] =  ws_container_->residual_force_[foot_idx];
        }
    }

    Eigen::MatrixXd M;
    Eigen::MatrixXd Minv;
    Eigen::MatrixXd grav;
    Eigen::MatrixXd coriolis;

    M = robot_->getMassMatrix();
    Minv = robot_->getInvMassMatrix();
    grav = robot_->getGravity();
    coriolis = robot_->getCoriolis();

    Eigen::VectorXd qddot = robot_->getQddot();

    // f_c = inv(Jc *Minv * JcT) * ( S'tau + Jc Minv JsT fa - Jc ddq - Jc Minv (c+g) )
    Eigen::VectorXd grf_des = Eigen::VectorXd::Zero(dim_rf);   
    
    Eigen::MatrixXd Ainv;
    Eigen::MatrixXd A = Jc*Minv*Jc.transpose();
    my_utils::pseudoInverse(A, 0.0001, Ainv);
   
    Eigen::VectorXd Sa = Eigen::MatrixXd::Zero(Magneto::n_adof, Magneto::n_dof);
    for(int i=0; i< Magneto::n_adof; ++i){ Sa(i, Magneto::idx_adof[i]) = 1.; }

    if(swing_phase) {        
        grf_des = Ainv * ( Jc*Minv*( Sa.transpose()*tau  + Js.transpose()*fa )
                        - Jc*qddot - Jc*Minv*(coriolis + grav) );
    }else {
        grf_des = Ainv * ( Jc*Minv*Sa.transpose()*tau
                        - Jc*qddot - Jc*Minv*(coriolis + grav) );
    }
    return grf_des;
}

void SlipObserver::checkForce(const Eigen::VectorXd& tau) {

    Eigen::VectorXd grf_des_stacked = computeForceDesired(tau);
    Eigen::VectorXd grf_act = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd grf_des = Eigen::VectorXd::Zero(6);

    std::map<int, int> dim_rf_map;
    int swingfootlinkidx = sp_->curr_motion_command.get_moving_foot();
    for ( auto &[foot_idx, contact] : ws_container_->foot_contact_map_) {  
        if( ((BodyFrameSurfaceContactSpec*)(contact))->getLinkIdx()
            != swingfootlinkidx) { dim_rf_map[foot_idx] = contact->getDim(); }        
    }

    std::cout<<"swingfootlinkidx = " << swingfootlinkidx << std::endl;
    my_utils::pretty_print(grf_des_stacked, std::cout, "grf_des_stacked");
    


    int dim_rf_stacked = 0;
    for ( auto &[foot_idx, dim_rf] : dim_rf_map ) {
        // desired force (estimated from observation)
        grf_des = grf_des_stacked.segment(dim_rf_stacked, dim_rf);
        std::cout<<"foot_idx = " << foot_idx << ", dim_rf = "<< dim_rf << std::endl;
        my_utils::pretty_print(grf_des, std::cout, "grf_des");
        dim_rf_stacked += dim_rf;
        
        // sensor data (actual reaction force)
        switch(foot_idx) {
            case MagnetoFoot::AL:
                grf_act = sp_->al_rf;
                break;
            case MagnetoFoot::BL:
                grf_act = sp_->bl_rf;
                break;
            case MagnetoFoot::AR:
                grf_act = sp_->ar_rf;
                break;
            case MagnetoFoot::BR:
                grf_act = sp_->br_rf;
                break;
        }

        std::cout << "grf @ foot " << foot_idx <<"["<<dim_rf<<"]: des / act"<< std::endl;
        for(int i(0); i<dim_rf; ++i){
            std::cout << grf_des[i] <<",";
        }
        std::cout<<std::endl;
        for(int i(0); i<dim_rf; ++i){
            std::cout << grf_act[i] <<",";
        }
        std::cout<<std::endl;
    }
}

// void SlipObserver::checkJointConfiguration(int foot_idx) {
//     // joint desired vs actual joint
// }

void SlipObserver::evaluate() {

}

void SlipObserver::initialization(const YAML::Node& node) {

}