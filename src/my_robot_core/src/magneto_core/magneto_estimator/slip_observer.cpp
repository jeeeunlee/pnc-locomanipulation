#include <my_robot_core/magneto_core/magneto_wbc_controller/containers/wbc_spec_container.hpp>
#include <my_robot_core/magneto_core/magneto_estimator/slip_observer.hpp>


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

void SlipObserver::checkForce()
{
    Eigen::MatrixXd Jc;
    Eigen::VectorXd JcDotQdot;

    int swingfootlinkidx = sp_->curr_motion_command.get_moving_foot();
    int contact_link_idx;
    for ( auto &[foot_idx, contact] : ws_container_->foot_contact_map_){
        contact_link_idx = ((BodyFrameSurfaceContactSpec*)(contact))->getLinkIdx();
        // foot in contact
        if( contact_link_idx != swingfootlinkidx) {
            contact->updateContactSpec();
            contact->getContactJacobian(Jc);
            contact->getJcDotQdot(JcDotQdot);
        } // swing foot
        else {

        }
        
    }
        

    Eigen::MatrixXd A;
    Eigen::MatrixXd Ainv;
    Eigen::MatrixXd grav;
    Eigen::MatrixXd coriolis;

    A = robot_->getMassMatrix();
    Ainv = robot_->getInvMassMatrix();
    grav = robot_->getGravity();
    coriolis = robot_->getCoriolis();

    Eigen::VectorXd grf_des;
    Eigen::VectorXd grf_act;
    // fc = inv(Jc *Ainv * JcT) * ( S'tau + Jc Ainv JsT fa - Jc ddq - Jc Ainv (c+g) )
    grf_des = ()


    // sensor data (actual reaction force)
    switch(foot_idx){
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

    


}

void SlipObserver::checkJointConfiguration(int foot_idx)
{
    // joint desired vs actual joint

}

void SlipObserver::evaluate() {

}

void SlipObserver::initialization(const YAML::Node& node) {

}