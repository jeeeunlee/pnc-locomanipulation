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
    
    std::cout<<" foot [" << foot_idx << "] is moving at : ";
    std::cout<< "vel: "<< xcdot.transpose() << std::endl;
    std::cout<< "acc: "<< xcddot.transpose() << std::endl;
    
}

void SlipObserver::evaluate() {

}

void SlipObserver::initialization(const YAML::Node& node) {

}