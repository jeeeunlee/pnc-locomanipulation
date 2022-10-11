#include <my_utils/Math/hermite_quaternion_curve.hpp>

// hermite orientation spline on so(3) 
// Note that we implement the global frame case not the local frame!

HermiteQuaternionCurve::HermiteQuaternionCurve(){ }

HermiteQuaternionCurve::HermiteQuaternionCurve(
    const Eigen::Quaterniond& quat_start,
    const Eigen::Vector3d& angular_velocity_start,
    const Eigen::Quaterniond& quat_end,
    const Eigen::Vector3d& angular_velocity_end,
    double duration) {
  initialize(quat_start, angular_velocity_start, quat_end,
             angular_velocity_end, duration);
}

void HermiteQuaternionCurve::initialize(
    const Eigen::Quaterniond& quat_start,
    const Eigen::Vector3d& angular_velocity_start,
    const Eigen::Quaterniond& quat_end,
    const Eigen::Vector3d& angular_velocity_end,
    double duration) {
  qa = quat_start;
  omega_a = angular_velocity_start;

  qb = quat_end;
  omega_b = angular_velocity_end;

  t_dur = duration;

  initialize_data_structures();
}

HermiteQuaternionCurve::~HermiteQuaternionCurve(){}

void HermiteQuaternionCurve::initialize_data_structures(){
  // 
  // q(t) = exp( theta(t) ) * qa : global frame
  // q(t) = qa * exp( theta(t) ) : local frame
  // where theta(t) is hermite cubic spline with
  // theta(0) = 0, theta(t_dur) = log(delq_ab)
  // dot_theta(0) = omega_a, dot_theta(1) = omega_b

  Eigen::AngleAxisd delq_ab = Eigen::AngleAxisd(qb*qa.inverse());
  // Eigen::AngleAxisd del_qab = qa.inverse()*qb;

  Eigen::VectorXd start_pos = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd start_vel = omega_a;
  Eigen::VectorXd end_pos = delq_ab.axis() * delq_ab.angle(); 
  Eigen::VectorXd end_vel = omega_b;

  theta_ab.initialize(start_pos, start_vel, end_pos, end_vel, t_dur);
}

void HermiteQuaternionCurve::evaluate(const double & t_in, Eigen::Quaterniond & quat_out){
  Eigen::VectorXd delq_vec = theta_ab.evaluate(t_in);

  if(delq_vec.norm() < 1e-6)
    delq = Eigen::Quaterniond(1, 0, 0, 0);
  else 
    delq = Eigen::AngleAxisd(delq_vec.norm(), delq_vec/delq_vec.norm());
  // quat_out = q0 * delq; // local frame
  quat_out = delq * qa; // global frame
}

void HermiteQuaternionCurve::getAngularVelocity(const double & t_in, Eigen::Vector3d & ang_vel_out){
  ang_vel_out = theta_ab.evaluateFirstDerivative(t_in);
}

// For world frame
void HermiteQuaternionCurve::getAngularAcceleration(const double & t_in, Eigen::Vector3d & ang_acc_out){
  ang_acc_out = theta_ab.evaluateSecondDerivative(t_in);
  // not sure about this
}

void HermiteQuaternionCurve::printQuat(const Eigen::Quaterniond & quat){
  std::cout <<  quat.x() << " " <<
                quat.y() << " " <<
                quat.z() << " " <<
                quat.w() << " " << std::endl;
}
