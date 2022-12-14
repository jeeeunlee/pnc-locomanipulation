#include <my_robot_system/RobotSystem.hpp>
#include <my_robot_core/anymal_core/anymal_state_provider.hpp>

ANYmalStateProvider* ANYmalStateProvider::getStateProvider(
    RobotSystem* _robot) {
    static ANYmalStateProvider state_provider_(_robot);
    return &state_provider_;
}

ANYmalStateProvider::ANYmalStateProvider(RobotSystem* _robot) {
    my_utils::pretty_constructor(1, "ANYmal State Provider");

    robot_ = _robot;

    //
    curr_time = 0.;    
    curr_state = -1;
    curr_motion_command = MotionCommand();    
    curr_manipulation_command = ManipulationCommand();
    num_state = 0;
    //

    q = Eigen::VectorXd::Zero(ANYmal::n_dof);
    qdot = Eigen::VectorXd::Zero(ANYmal::n_dof);
    q_des = Eigen::VectorXd::Zero(ANYmal::n_dof);
    tau_cmd_prev = Eigen::VectorXd::Zero(ANYmal::n_dof);
    jpos_ini = Eigen::VectorXd::Zero(ANYmal::n_adof);
    b_foot_contact = {0,0,0,0};
    
    
    /* -------------- com/foot planner ---------------*/
    feasible_com_list.clear();

    com_pos_init.setZero();
    com_pos_target.setZero();

    foot_pos_init = Eigen::VectorXd::Zero(3);
    foot_pos_target = Eigen::VectorXd::Zero(3);
    ee_pos_init = Eigen::VectorXd::Zero(3);
    ee_pos_target = Eigen::VectorXd::Zero(3);    

    check_com_planner_updated = 0;
    check_foot_planner_updated = 0;
    check_ee_planner_updated = 0;

    /*-------------- data manager ---------------*/
    com_pos.setZero();
    com_vel.setZero();
    mom = Eigen::VectorXd::Zero(6);

    com_pos_des.setZero();
    com_vel_des.setZero();
    mom_des = Eigen::VectorXd::Zero(6);

    for(int i=0;i<ANYmal::n_leg;++i){
        foot_pos[i] = Eigen::VectorXd::Zero(3);
        foot_pos_des[i] = Eigen::VectorXd::Zero(3);
        foot_vel[i] = Eigen::VectorXd::Zero(3);
        foot_vel_des[i] = Eigen::VectorXd::Zero(3);
        foot_rf[i] = Eigen::VectorXd::Zero(6);
        foot_rf_des[i] = Eigen::VectorXd::Zero(6);
    }

    base_ori = Eigen::Quaternion<double>::Identity();
    base_ang_vel = Eigen::VectorXd::Zero(3);
}

void ANYmalStateProvider::saveCurrentData() {

    com_pos = robot_->getCoMPosition();
    com_vel = robot_->getCoMVelocity();
    mom = robot_->getCentroidMomentum();

    for(int i=0;i<ANYmal::n_leg;++i){
        foot_pos[i] = robot_->getBodyNodeIsometry(
                        ANYmalFoot::LinkIdx[i]).translation();        
        foot_vel[i] = robot_->getBodyNodeSpatialVelocity(
                        ANYmalFoot::LinkIdx[i]).tail(3);
    }
}

// functions

Eigen::VectorXd ANYmalStateProvider::getActiveJointValue()
{
    return getActiveJointValue(q);
}

Eigen::VectorXd ANYmalStateProvider::getActiveJointValue(const Eigen::VectorXd& q_full){
    Eigen::VectorXd q_a(ANYmal::n_adof);
    for(int i = 0; i < ANYmal::n_adof; ++i)        
        q_a[i] = q_full[ANYmal::idx_adof[i]]; 
    return  q_a;
}

Eigen::VectorXd ANYmalStateProvider::getVirtualJointValue(){
    return getVirtualJointValue(q);
}

Eigen::VectorXd ANYmalStateProvider::getVirtualJointValue(const Eigen::VectorXd& q_full){
    Eigen::VectorXd q_v(ANYmal::n_vdof);
    for(int i = 0; i < ANYmal::n_vdof; ++i)    
        q_v[i] = q_full[ANYmal::idx_vdof[i]];
    return q_v;
}

Eigen::VectorXd ANYmalStateProvider::getFullJointValue(const Eigen::VectorXd& q_a)
{
    Eigen::VectorXd q_full(ANYmal::n_dof);
    for(int i = 0; i < ANYmal::n_vdof; ++i)    
        q_full[ANYmal::idx_vdof[i]] = 0.0;
    for(int i = 0; i < ANYmal::n_adof; ++i)        
        q_full[ANYmal::idx_adof[i]] = q_a[i]; 
    return q_full;
}

Eigen::VectorXd ANYmalStateProvider::getFullJointValue(const Eigen::VectorXd& q_a, const Eigen::VectorXd& q_v)
{
    Eigen::VectorXd q_full(ANYmal::n_dof);
    for(int i = 0; i < ANYmal::n_vdof; ++i)    
        q_full[ANYmal::idx_vdof[i]] = q_v[i];
    for(int i = 0; i < ANYmal::n_adof; ++i)        
        q_full[ANYmal::idx_adof[i]] = q_a[i]; 
    return q_full;
}

void ANYmalStateProvider::divideJoints2AnV(const Eigen::VectorXd& q_full, Eigen::VectorXd& q_a, Eigen::VectorXd& q_v) {
    if(q_full.size() == ANYmal::n_dof)
    {
        q_a = getActiveJointValue(q_full);
        q_v = getVirtualJointValue(q_full);
    }    
}