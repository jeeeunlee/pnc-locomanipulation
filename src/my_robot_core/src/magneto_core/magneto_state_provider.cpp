#include <my_robot_system/RobotSystem.hpp>
#include <my_robot_core/magneto_core/magneto_state_provider.hpp>

MagnetoStateProvider* MagnetoStateProvider::getStateProvider(
    RobotSystem* _robot) {
    static MagnetoStateProvider state_provider_(_robot);
    return &state_provider_;
}

MagnetoStateProvider::MagnetoStateProvider(RobotSystem* _robot) {
    my_utils::pretty_constructor(1, "Magneto State Provider");

    robot_ = _robot;
    curr_time = 0.;    
    
    curr_state = -1;
    curr_motion_command = MotionCommand();
    curr_simulation_command = SimulationCommand();
    num_state = 0;

    q = Eigen::VectorXd::Zero(Magneto::n_dof);
    qdot = Eigen::VectorXd::Zero(Magneto::n_dof);
    tau_cmd_prev = Eigen::VectorXd::Zero(Magneto::n_dof);

    surface_normal = {Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ(), 
                        Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ()};    

    b_arfoot_contact = 0;
    b_brfoot_contact = 0;
    b_alfoot_contact = 0;
    b_blfoot_contact = 0;
    

    foot_pos_target = Eigen::VectorXd::Zero(3);

    feasible_com_list.clear();

    check_com_planner_updated = 0;
    check_foot_planner_updated = 0;

    com_pos.setZero();
    com_vel.setZero();
    mom = Eigen::VectorXd::Zero(6);

    com_pos_des.setZero();
    com_vel_des.setZero();
    mom_des = Eigen::VectorXd::Zero(6);

    arf_pos = Eigen::VectorXd::Zero(3);
    arf_vel = Eigen::VectorXd::Zero(3);
    brf_pos = Eigen::VectorXd::Zero(3);
    brf_vel = Eigen::VectorXd::Zero(3);

    alf_pos = Eigen::VectorXd::Zero(3);
    alf_vel = Eigen::VectorXd::Zero(3);
    blf_pos = Eigen::VectorXd::Zero(3);
    blf_vel = Eigen::VectorXd::Zero(3);

    arf_pos_des = Eigen::VectorXd::Zero(3);
    arf_vel_des = Eigen::VectorXd::Zero(3);
    brf_pos_des = Eigen::VectorXd::Zero(3);
    brf_vel_des = Eigen::VectorXd::Zero(3);

    alf_pos_des = Eigen::VectorXd::Zero(3);
    alf_vel_des = Eigen::VectorXd::Zero(3);
    blf_pos_des = Eigen::VectorXd::Zero(3);
    blf_vel_des = Eigen::VectorXd::Zero(3);

    base_ori = Eigen::Quaternion<double>::Identity();
    base_ang_vel = Eigen::VectorXd::Zero(3);

    ar_rf = Eigen::VectorXd::Zero(6);
    br_rf = Eigen::VectorXd::Zero(6);
    al_rf = Eigen::VectorXd::Zero(6);
    bl_rf = Eigen::VectorXd::Zero(6);

    ar_rf_des = Eigen::VectorXd::Zero(6);
    br_rf_des = Eigen::VectorXd::Zero(6);
    al_rf_des = Eigen::VectorXd::Zero(6);
    bl_rf_des = Eigen::VectorXd::Zero(6);

}

void MagnetoStateProvider::saveCurrentData() {
    for (int i = 0; i < 3; ++i) {
        com_pos[i] = robot_->getCoMPosition()[i];
        com_vel[i] = robot_->getCoMVelocity()[i];
    }

    mom = robot_->getCentroidMomentum();

    // depending on how to deal with gimbal joints
    // AL_foot_link? or AL_foot_link_3
    arf_pos = robot_->getBodyNodeIsometry(MagnetoBodyNode::AR_foot_link)
                 .translation(); // todo: rightCOP_Frame 
    arf_vel = robot_->getBodyNodeSpatialVelocity(MagnetoBodyNode::AR_foot_link ) 
            .tail(3); // rightCOP_Frame
    brf_pos = robot_->getBodyNodeIsometry(MagnetoBodyNode::BR_foot_link)
                 .translation(); // todo: rightCOP_Frame
    brf_vel = robot_->getBodyNodeSpatialVelocity(MagnetoBodyNode::BR_foot_link ) 
            .tail(3); // rightCOP_Frame            
    alf_pos = robot_->getBodyNodeIsometry(MagnetoBodyNode::AL_foot_link ) 
                 .translation(); // leftCOP_Frame
    alf_vel = robot_->getBodyNodeSpatialVelocity(MagnetoBodyNode::AL_foot_link ) 
                 .tail(3);//leftCOP_Frame
    blf_pos = robot_->getBodyNodeIsometry(MagnetoBodyNode::BL_foot_link ) 
                 .translation(); // leftCOP_Frame
    blf_vel = robot_->getBodyNodeSpatialVelocity(MagnetoBodyNode::BL_foot_link ) 
                 .tail(3);//leftCOP_Frame

}


Eigen::VectorXd MagnetoStateProvider::getActiveJointValue()
{
    return getActiveJointValue(q);
}

Eigen::VectorXd MagnetoStateProvider::getActiveJointValue(const Eigen::VectorXd& q_full){
    Eigen::VectorXd q_a(Magneto::n_adof);
    for(int i = 0; i < Magneto::n_adof; ++i)        
        q_a[i] = q_full[Magneto::idx_adof[i]]; 

    return  q_a;
}

Eigen::VectorXd MagnetoStateProvider::getVirtualJointValue(){
    return getVirtualJointValue(q);
}

Eigen::VectorXd MagnetoStateProvider::getVirtualJointValue(const Eigen::VectorXd& q_full){
    Eigen::VectorXd q_v(Magneto::n_vdof);
    for(int i = 0; i < Magneto::n_vdof; ++i)    
        q_v[i] = q_full[Magneto::idx_vdof[i]];

    return q_v;
}

Eigen::VectorXd MagnetoStateProvider::getFullJointValue(const Eigen::VectorXd& q_a)
{
    Eigen::VectorXd q_full(Magneto::n_dof);
    for(int i = 0; i < Magneto::n_vdof; ++i)    
        q_full[Magneto::idx_vdof[i]] = 0.0;
    for(int i = 0; i < Magneto::n_adof; ++i)        
        q_full[Magneto::idx_adof[i]] = q_a[i]; 
    return q_full;
}

Eigen::VectorXd MagnetoStateProvider::getFullJointValue(const Eigen::VectorXd& q_a, const Eigen::VectorXd& q_v)
{
    Eigen::VectorXd q_full(Magneto::n_dof);
    for(int i = 0; i < Magneto::n_vdof; ++i)    
        q_full[Magneto::idx_vdof[i]] = q_v[i];
    for(int i = 0; i < Magneto::n_adof; ++i)        
        q_full[Magneto::idx_adof[i]] = q_a[i]; 
    return q_full;
}

void MagnetoStateProvider::divideJoints2AnV(const Eigen::VectorXd& q_full, Eigen::VectorXd& q_a, Eigen::VectorXd& q_v) {
    if(q_full.size() == Magneto::n_dof)
    {
        q_a = getActiveJointValue(q_full);
        q_v = getVirtualJointValue(q_full);
    }    
}