
#include "../my_utils/Configuration.h"

#include <math.h>
#include <stdio.h>

#include <my_utils/IO/IOUtilities.hpp>
#include <my_utils/Math/MathUtilities.hpp>
#include <my_test/test_system.hpp>
#include "my_robot_core/anymal_core/anymal_definition.hpp"
#include <string>

class floatingRobotData{
  public:
    floatingRobotData(int _ndof): n_adof(_ndof){
        std::cout<<"floatingRobotData made"<<std::endl;
        base_joint_pos.setZero();
        base_joint_quat.setIdentity();
        base_joint_lin_vel.setZero();
        base_joint_ang_vel.setZero();

        joint_pos = Eigen::VectorXd::Zero(n_adof);
        joint_vel = Eigen::VectorXd::Zero(n_adof);
    }
    Eigen::Vector3d base_joint_pos;
    Eigen::Quaterniond base_joint_quat;
    Eigen::Vector3d base_joint_lin_vel; 
    Eigen::Vector3d base_joint_ang_vel;
    Eigen::VectorXd joint_pos;
    Eigen::VectorXd joint_vel;
    int n_adof;
};
void read_data(const YAML::Node& cfg, floatingRobotData* data){
    Eigen::VectorXd temp;

    my_utils::readParameter(cfg, "pose", data->base_joint_pos);   
    my_utils::readParameter(cfg, "quat", temp);
    data->base_joint_quat = Eigen::Quaternion<double>(
                        temp[0], temp[1], temp[2], temp[3] );
    my_utils::readParameter(cfg, "leg_config", temp); 
    data->joint_pos.head(12) = temp;    
    my_utils::readParameter(cfg, "arm_config", temp);
    data->joint_pos.tail(6) = temp;    

    my_utils::readParameter(cfg, "pose_vel", data->base_joint_lin_vel);  
    my_utils::readParameter(cfg, "ori_vel", data->base_joint_ang_vel);  
    my_utils::readParameter(cfg, "q_vel", data->joint_vel);              
}


void test_robot_system(RobotSystem* robot, floatingRobotData* data){

    Eigen::VectorXd curr_cofig = Eigen::VectorXd::Zero(ANYmal::n_dof);
    Eigen::VectorXd curr_qdot = Eigen::VectorXd::Zero(ANYmal::n_dof);

    curr_cofig.segment(0,3) = data->base_joint_pos;
    curr_cofig.segment(3,3) = my_utils::convertQuatToEulerAngles(data->base_joint_quat);
    curr_cofig.tail(ANYmal::n_adof) = data->joint_pos;

    curr_qdot.segment(0,3) = data->base_joint_lin_vel;
    curr_qdot.segment(3,3) = data->base_joint_ang_vel; // ??
    curr_qdot.tail(ANYmal::n_adof) = data->joint_vel;
    robot->updateSystem(curr_cofig, curr_qdot, true);

    Eigen::VectorXd q = robot->getQ();
    Eigen::VectorXd qdot = robot->getQdot();

    // check link J, JcDotQdot
    std::array<int, 6> check_link_id = { 
        ANYmalBodyNode::base,    ANYmalBodyNode::LF_FOOT, 
        ANYmalBodyNode::RF_FOOT, ANYmalBodyNode::LH_FOOT, 
        ANYmalBodyNode::RH_FOOT, ANYmalBodyNode::ur3_ee_link };
    // base, LF_FOOT, RF_FOOT, LH_FOOT, RH_FOOT, ur3_ee_link
    std::cout<<" ========================== " << std::endl;
    my_utils::pretty_print(q, std::cout, "q");
    my_utils::pretty_print(qdot, std::cout, "qdot");
    // my_utils::pretty_print(data->base_joint_pos, std::cout, "Jbase_joint_posb");
    // my_utils::pretty_print(data->base_joint_quat, std::cout, "base_joint_quat");
    // my_utils::pretty_print(data->base_joint_lin_vel, std::cout, "base_joint_lin_vel");
    // my_utils::pretty_print(data->base_joint_ang_vel, std::cout, "base_joint_ang_vel");
    // my_utils::pretty_print(data->joint_pos, std::cout, "joint_pos");
    // my_utils::pretty_print(data->joint_vel, std::cout, "joint_vel");

    for(auto &link_idx : check_link_id) {
        Eigen::MatrixXd Jb = robot->getBodyNodeBodyJacobian(link_idx);
        Eigen::VectorXd JbDotQdot = robot->getBodyNodeBodyJacobianDot(link_idx)*qdot;
        Eigen::MatrixXd J = robot->getBodyNodeJacobian(link_idx);
        Eigen::VectorXd JDotQdot = robot->getBodyNodeJacobianDot(link_idx)*qdot;

        std::cout<<"----- link_idx = "<< link_idx << " : " <<std::endl;
        my_utils::pretty_print(Jb, std::cout, "Jb");
        my_utils::pretty_print(JbDotQdot, std::cout, "JbDotQdot");
        my_utils::pretty_print(J, std::cout, "J");
        my_utils::pretty_print(JDotQdot, std::cout, "JDotQdot");
    } 
}



int main(int argc, char** argv) {

    const char* filename = nullptr;
    if (argc >  1) filename = argv[1];    
    else filename = THIS_COM "config/ANYmal/TEST_CONFIGURATION.yaml";

    RobotSystem* robot = new RobotSystem(6, THIS_COM "/robot_description/Robot/ANYmal/anymal_ur3.urdf"); //ANYmalwithArm
    robot->setActuatedJoint(ANYmal::idx_adof);
    floatingRobotData* data = new floatingRobotData(ANYmal::n_adof);

    try {
        YAML::Node cfg = YAML::LoadFile(filename);
        read_data(cfg["config1"], data);
        test_robot_system( robot , data );
        
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                << __FILE__ << "]" << std::endl
                << std::endl;
    }   
    
}

