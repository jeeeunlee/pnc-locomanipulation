#pragma once
#include <utility>

#include <../my_utils/Configuration.h>
#include <my_utils/General/Clock.hpp>
#include <my_utils/IO/IOUtilities.hpp>
//#include <RobotSystem/include/CentroidModel.hpp>
#include <my_robot_core/anymal_core/anymal_definition.hpp>
#include <my_robot_core/anymal_core/anymal_command_api.hpp>

class RobotSystem;


class ANYmalStateProvider {
   public:
    static ANYmalStateProvider* getStateProvider(RobotSystem* _robot);
    ~ANYmalStateProvider() {}

    void saveCurrentData();
    void divideJoints2AnV(const Eigen::VectorXd& q_full, 
                        Eigen::VectorXd& q_a, Eigen::VectorXd& q_v);    
   
    Eigen::VectorXd getActiveJointValue();
    Eigen::VectorXd getVirtualJointValue();
    Eigen::VectorXd getActiveJointValue(const Eigen::VectorXd& q_full);
    Eigen::VectorXd getVirtualJointValue(const Eigen::VectorXd& q_full);
    Eigen::VectorXd getFullJointValue(const Eigen::VectorXd& q_a);
    Eigen::VectorXd getFullJointValue(const Eigen::VectorXd& q_a, const Eigen::VectorXd& q_v);

    Clock clock;
    
    
    // 
    double curr_time;
    int curr_state;
    MotionCommand curr_motion_command;
    ManipulationCommand curr_manipulation_command;
    int num_state; // num of remaining states to run
    //

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd q_des;
    Eigen::VectorXd tau_cmd_prev;

    Eigen::VectorXd jpos_ini;

    std::array<int, ANYmal::n_leg> b_foot_contact;


    /* -------------- com/foot planner ---------------*/

    // save feasibile com for the plot
    std::vector<std::pair<double, Eigen::Vector3d>> feasible_com_list;

    Eigen::Vector3d com_pos_init;
    Eigen::Vector3d com_pos_target;
    int check_com_planner_updated;
    // foot desired position
    Eigen::VectorXd foot_pos_init;
    Eigen::VectorXd foot_pos_target;
    int check_foot_planner_updated;
    // EE desired position
    Eigen::VectorXd ee_pos_init;
    Eigen::VectorXd ee_pos_target;
    int check_ee_planner_updated;

    /*-------------- data manager ---------------*/
    Eigen::Vector3d com_pos;
    Eigen::Vector3d com_vel;
    Eigen::VectorXd mom;

    Eigen::Vector3d com_pos_des;
    Eigen::Vector3d com_vel_des;
    Eigen::VectorXd mom_des;

    Eigen::Quaternion<double> base_ori;
    Eigen::VectorXd base_ang_vel;

    std::array<Eigen::VectorXd, ANYmal::n_leg> foot_pos;
    std::array<Eigen::VectorXd, ANYmal::n_leg> foot_pos_des;
    std::array<Eigen::VectorXd, ANYmal::n_leg> foot_vel;
    std::array<Eigen::VectorXd, ANYmal::n_leg> foot_vel_des;

    std::array<Eigen::VectorXd, ANYmal::n_leg> foot_rf;
    std::array<Eigen::VectorXd, ANYmal::n_leg> foot_rf_des;

    Eigen::VectorXd arm_pos;
    Eigen::VectorXd arm_pos_des;
    Eigen::VectorXd arm_vel;  
    Eigen::VectorXd arm_vel_des;

   private:
    ANYmalStateProvider(RobotSystem* _robot);
    RobotSystem* robot_;
};
