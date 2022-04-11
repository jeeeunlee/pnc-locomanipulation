#pragma once
#include <utility>

#include <../my_utils/Configuration.h>
#include <my_utils/General/Clock.hpp>
#include <my_utils/IO/IOUtilities.hpp>
//#include <RobotSystem/include/CentroidModel.hpp>
#include <my_robot_core/magneto_core/magneto_definition.hpp>
#include <my_robot_core/magneto_core/magneto_command_api.hpp>

class RobotSystem;
// class MotionCommand;
// class SimulationCommand;

class MagnetoStateProvider {
   public:
    static MagnetoStateProvider* getStateProvider(RobotSystem* _robot);
    ~MagnetoStateProvider() {}

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

    double curr_time;
    
    // 
    int curr_state;
    MotionCommand curr_motion_command;
    SimulationCommand curr_simulation_command;
    int num_state; // num of remaining states to run
    //

    Eigen::VectorXd q_des;
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd tau_cmd_prev;

    Eigen::VectorXd jpos_ini;

    int b_arfoot_contact;
    int b_brfoot_contact;
    int b_alfoot_contact;
    int b_blfoot_contact;

    std::array<Eigen::Vector3d, Magneto::n_leg> surface_normal;

    /* -------------- com planner ---------------*/

    // save feasibile com for the plot
    std::vector<std::pair<double, Eigen::Vector3d>> feasible_com_list;

    Eigen::Vector3d com_pos_init;
    Eigen::Vector3d com_pos_target;
    int check_com_planner_updated;
    // foot desired position
    Eigen::VectorXd foot_pos_init;
    Eigen::VectorXd foot_pos_target;
    int check_foot_planner_updated;


    /*-------------- data manager ---------------*/

    Eigen::Vector3d com_pos;
    Eigen::Vector3d com_vel;
    Eigen::VectorXd mom;

    Eigen::Vector3d com_pos_des;
    Eigen::Vector3d com_vel_des;
    Eigen::VectorXd mom_des;

    Eigen::VectorXd arf_pos;
    Eigen::VectorXd arf_vel;
    Eigen::VectorXd brf_pos;
    Eigen::VectorXd brf_vel;
    Eigen::VectorXd alf_pos;
    Eigen::VectorXd alf_vel;
    Eigen::VectorXd blf_pos;
    Eigen::VectorXd blf_vel;

    Eigen::VectorXd arf_pos_des;
    Eigen::VectorXd arf_vel_des;
    Eigen::VectorXd brf_pos_des;
    Eigen::VectorXd brf_vel_des;
    Eigen::VectorXd alf_pos_des;
    Eigen::VectorXd alf_vel_des;
    Eigen::VectorXd blf_pos_des;
    Eigen::VectorXd blf_vel_des;

    Eigen::Quaternion<double> base_ori;
    Eigen::VectorXd base_ang_vel;

    Eigen::VectorXd al_rf_des;
    Eigen::VectorXd bl_rf_des;
    Eigen::VectorXd ar_rf_des;
    Eigen::VectorXd br_rf_des;

    Eigen::VectorXd al_rf;        
    Eigen::VectorXd bl_rf;
    Eigen::VectorXd ar_rf;
    Eigen::VectorXd br_rf;

   private:
    MagnetoStateProvider(RobotSystem* _robot);
    RobotSystem* robot_;
};
