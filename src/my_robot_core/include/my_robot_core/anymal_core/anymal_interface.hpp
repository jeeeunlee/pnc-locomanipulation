
#pragma once

#include "my_robot_core/env_interface.hpp"
#include "my_robot_core/anymal_core/anymal_definition.hpp"
#include "my_robot_core/anymal_core/anymal_command_api.hpp"


class ANYmalStateProvider;
class ANYmalStateEstimator;

namespace RUN_MODE {
constexpr int BALANCE = 0;
constexpr int STATICWALK = 1;
constexpr int MPCCLIMBING = 2;
};  // namespace RUN_MODE

class ANYmalSensorData {
   public:
    ANYmalSensorData() {
        elapsedtime = 0.;
        q = Eigen::VectorXd::Zero(ANYmal::n_adof);
        qdot = Eigen::VectorXd::Zero(ANYmal::n_adof);
        virtual_q = Eigen::VectorXd::Zero(ANYmal::n_vdof+1);
        virtual_qdot = Eigen::VectorXd::Zero(ANYmal::n_vdof);

        foot_wrench = {Eigen::VectorXd::Zero(6),Eigen::VectorXd::Zero(6),
                        Eigen::VectorXd::Zero(6),Eigen::VectorXd::Zero(6)};
        b_foot_contact = {false,false,false,false};
        tau_cmd_prev = Eigen::VectorXd::Zero(ANYmal::n_adof);
    }
    virtual ~ANYmalSensorData() {}

    double elapsedtime;
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd virtual_q;
    Eigen::VectorXd virtual_qdot;

    std::array<Eigen::VectorXd, ANYmal::n_leg> foot_wrench;
    std::array<bool, ANYmal::n_leg> b_foot_contact;

    Eigen::VectorXd tau_cmd_prev;
};

class ANYmalCommand {
   public:
    ANYmalCommand() {
        q = Eigen::VectorXd::Zero(ANYmal::n_adof);
        qdot = Eigen::VectorXd::Zero(ANYmal::n_adof);
        jtrq = Eigen::VectorXd::Zero(ANYmal::n_adof);
    }
    virtual ~ANYmalCommand() {}

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd jtrq;
};

class ANYmalInterface : public EnvInterface {
   protected:
    bool _Initialization(ANYmalSensorData*, ANYmalCommand*);
    bool _CheckCommand(ANYmalCommand* cmd);
    void _SetStopCommand(ANYmalSensorData*, ANYmalCommand* cmd);
    void _SaveDataCmd(ANYmalSensorData*, ANYmalCommand* cmd);
    void _ParameterSetting(const YAML::Node& cfg);

    std::string test_name_;

    ANYmalStateEstimator* state_estimator_;
    ANYmalStateProvider* sp_;


    int count_;
    int waiting_count_;
    Eigen::VectorXd cmd_jpos_;
    Eigen::VectorXd cmd_jvel_;
    Eigen::VectorXd cmd_jtrq_;

    int check_com_planner_updated;
    int check_foot_planner_updated;

   public:
    ANYmalInterface();
    virtual ~ANYmalInterface();

    virtual void getCommand(void* _sensor_data, void* _command_data);   

    void GetFeasibleCoM(std::vector <std::pair<double, Eigen::Vector3d>>& 
                        feasible_com_list);
    void GetCurrentCoM(Eigen::VectorXd& com_pos);
    void GetOptimalCoM(Eigen::VectorXd& com_pos);
    void GetCurrentFootStep(Eigen::VectorXd& foot_pos);
    void GetNextFootStep(Eigen::VectorXd& foot_pos);

    void GetCoMPlans(Eigen::VectorXd& com_pos_ini,
                    Eigen::VectorXd& com_pos_goal);
    
    bool IsPlannerUpdated();
    bool IsFootPlannerUpdated();
};
