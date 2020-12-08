#pragma once

#include "my_pnc/EnvInterface.hpp"
#include "my_pnc/MagnetoPnC/MagnetoDefinition.hpp"
#include "my_pnc/MagnetoPnC/MagnetoMotionAPI.hpp"
#include "my_pnc/MagnetoPnC/MagnetoLogicInterrupt/WalkingInterruptLogic.hpp"

class MagnetoStateProvider;
class MagnetoStateEstimator;
// class StaticWalkingPatternParam;
// class BalancingPatternParam;

namespace RUN_MODE {
constexpr int BALANCE = 0;
constexpr int STATICWALK = 1;

};  // namespace BLPhase

class MagnetoSensorData {
   public:
    MagnetoSensorData() {
        q = Eigen::VectorXd::Zero(Magneto::n_adof);
        qdot = Eigen::VectorXd::Zero(Magneto::n_adof);
        virtual_q = Eigen::VectorXd::Zero(Magneto::n_vdof);
        virtual_qdot = Eigen::VectorXd::Zero(Magneto::n_vdof);

        alf_wrench = Eigen::VectorXd::Zero(6);
        blf_wrench = Eigen::VectorXd::Zero(6);
        arf_wrench = Eigen::VectorXd::Zero(6);
        brf_wrench = Eigen::VectorXd::Zero(6);
        alfoot_contact = false;
        blfoot_contact = false;
        arfoot_contact = false;
        brfoot_contact = false;

        R_ground = Eigen::MatrixXd::Identity(3,3);
    }
    virtual ~MagnetoSensorData() {}

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd virtual_q;
    Eigen::VectorXd virtual_qdot;
    Eigen::VectorXd alf_wrench;
    Eigen::VectorXd blf_wrench;
    Eigen::VectorXd arf_wrench;
    Eigen::VectorXd brf_wrench;
    bool alfoot_contact;
    bool blfoot_contact;
    bool arfoot_contact;
    bool brfoot_contact;

    Eigen::MatrixXd R_ground;
};

class MagnetoCommand {
   public:
    MagnetoCommand() {
        q = Eigen::VectorXd::Zero(Magneto::n_adof);
        qdot = Eigen::VectorXd::Zero(Magneto::n_adof);
        jtrq = Eigen::VectorXd::Zero(Magneto::n_adof);

        b_magnetism_map[MagnetoBodyNode::AL_foot_link] = false;
        b_magnetism_map[MagnetoBodyNode::BL_foot_link] = false;
        b_magnetism_map[MagnetoBodyNode::AR_foot_link] = false;
        b_magnetism_map[MagnetoBodyNode::BR_foot_link] = false;
    }
    virtual ~MagnetoCommand() {}

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd jtrq;

    // bool alfoot_magnetism_on;
    // bool blfoot_magnetism_on;
    // bool arfoot_magnetism_on;
    // bool brfoot_magnetism_on;
    std::map<int, bool> b_magnetism_map;

    // double alfoot_magnetism_on; // 0~1
    // double blfoot_magnetism_on; // 0~1
    // double arfoot_magnetism_on; // 0~1
    // double brfoot_magnetism_on; // 0~1
};

class MagnetoInterface : public EnvInterface {
   protected:
    void _ParameterSetting();
    bool _Initialization(MagnetoSensorData*, MagnetoCommand*);
    bool _CheckCommand(MagnetoCommand* cmd);
    void _SetStopCommand(MagnetoSensorData*, MagnetoCommand* cmd);
    void _SaveDataCmd(MagnetoSensorData*, MagnetoCommand* cmd);

    std::string test_name_;

    MagnetoStateEstimator* state_estimator_;
    MagnetoStateProvider* sp_;


    int count_;
    int waiting_count_;
    Eigen::VectorXd cmd_jpos_;
    Eigen::VectorXd cmd_jvel_;
    Eigen::VectorXd cmd_jtrq_;

    double prev_planning_moment_;
    int check_com_planner_updated;
    int check_foot_planner_updated;
    int run_mode_;

   public:
    MagnetoInterface();
    virtual ~MagnetoInterface();

    virtual void getCommand(void* _sensor_data, void* _command_data);
    int getRunMode() {return run_mode_;}

    void StaticWalk(const int& _moving_foot,
                    const Eigen::Vector3d& _pos, 
                    const Eigen::Quaternion<double>& _ori, 
                    const double& _motion_period,
                    const double& _swing_height,
                    bool _is_bodyframe );
    void AddScriptWalkMotion(int _link_idx, 
                            const MOTION_DATA& _motion_data);
    
    void GetCoMTrajectory(std::vector<Eigen::VectorXd>& com_des_list);
    void GetContactSequence(std::vector<Eigen::Isometry3d>& foot_target_list);
    bool IsTrajectoryUpdated();

    void GetFeasibleCoM(std::vector <std::pair<double, Eigen::Vector3d>>& 
                        feasible_com_list);
    void GetCurrentCoM(Eigen::VectorXd& com_pos);
    void GetOptimalCoM(Eigen::VectorXd& com_pos);
    void GetCurrentFootStep(Eigen::VectorXd& foot_pos);
    void GetNextFootStep(Eigen::VectorXd& foot_pos);
    
    bool IsPlannerUpdated();
    bool IsFootPlannerUpdated();
};
