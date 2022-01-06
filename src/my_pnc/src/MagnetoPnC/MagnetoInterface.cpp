#include <math.h>
#include <stdio.h>
#include <my_robot_system/RobotSystem.hpp>
#include <my_pnc/MagnetoPnC/MagnetoCtrlArchitecture/MagnetoCtrlArchitecture.hpp>
#include <my_pnc/MagnetoPnC/MagnetoInterface.hpp>
#include <my_pnc/MagnetoPnC/MagnetoStateEstimator.hpp>
#include <my_pnc/MagnetoPnC/MagnetoStateProvider.hpp>
#include <my_pnc/MagnetoPnC/MagnetoMotionAPI.hpp>
#include <my_utils/IO/IOUtilities.hpp>
#include <my_utils/Math/MathUtilities.hpp>
#include <string>

MagnetoInterface::MagnetoInterface() : EnvInterface() {
    // MagnetoInterface 
    std::string border = "=";
    for (int i = 0; i < 79; ++i) {
        border += "=";
    }
    my_utils::color_print(myColor::BoldCyan, border);
    my_utils::pretty_constructor(0, "Magneto Interface");

    // set run_mode from interface.yaml
    _ParameterSetting(); 

    // declare
    robot_ = new RobotSystem(
        6+3*4, THIS_COM "robot_description/Robot/Magneto/MagnetoSim_Dart.urdf");
    robot_->setActuatedJoint(Magneto::idx_adof);
    // robot_->setRobotMass();
    // robot_->printRobotInfo();    

    state_estimator_ = new MagnetoStateEstimator(robot_);
    sp_ = MagnetoStateProvider::getStateProvider(robot_);

    control_architecture_ = new MagnetoControlArchitecture(robot_);

    switch(run_mode_) {
        case RUN_MODE::BALANCE:
        case RUN_MODE::STATICWALK:
            interrupt_ = new WalkingInterruptLogic(
            static_cast<MagnetoControlArchitecture*>(control_architecture_));
        break;
        case RUN_MODE::MPCCLIMBING:
            interrupt_ = new ClimbingInterruptLogic(
            static_cast<MagnetoControlArchitecture*>(control_architecture_));  
        break;
        default:
        break;
    }    

    sp_->stance_foot = MagnetoBodyNode::base_link; // todo base_link

    count_ = 0;
    waiting_count_ = 2;
    cmd_jpos_ = Eigen::VectorXd::Zero(Magneto::n_adof);
    cmd_jvel_ = Eigen::VectorXd::Zero(Magneto::n_adof);
    cmd_jtrq_ = Eigen::VectorXd::Zero(Magneto::n_adof);

    prev_planning_moment_ = 0.;
    check_com_planner_updated = 0;
    check_foot_planner_updated = 0;

    

    my_utils::color_print(myColor::BoldCyan, border);
}

MagnetoInterface::~MagnetoInterface() {
    delete robot_;
    delete state_estimator_;
    delete control_architecture_;
    delete interrupt_;
}

void MagnetoInterface::getCommand(void* _data, void* _command) {
    MagnetoCommand* cmd = ((MagnetoCommand*)_command);
    MagnetoSensorData* data = ((MagnetoSensorData*)_data);
    

    if(!_Initialization(data, cmd)) {
        state_estimator_->Update(data); // robot skelPtr in robotSystem updated 
        interrupt_->processInterrupts();
        control_architecture_->getCommand(cmd);
        
        if(!_CheckCommand(cmd)) { _SetStopCommand(data,cmd); }    
    }   

    // save data
    _SaveDataCmd(data,cmd);

    running_time_ = ((double)count_)*MagnetoAux::servo_rate;
    sp_->curr_time = running_time_;
    sp_->phase_copy = control_architecture_->getState();
    ++count_;
}

bool MagnetoInterface::_Initialization(MagnetoSensorData* data,
                                        MagnetoCommand* _command) {
    static bool test_initialized(false);
    if (!test_initialized) {
        control_architecture_->ControlArchitectureInitialization();
        test_initialized = true;
    }
    if (count_ < waiting_count_) {
         _SetStopCommand(data, _command);
        state_estimator_->Initialization(data);
        return true;
    }
    return false;
}

void MagnetoInterface::_ParameterSetting() {
    try {
        YAML::Node cfg =
            YAML::LoadFile(THIS_COM "config/Magneto/INTERFACE.yaml");
        std::string test_name =
            my_utils::readParameter<std::string>(cfg, "test_name");
        if (test_name == "static_walking_test") {
            run_mode_ = RUN_MODE::STATICWALK;
        } else if (test_name == "balance_test") {
            run_mode_ = RUN_MODE::BALANCE;
        } else if (test_name == "mpc_climbing_test") {
            run_mode_ = RUN_MODE::MPCCLIMBING;
        } else {
            printf(
            "[Magneto Interface] There is no matching test with the "
            "name\n");
            exit(0);
        }
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}

bool MagnetoInterface::_CheckCommand(MagnetoCommand* cmd){
    // return false, if it fails to update test command
    // e.g.) check limit cmd->q, cmd->qdot, cmd-<jtrq 
    return true;
}

void MagnetoInterface::_SetStopCommand(MagnetoSensorData* data, MagnetoCommand* cmd) {
  for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
    cmd->jtrq[i] = 0.;
    cmd->q[i] = data->q[i];
    cmd->qdot[i] = 0.;
  }
}

void MagnetoInterface::_SaveDataCmd(MagnetoSensorData* data, MagnetoCommand* cmd)  {
    // cmd->jtrq = my_utils::CropVector(cmd->jtrq,
    // robot_->GetTorqueLowerLimits(), robot_->GetTorqueUpperLimits(), "clip trq in interface");
    cmd_jtrq_ = cmd->jtrq;
    cmd_jvel_ = cmd->qdot;
    cmd_jpos_ = cmd->q;
}

bool MagnetoInterface::IsTrajectoryUpdated() {
    if (prev_planning_moment_ == sp_->planning_moment) {
        prev_planning_moment_ = sp_->planning_moment;
        return false;
    } else {
        prev_planning_moment_ = sp_->planning_moment;
        return true;
    }
}

bool MagnetoInterface::IsPlannerUpdated() {
    if (check_com_planner_updated == sp_->check_com_planner_updated) {
        return false;
    } else {
        check_com_planner_updated = sp_->check_com_planner_updated;
        return true;
    }   
}


bool MagnetoInterface::IsFootPlannerUpdated() {
    if (check_foot_planner_updated == sp_->check_foot_planner_updated) {
        return false;
    } else {
        check_foot_planner_updated = sp_->check_foot_planner_updated;
        return true;
    }   
}

void MagnetoInterface::GetCoMTrajectory(
    std::vector<Eigen::VectorXd>& com_des_list) {
    com_des_list = sp_->com_des_list;
}
void MagnetoInterface::GetContactSequence(
    std::vector<Eigen::Isometry3d>& foot_target_list) {
    foot_target_list = sp_->foot_target_list;
}

void MagnetoInterface::GetFeasibleCoM(
    std::vector <std::pair<double, Eigen::Vector3d>>& feasible_com_list) {
    feasible_com_list = sp_->feasible_com_list;
}

void MagnetoInterface::GetCurrentCoM(Eigen::VectorXd& com_des) {
    com_des = sp_->com_pos_init;
}

void MagnetoInterface::GetOptimalCoM(Eigen::VectorXd& com_des) {
    com_des = sp_->com_pos_target;
}

void MagnetoInterface::GetCurrentFootStep(Eigen::VectorXd& foot_pos) {
    foot_pos = sp_->foot_pos_init;
}

void MagnetoInterface::GetNextFootStep(Eigen::VectorXd& foot_pos) {
    foot_pos = sp_->foot_pos_target;
}

///////////////////////////////////////////////////////////////////////////////////


// void MagnetoInterface::AddScriptWalkMotion(int _link_idx, 
//                                         const MOTION_DATA& _motion_data) {

//     // motion_param->set_walking_pattern(_moving_foot, _pos, _ori, _motion_period, _is_bodyframe);
//     MotionCommand motion_command = MotionCommand(_link_idx,_motion_data);
//     ((WalkingInterruptLogic*)interrupt_)
//         ->motion_command_script_list_.push_back(motion_command);
// }
void MagnetoInterface::AddScriptMotion(const YAML::Node& motion_cfg){

}

void MagnetoInterface::AddScriptWalkMotion(const YAML::Node& motion_cfg){

}

void MagnetoInterface::AddScriptClimbMotion(const YAML::Node& motion_cfg) {

    // motion_param->set_walking_pattern(_moving_foot, _pos, _ori, _motion_period, _is_bodyframe);
    MotionCommand motion_command = MotionCommand(_link_idx,_motion_data);
    ((ClimbingInterruptLogic*)interrupt_)
        ->motion_command_script_list_.push_back(motion_command);
}




