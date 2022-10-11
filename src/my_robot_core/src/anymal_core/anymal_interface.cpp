#include <math.h>
#include <stdio.h>
#include <my_robot_system/RobotSystem.hpp>
#include <my_robot_core/anymal_core/anymal_interface.hpp>
#include <my_robot_core/anymal_core/anymal_state_estimator.hpp>
#include <my_robot_core/anymal_core/anymal_state_provider.hpp>
#include <my_robot_core/anymal_core/anymal_command_api.hpp>

#include <my_robot_core/anymal_core/anymal_control_architecture/anymal_control_architecture_set.hpp>
#include <my_robot_core/anymal_core/anymal_logic_interrupt/anymal_interrupt_logic_set.hpp>

#include <my_utils/IO/IOUtilities.hpp>
#include <my_utils/Math/MathUtilities.hpp>
#include <string>

ANYmalInterface::ANYmalInterface() : EnvInterface() {
    // ANYmalInterface 
    std::string border = "=";
    for (int i = 0; i < 79; ++i) {
        border += "=";
    }
    my_utils::color_print(myColor::BoldCyan, border);
    my_utils::pretty_constructor(0, "ANYmal Interface");
    YAML::Node cfg = YAML::LoadFile(THIS_COM "config/ANYmal/INTERFACE.yaml");

    // declare
    robot_ = new RobotSystem(6, THIS_COM 
            "robot_description/Robot/ANYmal/anymal_ur3.urdf");//ANYmalSim_Dart
    robot_->setActuatedJoint(ANYmal::idx_adof);
    // robot_->setRobotMass();
    // robot_->printRobotInfo();    

    state_estimator_ = new ANYmalStateEstimator(robot_);
    sp_ = ANYmalStateProvider::getStateProvider(robot_);

    // control_architecture_ = new ANYmalWblcControlArchitecture(robot_);
    // control_architecture_ = new ANYmalMpcControlArchitecture(robot_);     
    control_architecture_ = new ANYmalLocoManipulationControlArchitecture(robot_);     

    // interrupt_ = new WalkingInterruptLogic(control_architecture_);  
    interrupt_ = new LocoManipulationInterruptLogic(control_architecture_);   

    // read from INTERFACE.yaml
    _ParameterSetting(cfg);

    count_ = 0;
    waiting_count_ = 2;
    cmd_jpos_ = Eigen::VectorXd::Zero(ANYmal::n_adof);
    cmd_jvel_ = Eigen::VectorXd::Zero(ANYmal::n_adof);
    cmd_jtrq_ = Eigen::VectorXd::Zero(ANYmal::n_adof);

    check_com_planner_updated = 0;
    check_foot_planner_updated = 0;    

    my_utils::color_print(myColor::BoldCyan, border);
}

ANYmalInterface::~ANYmalInterface() {
    delete robot_;
    delete state_estimator_;
    delete control_architecture_;
    delete interrupt_;
}

void ANYmalInterface::_ParameterSetting(const YAML::Node& cfg) {      
    try {
        std::string motion_script;        
        my_utils::readParameter(cfg, "motion_script", motion_script);
        std::ostringstream motion_file_name;    
        motion_file_name << THIS_COM << motion_script;
        YAML::Node motion_cfg = YAML::LoadFile(motion_file_name.str());
        int num_motion;
        my_utils::readParameter(motion_cfg, "num_motion", num_motion);
        
        for(int i(0); i<num_motion; ++i){
            std::ostringstream stringStream;
            stringStream << "motion" << i;
            std::string conf = stringStream.str();
            interrupt_->setInterruptRoutine(motion_cfg[conf]);           
        }
        
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}


void ANYmalInterface::getCommand(void* _data, void* _command) {
    ANYmalCommand* cmd = ((ANYmalCommand*)_command);
    ANYmalSensorData* data = ((ANYmalSensorData*)_data);

    if(!_Initialization(data, cmd)) {
        std::cout<<"getCommand : Iniitilized"<<std::endl;
        state_estimator_->Update(data); // robot skelPtr in robotSystem updated 
        interrupt_->processInterrupts();
        control_architecture_->getCommand(cmd);
        
        
        if(!_CheckCommand(cmd)) { _SetStopCommand(data,cmd); }    
    }   

    // save data
    _SaveDataCmd(data,cmd);

    count_++;
    running_time_ = (double)(data->elapsedtime);
    sp_->curr_time = running_time_;
}

bool ANYmalInterface::_Initialization(ANYmalSensorData* data,
                                        ANYmalCommand* _command) {
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

bool ANYmalInterface::_CheckCommand(ANYmalCommand* cmd){
    // return false, if it fails to update test command
    // e.g.) check limit cmd->q, cmd->qdot, cmd-<jtrq 
    return true;
}

void ANYmalInterface::_SetStopCommand(ANYmalSensorData* data, ANYmalCommand* cmd) {
  for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
    cmd->jtrq[i] = 0.;
    cmd->q[i] = data->q[i];
    cmd->qdot[i] = 0.;
  }
}

void ANYmalInterface::_SaveDataCmd(ANYmalSensorData* data, ANYmalCommand* cmd)  {
    // cmd->jtrq = my_utils::CropVector(cmd->jtrq,
    // robot_->GetTorqueLowerLimits(), robot_->GetTorqueUpperLimits(), "clip trq in interface");
    cmd_jtrq_ = cmd->jtrq;
    cmd_jvel_ = cmd->qdot;
    cmd_jpos_ = cmd->q;
}

bool ANYmalInterface::IsPlannerUpdated() {
    if (check_com_planner_updated == sp_->check_com_planner_updated) {
        return false;
    } else {
        check_com_planner_updated = sp_->check_com_planner_updated;
        return true;
    }   
}

bool ANYmalInterface::IsFootPlannerUpdated() {
    if (check_foot_planner_updated == sp_->check_foot_planner_updated) {
        return false;
    } else {
        check_foot_planner_updated = sp_->check_foot_planner_updated;
        return true;
    }   
}

void ANYmalInterface::GetFeasibleCoM(
    std::vector <std::pair<double, Eigen::Vector3d>>& feasible_com_list) {
    feasible_com_list = sp_->feasible_com_list;
}

void ANYmalInterface::GetCurrentCoM(Eigen::VectorXd& com_des) {
    com_des = sp_->com_pos_init;
}

void ANYmalInterface::GetOptimalCoM(Eigen::VectorXd& com_des) {
    com_des = sp_->com_pos_target;
}

void ANYmalInterface::GetCurrentFootStep(Eigen::VectorXd& foot_pos) {
    foot_pos = sp_->foot_pos_init;
}

void ANYmalInterface::GetNextFootStep(Eigen::VectorXd& foot_pos) {
    foot_pos = sp_->foot_pos_target;
}

void ANYmalInterface::GetCoMPlans(Eigen::VectorXd& com_pos_ini,
                                    Eigen::VectorXd& com_pos_goal) {
    com_pos_ini = sp_->com_pos_init;
    com_pos_goal = sp_->com_pos_target;
}






