#pragma once

#include <my_robot_core/interrupt_logic.hpp>
#include <my_robot_core/control_architecture.hpp>
#include <my_robot_core/anymal_core/anymal_command_api.hpp>
#include <my_robot_core/anymal_core/anymal_definition.hpp>


// Forward Declare 
class ANYmalStateProvider;

namespace EE_IDX{
constexpr int armEE = -2;  
constexpr int CoM = -1; 
constexpr int LF = 0; 
constexpr int LH = 1; 
constexpr int RF = 2; 
constexpr int RH = 3; 
}; // namespace EE_IDX

class LocoManipulationInterruptLogic : public InterruptLogic {  
 public:
  LocoManipulationInterruptLogic(ControlArchitecture* ctrl_arch_);
  ~LocoManipulationInterruptLogic();

  void processInterrupts();
  void setInterruptRoutine(const YAML::Node& motion_cfg);

  ControlArchitecture* ctrl_arch_;
  ANYmalStateProvider* sp_;

  std::map<int, std::vector<MotionCommand>> script_user_cmd_map_ = {
    {EE_IDX::armEE, {} }, {EE_IDX::CoM, {} }, {EE_IDX::LF, {} },
    {EE_IDX::LH, {} }, {EE_IDX::RF, {} }, {EE_IDX::RH, {} }
  };

  // state id, mc
  std::vector<std::pair<int, MotionCommand>> script_user_cmd_arm_; 
  std::vector<std::pair<int, MotionCommand>> script_user_cmd_CoM_;
  std::array< std::vector<std::pair<int, MotionCommand>>, 
              ANYmal::n_leg> script_user_cmd_feet_;
  // std::vector<std::pair<int, MotionCommand>> script_user_cmd_LF_;
  // std::vector<std::pair<int, MotionCommand>> script_user_cmd_LH_;
  // std::vector<std::pair<int, MotionCommand>> script_user_cmd_RF_;
  // std::vector<std::pair<int, MotionCommand>> script_user_cmd_RH_;

private:
  void addStateCommand(int _state_id, const MotionCommand& _mc, int state_type);
  void scriptMotionToCommandSet();
  void scriptMotionToEECommandSet(int ee_idx, double script_time_max,
        std::vector<std::pair<int, MotionCommand>> &_script_cmds);
  void scriptMotionToFeetCommandSet(int ee_idx, double script_time_max,
        std::vector<std::pair<int, MotionCommand>> &_script_cmds);

};
