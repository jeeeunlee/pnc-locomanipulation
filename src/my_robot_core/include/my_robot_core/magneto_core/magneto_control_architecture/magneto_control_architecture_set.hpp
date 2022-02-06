#include <my_robot_core/magneto_core/magneto_control_architecture/mpc_architecture.hpp>
#include <my_robot_core/magneto_core/magneto_control_architecture/wbmc_architecture.hpp>



namespace CONTROLLER_TYPES {
constexpr int WBMC = 0;
constexpr int WBRMC = 1;
}; // namespace CONTROLLER_TYPES

namespace MAGNETO_STATES {
constexpr int INITIALIZE = 0;
constexpr int BALANCE = 1; // DEFAULT
constexpr int SWING_START_TRANS = 2;
constexpr int SWING = 3;
constexpr int SWING_END_TRANS = 4;
};  // namespace MAGNETO_STATES

class SimMotionCommand;
class MagnetoUserStateCommand {
   public:
   MagnetoUserStateCommand(){
       state_id = -1;
       user_cmd = SimMotionCommand();
   }
   void setCommand(int _state_id, const SimMotionCommand& _state_cmd) {
       std::cout<<"setCommand"<<std::endl;
       state_id = _state_id;
       std::cout<<"_state_id"<<std::endl;
       user_cmd = _state_cmd;
       std::cout<<"_state_cmd"<<std::endl;
   }
   int state_id;
   SimMotionCommand user_cmd;
};