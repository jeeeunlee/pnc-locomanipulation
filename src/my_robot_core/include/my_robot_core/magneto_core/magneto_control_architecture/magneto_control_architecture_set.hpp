#include <my_robot_core/magneto_core/magneto_control_architecture/mpc_architecture.hpp>
#include <my_robot_core/magneto_core/magneto_control_architecture/wbmc_architecture.hpp>

class SimMotionCommand;
class MagnetoUserStateCommand {
   public:
   MagnetoUserStateCommand(int _state_id, const SimMotionCommand& _state_cmd) {
       setCommand(_state_id, _state_cmd)
   }
   void setCommand(int _state_id, const SimMotionCommand& _state_cmd) {
       state_id = _state_id;
       user_cmd = _state_cmd;
   }
   int state_id;
   SimMotionCommand user_cmd;
};