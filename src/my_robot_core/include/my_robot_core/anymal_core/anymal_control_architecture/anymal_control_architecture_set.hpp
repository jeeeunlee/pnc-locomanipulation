#include <my_robot_core/anymal_core/anymal_control_architecture/locomanipulation_architecture.hpp>

namespace ANYMAL_STATES {
constexpr int INITIALIZE = 0;
constexpr int IDLE = 1; // DEFAULT
constexpr int BALANCE = 2; // com motion given
constexpr int MANIPULATION = 3; // arm motion given (+com)
constexpr int LOCOMOTION = 4; // foot motion given (+com)
constexpr int LOCOMANIPULATION = 5; // foot & arm motion given (+com)
};  // namespace ANYMAL_STATES

namespace ANYMAL_FOOT_STATES {
constexpr int INITIALIZE = 0;
constexpr int SUPPORT = 1; // DEFAULT
constexpr int SWING_START_TRANS = 2;
constexpr int SWING = 3;
constexpr int SWING_END_TRANS = 4;
};  // namespace ANYMAL_FOOT_STATES

namespace ANYMAL_COM_STATES {
constexpr int INITIALIZE = 0;
constexpr int IDLE = 1;
 constexpr int BALANCE = 2;
};  // namespace ANYMAL_COM_STATES

namespace ANYMAL_ARM_STATES {
constexpr int INITIALIZE = 0;
constexpr int IDLE = 1;
constexpr int MANIPULATION = 2;
};  // namespace ANYMAL_ARM_STATES