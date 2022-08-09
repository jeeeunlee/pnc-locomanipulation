#include <my_robot_core/anymal_core/anymal_control_architecture/mpc_architecture.hpp>
#include <my_robot_core/anymal_core/anymal_control_architecture/wblc_architecture.hpp>
#include <my_robot_core/anymal_core/anymal_control_architecture/manipulation_architecture.hpp>

namespace ANYMAL_STATES {
constexpr int INITIALIZE = 0;
constexpr int BALANCE = 1; // DEFAULT
constexpr int SWING_START_TRANS = 2;
constexpr int SWING = 3;
constexpr int SWING_END_TRANS = 4;
constexpr int MANIPULATION = 5;

};  // namespace ANYMAL_STATES

