#pragma once

#include <vector>

#include <my_robot_system/RobotSystem.hpp>
#include <my_utils/IO/IOUtilities.hpp>

#include <my_wbc/Contact/ContactSpec.hpp>
#include <my_wbc/Task/Task.hpp>

// Simple class to hold on to task list and contact list

class TaskAndForceContainer {
 public:
  TaskAndForceContainer(RobotSystem* _robot) { robot_ = _robot; }
  virtual ~TaskAndForceContainer() {}
  virtual void paramInitialization(const YAML::Node& node) = 0;

  RobotSystem* robot_;
  std::vector<Task*> task_list_;
  std::vector<ContactSpec*> contact_list_;
};
