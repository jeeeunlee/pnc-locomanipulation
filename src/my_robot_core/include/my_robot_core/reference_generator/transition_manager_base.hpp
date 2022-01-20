#pragma once

#include <vector>

#include <my_utils/IO/IOUtilities.hpp>
#include <my_utils/Math/MathUtilities.hpp>

// Object to manage common transition primitives.
// Base class for the transition manager

class TransitionManagerBase {
 public:
  TransitionManagerBase() { 

    traj_start_time_ = 0.;
    traj_end_time_ = 0.;
    traj_duration_ = 0.;
  }
  virtual ~TransitionManagerBase() {}

 protected:
  double traj_start_time_;
  double traj_end_time_;
  double traj_duration_;
};
