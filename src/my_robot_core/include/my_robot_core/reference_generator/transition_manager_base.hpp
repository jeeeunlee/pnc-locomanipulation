#pragma once

#include <vector>

#include <my_utils/IO/IOUtilities.hpp>
#include <my_utils/Math/MathUtilities.hpp>

// Object to manage common transition primitives.
// Base class for the transition manager

class TransitionManagerBase {
 public:
  TransitionManagerBase() { 

    trans_start_time_ = 0.;
    trans_end_time_ = 0.;
    trans_duration_ = 0.;
  }
  virtual ~TransitionManagerBase() {}

 protected:
  double trans_start_time_;
  double trans_end_time_;
  double trans_duration_;
};
