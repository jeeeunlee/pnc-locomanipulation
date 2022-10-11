#pragma once

#include <deque>


// default component
class UserCommand {
  public:
    UserCommand() {};
    virtual ~UserCommand() {}; 
};

// container
template<class T>
class StateSequence{
  protected:
    std::deque<std::pair<int,T>> state_sequence_;

  public:
    StateSequence() { state_sequence_.clear(); }
    void addState(int state_id, const T& state_cmd){
      state_sequence_.push_back( std::make_pair(state_id, state_cmd) );
    }
    bool getNextState(int& state_id, T& state_cmd){
      if(state_sequence_.empty()) return false;
      auto pair = state_sequence_.front();
      state_id = pair.first;
      state_cmd = pair.second;
      state_sequence_.pop_front();
      return true;
    }
    int getNumStates(){
      return state_sequence_.size();
    }
};