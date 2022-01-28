#pragma once

#include <deque>


// default component
class UserCommand {
  public:
    virtual ~UserCommand() {}; 
    virtual void CopyCommand(UserCommand* _cmd)=0;
};

// container
class UserCommandSequence {
  private:
    std::deque< UserCommand* > sequence_;
  public:
    UserCommandSequence() { sequence_.clear(); }
    ~UserCommandSequence() { for(auto &cmd: sequence_) delete cmd; delete sequence_;}
    void addCommand(UserCommand* _cmd) {
      sequence_.push_back(_cmd);
    }
    bool getNextCommand(UserCommand* _cmd) {
        if(sequence_.empty())
            return false;
        _cmd->CopyCommand(sequence_.front());
        delete (sequence_.front());
        sequence_.pop_front();
        return true;
    };
}

    
// for state machine
class StateCommand{
  public:
    StateCommand(int id, UserCommand* _cmd){
      state_id = id;
      user_cmd = _cmd;
    }
    ~StateCommand() {delete user_cmd;}
    getStateCommand(int& _state_id, UserCommand* _cmd){
      _state_id = state_id;
      _cmd->CopyCommand(user_cmd);
    }
  public:
    int state_id;
    UserCommand* user_cmd;
}
    
class StateSequence {
  private:
    std::deque< StateCommand* > sequence_;
  public:
    StateSequence() { sequence_.clear(); }
    ~StateSequence() { for(auto &cmd: sequence_) delete cmd; delete sequence_;}
    void addState(int state_id, UserCommand* _cmd) {
      sequence_.push_back( new StateCommand(state_id, _cmd) );
    }
    bool getNextState(int& state_id, UserCommand* _cmd) {
        if(sequence_.empty())
            return false;
        StateCommand* state_cmd = sequence_.front();
        state_cmd->getStateCommand(state_id, _cmd);
        delete (state_cmd);
        sequence_.pop_front();
        return true;
    };
}
