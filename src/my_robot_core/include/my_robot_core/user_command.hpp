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
    std::deque<UserCommand*> sequence_;
  public:
    void addCommand(UserCommand* _cmd);
    bool getNextCommand(UserCommand* _cmd) {
        if(sequence_.empty())
            return false;
        _cmd->CopyCommand(sequence_.front());
        delete (sequence_.front());
        sequence_.pop_front();
        return true;
    };
}
