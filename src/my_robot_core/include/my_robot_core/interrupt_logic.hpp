#pragma once

#include <my_utils/IO/IOUtilities.hpp>

class InterruptLogic {
 public:
  InterruptLogic() { resetFlags(); }
  virtual ~InterruptLogic() {}
  virtual void processInterrupts() { resetFlags(); }
  virtual void resetFlags() {    
    // b_interrupt_button_p = false;
    // std::cout << "resetFlags " << std::endl;
    b_button_pressed = false;   
  }
  virtual void setInterruptRoutine(const YAML::Node& motion_cfg) {};
  virtual void setFlags(uint16_t key) {b_button_pressed = true; pressed_button = key;
  // std::cout << "setFlags(" << b_button_pressed <<") = " << pressed_button << std::endl;
  };
// bool b_interrupt_button_p;
 protected:
  bool b_button_pressed;
  uint16_t pressed_button;  
};
