#pragma once

#include <my_robot_core/magneto_core/magneto_state_provider.hpp>
#include <my_robot_core/state_estimator.hpp>

class MagnetoWbcSpecContainer;
class MagnetoReferenceGeneratorContainer;

class SlipObserverData {
    public:
    SlipObserverData() {

    }
    ~SlipObserverData() {}

};

class SlipObserver : public StateEstimator {
  public:
    SlipObserver( MagnetoWbcSpecContainer* ws_container,
                RobotSystem* _robot);
    ~SlipObserver();
    void evaluate();
    void initialization(const YAML::Node& node);

    void checkVelocity(int foot_idx);

  protected:
    MagnetoStateProvider* sp_;
    MagnetoWbcSpecContainer* ws_container_;
};