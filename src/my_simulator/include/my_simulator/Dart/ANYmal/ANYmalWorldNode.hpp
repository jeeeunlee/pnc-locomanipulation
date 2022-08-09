#pragma once

#include <Eigen/Dense>
#include <queue> 
#include <dart/dart.hpp>
#include <dart/gui/GLFuncs.hpp>
#include <dart/gui/osg/osg.hpp>

#include <my_robot_core/anymal_core/anymal_definition.hpp>
#include <my_utils/IO/IOUtilities.hpp>

// ANYmalInterface
class EnvInterface;
class ANYmalSensorData;
class ANYmalCommand;

class ANYmalWorldNode : public dart::gui::osg::WorldNode {
   private:
    void UpdateContactDistance_();
    void UpdateContactSwitchData_();
    void UpdateContactWrenchData_();
    
    void PlotResult_();
    void PlotFootStepResult_();
    void PlotForce_(int fidx, const Eigen::Vector3d& frc_foot);
    
    void CheckRobotSkeleton(const dart::dynamics::SkeletonPtr& skel);
    
    void EnforceTorqueLimit(); 

    void saveData();

    EnvInterface* interface_;
    ANYmalSensorData* sensor_data_;
    ANYmalCommand* command_;

    dart::simulation::WorldPtr world_;
    dart::dynamics::SkeletonPtr robot_;
    dart::dynamics::SkeletonPtr ground_;

    Eigen::VectorXd trq_cmd_;

    int count_;
    double t_;
    double servo_rate_;
    int n_dof_;
    double kp_;
    double kd_;
    double torque_limit_;

    Eigen::MatrixXd R_ground_;
    Eigen::MatrixXd p_ground_;    

    float contact_threshold_;
    std::array<double, ANYmal::n_leg> contact_distance_;
    std::array<double, ANYmal::n_leg> coef_fric_;

    Eigen::VectorXd trq_lb_;
    Eigen::VectorXd trq_ub_;

    bool b_plot_result_;

   public:
    ANYmalWorldNode(const dart::simulation::WorldPtr& world);
    virtual ~ANYmalWorldNode();

    void customPreStep() override;
    void customPostStep() override;

    // user button
    void enableButtonFlag(uint16_t key);
    void setParameters(const YAML::Node& simulation_cfg);

    // 
    void setFrictionCoeff();
};