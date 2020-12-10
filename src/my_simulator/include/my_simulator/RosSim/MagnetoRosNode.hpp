#pragma once

#include <Eigen/Dense>
#include <queue> 
#include <dart/dart.hpp>
#include <dart/gui/GLFuncs.hpp>
#include <dart/gui/osg/osg.hpp>
#include <../my_utils/Configuration.h>
#include <my_pnc/MagnetoPnC/MagnetoInterface.hpp>
#include <my_utils/IO/IOUtilities.hpp>
#include <my_utils/Math/MathUtilities.hpp>
#include <ros/ros.h>


// MagnetoInterface
class EnvInterface;
class MagnetoSensorData;
class MagnetoCommand;

class MagnetoRosNode : public dart::gui::osg::WorldNode {

  public:
    MagnetoRosNode(ros::NodeHandle& nh, const dart::simulation::WorldPtr& _world);
    virtual ~MagnetoRosNode();

    void customPreStep() override;
    void customPostStep() override;

    // user button
    void enableButtonFlag(uint16_t key);

   private:

    void UpdateContactDistance_();
    void UpdateContactSwitchData_();
    void UpdateContactWrenchData_();

    void SetParams_();
    void ReadMotions_();
    void PlotResult_();
    void PlotFootStepResult_();
    void CheckInterrupt_();
    
    void CheckRobotSkeleton(const dart::dynamics::SkeletonPtr& skel);
    
    void EnforceTorqueLimit(); 
    void ApplyMagneticForce();

    ros::NodeHandle nh_;
    
    EnvInterface* interface_;
    MagnetoSensorData* sensor_data_;
    MagnetoCommand* command_;

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

    int run_mode_;

    double magnetic_force_; // 147. #[N] 
    double residual_magnetism_; //  3.0 #[%]


    float contact_threshold_;
    std::map<int, double> contact_distance_;

    Eigen::VectorXd trq_lb_;
    Eigen::VectorXd trq_ub_;

    bool b_plot_result_;

    std::string motion_file_name_;




};
