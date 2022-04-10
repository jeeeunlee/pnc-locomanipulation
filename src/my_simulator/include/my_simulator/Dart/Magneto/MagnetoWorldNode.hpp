#pragma once

#include <Eigen/Dense>
#include <queue> 
#include <dart/dart.hpp>
#include <dart/gui/GLFuncs.hpp>
#include <dart/gui/osg/osg.hpp>

#include <my_robot_core/magneto_core/magneto_definition.hpp>
#include <my_utils/IO/IOUtilities.hpp>

// MagnetoInterface
class EnvInterface;
class MagnetoSensorData;
class MagnetoCommand;

class MagnetoWorldNode : public dart::gui::osg::WorldNode {
   private:

    void UpdateContactDistance_();
    void UpdateContactSwitchData_();
    void UpdateContactWrenchData_();
    
    void ReadMotions_(const std::string& _motion_file_name);
    
    void PlotResult_();
    void PlotFootStepResult_();
    void PlotForce_(int fidx, const Eigen::Vector3d& frc_foot);

    
    void CheckRobotSkeleton(const dart::dynamics::SkeletonPtr& skel);
    
    void EnforceTorqueLimit(); 
    void ApplyMagneticForce();
    void updateContactEnvSetup();

    void saveData();


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
    int contact_frame_type_;

    Eigen::MatrixXd R_ground_;
    Eigen::MatrixXd p_ground_;

    std::array<Eigen::Vector3d, Magneto::n_leg> surface_normal_;
    std::array<double, Magneto::n_leg> coef_fric_;
    std::array<double, Magneto::n_leg> magnetic_force_; // 147. #[N] 
    std::array<double, Magneto::n_leg> residual_magnetism_; //  3.0 #[%]

    float contact_threshold_;
    std::array<double, Magneto::n_leg> contact_distance_;

    Eigen::VectorXd trq_lb_;
    Eigen::VectorXd trq_ub_;

    bool b_plot_result_;

   public:
    MagnetoWorldNode(const dart::simulation::WorldPtr& world);
    virtual ~MagnetoWorldNode();

    void customPreStep() override;
    void customPostStep() override;

    // user button
    void enableButtonFlag(uint16_t key);
    void setParameters(const YAML::Node& simulation_cfg);

    //
    void setFrictionCoeff();
    void setMagneticParameter();
};
