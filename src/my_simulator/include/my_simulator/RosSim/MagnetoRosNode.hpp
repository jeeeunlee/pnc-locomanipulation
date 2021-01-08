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

class SimulatorParameter{
  public: 
    SimulatorParameter(std::string _config_file){
        
        // READ PARAMETERS FROM CONFIGURATION FILE
        _config_file.insert(0, THIS_COM);
        try {
            YAML::Node simulation_cfg = YAML::LoadFile(_config_file);
            my_utils::readParameter(simulation_cfg, "servo_rate", servo_rate_);
            my_utils::readParameter(simulation_cfg, "is_record", is_record_);
            my_utils::readParameter(simulation_cfg, "show_joint_frame", b_show_joint_frame_);
            my_utils::readParameter(simulation_cfg, "show_link_frame", b_show_link_frame_);

            my_utils::readParameter(simulation_cfg, "ground", fp_ground_);
            my_utils::readParameter(simulation_cfg, "robot", fp_robot_);
            my_utils::readParameter(simulation_cfg, "initial_pose", q_virtual_init_); 
            my_utils::readParameter(simulation_cfg, "friction", coeff_fric_);                

        } catch (std::runtime_error& e) {
            std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                    << __FILE__ << "]" << std::endl
                    << std::endl;
        }        

        // SET DEFAULT VALUES && POST PROCESSING
        gravity_ << 0.0, 0.0, -9.81;
        fp_ground_.insert(0, THIS_COM);
        fp_robot_.insert(0, THIS_COM);

        // CHECK THE VALUES
        my_utils::pretty_print(q_virtual_init_, std::cout, "q_virtual_init_");  
        my_utils::pretty_print(gravity_, std::cout, "gravity_");  
    }

  public:   
    double servo_rate_;
    bool is_record_;
    bool b_show_joint_frame_;
    bool b_show_link_frame_;

    // file path
    std::string fp_ground_;
    std::string fp_robot_;

    Eigen::VectorXd q_virtual_init_;
    Eigen::Vector3d gravity_;    
    double coeff_fric_;
};

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
