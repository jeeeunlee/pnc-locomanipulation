#include <../my_utils/Configuration.h>
#include <my_simulator/RosSim/MagnetoRosNode.hpp>
#include <my_simulator/RosSim/SimulatorFunc.hpp>

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>
#include "ros/ros.h"


class OneStepProgress : public osgGA::GUIEventHandler {
   public:
    OneStepProgress(MagnetoRosNode* worldnode) : worldnode_(worldnode) {}

    /** Deprecated, Handle events, return true if handled, false otherwise. */
    virtual bool handle(const osgGA::GUIEventAdapter& ea,
                        osgGA::GUIActionAdapter& /*aa*/) {

        if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP) {            
            if ( ea.getKey() == 'f') {
                int numStepProgress(50);                
                for (int i = 0; i < numStepProgress; ++i) {
                    worldnode_->customPreStep();
                    worldnode_->getWorld()->step();
                    worldnode_->customPostStep();
                }
                return true;
            } else {
                uint16_t button_pressed = ea.getKey();
                std::cout << "button(" << (char)button_pressed << ")  pressed handled @ Main.cpp" << std::endl;
                worldnode_->enableButtonFlag(button_pressed);
            }                     
        }
        return false;
    }
    MagnetoRosNode* worldnode_;
};

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

void setWorld(dart::simulation::WorldPtr& world, const SimulatorParameter& sim_param) {

    // add robot & ground
    dart::utils::DartLoader urdfLoader;
    std::cout << "ground file path = " << sim_param.fp_ground_ << std::endl;
    std::cout << "robot file path = " << sim_param.fp_robot_ << std::endl;

    dart::dynamics::SkeletonPtr ground = 
                    urdfLoader.parseSkeleton(sim_param.fp_ground_);
    dart::dynamics::SkeletonPtr robot = 
                    urdfLoader.parseSkeleton(sim_param.fp_robot_);

    world->addSkeleton(ground);
    world->addSkeleton(robot);

    // Friction & Restitution Coefficient
    double friction(sim_param.coeff_fric_); // maximum tangential force not mu
    ground->getBodyNode("ground_link")->setFrictionCoeff(friction);
    robot->getBodyNode("BL_foot_link_3")->setFrictionCoeff(friction);
    robot->getBodyNode("AL_foot_link_3")->setFrictionCoeff(friction);
    robot->getBodyNode("AR_foot_link_3")->setFrictionCoeff(friction);
    robot->getBodyNode("BR_foot_link_3")->setFrictionCoeff(friction);

    world->setGravity(sim_param.gravity_);
    world->setTimeStep(sim_param.servo_rate_);

    // Display Joints / Links Frame
    std::vector<std::string> link_to_display = {"AL_foot_link", "AR_foot_link"};
    if (sim_param.b_show_joint_frame_) displayJointFrames(world, robot);
    if (sim_param.b_show_link_frame_) displayLinkFrames(world, robot, link_to_display);
    if (sim_param.b_show_link_frame_) displayGroundFrames(world, ground);

    // Initial configuration
    Eigen::VectorXd q = robot->getPositions();
    q.segment(0,6) = sim_param.q_virtual_init_.head(6);

    std::string coxa("coxa_joint");
    std::string femur("femur_joint");
    std::string tibia("tibia_joint");
    std::string foot1("foot_joint_1");
    std::string foot2("foot_joint_2");
    std::string foot3("foot_joint_3"); 

    const std::array<std::string, 4> foot_name = {"AL_", "BL_", "AR_", "BR_"};

    double femur_joint_init = 1./10.*M_PI_2; // -1./10.*M_PI_2;
    double tibia_joint_init = -11./10.*M_PI_2; // -9./10.*M_PI_2;

    for(int i=0; i<foot_name.size(); i++) {
        q[robot->getDof(foot_name[i] + coxa)->getIndexInSkeleton()] = 0.0;
        q[robot->getDof(foot_name[i] + femur)->getIndexInSkeleton()] = femur_joint_init;
        q[robot->getDof(foot_name[i] + tibia)->getIndexInSkeleton()] = tibia_joint_init;
        q[robot->getDof(foot_name[i] + foot1)->getIndexInSkeleton()] = 0.0;
        q[robot->getDof(foot_name[i] + foot2)->getIndexInSkeleton()] = 0.0;
        q[robot->getDof(foot_name[i] + foot3)->getIndexInSkeleton()] = 0.0;
    }
    robot->setPositions(q);

    // Enabel Joit Limits
    setJointLimitConstraint(robot);

    // Set Transparency
    // setTransparency(robot);

    // Print Model Info
    // printRobotModel(robot);
}




int main(int argc, char** argv) {

    ros::init(argc, argv, "magneto_simulator");
    ros::NodeHandle nh;
    
    std::string sim_config_filename; 
    if(argc < 2) {
        sim_config_filename = "config/Magneto/SIMULATIONWALK.yaml";
        ROS_WARN("simulation config file default: %s", sim_config_filename.c_str());
    } else {
        sim_config_filename = std::string(argv[1]);
        ROS_INFO("simulation config file loaded: %s", sim_config_filename.c_str());
    }
    SimulatorParameter sim_param(sim_config_filename);

    
    // Generate world and add skeletons
    dart::simulation::WorldPtr world(new dart::simulation::World);
    setWorld(world, sim_param);

    // =========================================================================
    // Create and Set Viewer
    // =========================================================================

    // Wrap a worldnode
    osg::ref_ptr<MagnetoRosNode> node = new MagnetoRosNode(nh, world);
    node->setNumStepsPerCycle(10);

    // Create and Set Viewer
    dart::gui::osg::Viewer viewer;
    viewer.addWorldNode(node);
    viewer.simulate(false);
    viewer.switchHeadlights(false);
    viewer.getLightSource(0)->getLight()->setPosition(
        ::osg::Vec4(0.7, 0.14, 0.7, 0.0));
    viewer.getCamera()->setClearColor(osg::Vec4(0.93f, 0.95f, 1.0f, 0.95f));
    viewer.getCamera()->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    viewer.addEventHandler(new OneStepProgress(node));

    if (sim_param.is_record_) {
        std::cout << "[Video Record Enable]" << std::endl;
        viewer.record(THIS_COM "/ExperimentVideo");
    }

    // viewer.setUpViewInWindow(0, 0, 2880, 1800);
    viewer.setUpViewInWindow(1440, 0, 500, 500);
    viewer.getCameraManipulator()->setHomePosition(
                            ::osg::Vec3(5.14, 2.28, 3.0) * 1.5, 
                            ::osg::Vec3(0.0, 0.2, 0.5),
                            ::osg::Vec3(0.0, 0.0, 1.0));
    viewer.setCameraManipulator(viewer.getCameraManipulator());
    viewer.run();

    ros::spin();
    return 0;
}