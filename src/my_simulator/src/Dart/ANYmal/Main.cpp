#include <../my_utils/Configuration.h>
#include <my_simulator/Dart/ANYmal/ANYmalWorldNode.hpp>
#include <my_utils/IO/IOUtilities.hpp>
#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

void displayJointFrames(const dart::simulation::WorldPtr& world,
                        const dart::dynamics::SkeletonPtr& robot) {
    // -- DISPLAY WHOLE JOINTS
    // for (std::size_t i = 0; i < robot->getNumBodyNodes(); ++i) {
    //     dart::dynamics::BodyNode* bn = robot->getBodyNode(i);
    //     for (std::size_t j = 0; j < bn->getNumChildJoints(); ++j) {
    //         const dart::dynamics::Joint* joint = bn->getChildJoint(j);
    //         const Eigen::Isometry3d offset =
    //             joint->getTransformFromParentBodyNode();

    //         dart::gui::osg::InteractiveFramePtr frame =
    //             std::make_shared<dart::gui::osg::InteractiveFrame>(
    //                 bn, joint->getName() + "/frame", offset);

    //         for (const auto type : {dart::gui::osg::InteractiveTool::ANGULAR,
    //                                 dart::gui::osg::InteractiveTool::PLANAR})
    //             for (std::size_t i = 0; i < 3; ++i)
    //                 frame->getTool(type, i)->setEnabled(false);

    //         world->addSimpleFrame(frame);
    //     }
    // }

    // -- DISPLAY CERTAIN JOINTS
    std::vector<std::string> JointNametoDisplay;
    JointNametoDisplay.push_back("LF_HAA");
    JointNametoDisplay.push_back("LH_HAA");
    JointNametoDisplay.push_back("RF_HAA");
    JointNametoDisplay.push_back("RH_HAA");

    for(int i=0; i<JointNametoDisplay.size(); i++) {
        dart::dynamics::Joint* joint = robot->getJoint(JointNametoDisplay[i]);
        Eigen::Isometry3d offset = joint->getTransformFromParentBodyNode();
        dart::dynamics::BodyNode* bn = joint->getParentBodyNode();

        dart::gui::osg::InteractiveFramePtr frame =
            std::make_shared<dart::gui::osg::InteractiveFrame>(
                bn, joint->getName() + "/frame", offset);

        for (const auto type : {dart::gui::osg::InteractiveTool::ANGULAR,
                                dart::gui::osg::InteractiveTool::PLANAR})
            for (std::size_t i = 0; i < 3; ++i)
                frame->getTool(type, i)->setEnabled(false);

        world->addSimpleFrame(frame);
        
    }
}


void displayLinkFrames(const dart::simulation::WorldPtr& world,
                        const dart::dynamics::SkeletonPtr& robot) {
    // -- DISPLAY WHOLE LINKS
    // for (std::size_t i = 0; i < robot->getNumBodyNodes(); ++i) {
    //     dart::dynamics::BodyNode* bn = robot->getBodyNode(i);
    //         dart::gui::osg::InteractiveFramePtr frame =
    //             std::make_shared<dart::gui::osg::InteractiveFrame>(
    //                 bn, bn->getName() + "/frame");

    //         for (const auto type : {dart::gui::osg::InteractiveTool::ANGULAR,
    //                                 dart::gui::osg::InteractiveTool::PLANAR})
    //             for (std::size_t i = 0; i < 3; ++i)
    //                 frame->getTool(type, i)->setEnabled(false);

    //         world->addSimpleFrame(frame);        
    // }

    // -- DISPLAY CERTAIN LINKS
    std::vector<std::string> LinkNametoDisplay;
    LinkNametoDisplay.clear();
    LinkNametoDisplay.push_back("LF_FOOT");
    LinkNametoDisplay.push_back("LH_FOOT");
    LinkNametoDisplay.push_back("RF_FOOT");
    LinkNametoDisplay.push_back("RH_FOOT");
    LinkNametoDisplay.push_back("base");

    LinkNametoDisplay.push_back("ur3_base");
    LinkNametoDisplay.push_back("ur3_ee_link");
    

    for(int i=0; i<LinkNametoDisplay.size(); i++) {
        dart::dynamics::BodyNode* bn = robot->getBodyNode(LinkNametoDisplay[i]);
        dart::gui::osg::InteractiveFramePtr frame =
            std::make_shared<dart::gui::osg::InteractiveFrame>(
                bn, bn->getName() + "/frame");

        for (size_t i = 0; i < 3; ++i)
            for (size_t j = 0; j < 3; ++j)
            frame->getTool((dart::gui::osg::InteractiveTool::Type)(i), j)
                ->setEnabled(false);

        world->addSimpleFrame(frame);
    }

    // -- DISPLAY WORLD FRAME
    dart::dynamics::SkeletonPtr ground = world->getSkeleton("ground_skeleton");
    // dart::dynamics::BodyNode* bn = ground->getBodyNode("world_frame");
    // dart::gui::osg::InteractiveFramePtr frame =
    // std::make_shared<dart::gui::osg::InteractiveFrame>(bn, bn->getName() + "/frame");

    // for (const auto type : {dart::gui::osg::InteractiveTool::ANGULAR,
    //                         dart::gui::osg::InteractiveTool::PLANAR})
    //     for (std::size_t i = 0; i < 3; ++i)
    //         frame->getTool(type, i)->setEnabled(false);
    // world->addSimpleFrame(frame);

    // -- DISPLAY ground link FRAME
    dart::dynamics::BodyNode* bn_ground = ground->getBodyNode("ground_link");
    dart::gui::osg::InteractiveFramePtr frame_ground =
    std::make_shared<dart::gui::osg::InteractiveFrame>(bn_ground, bn_ground->getName() + "/frame");

    for (const auto type : {dart::gui::osg::InteractiveTool::ANGULAR,
                            dart::gui::osg::InteractiveTool::PLANAR})
        for (std::size_t i = 0; i < 3; ++i)
            frame_ground->getTool(type, i)->setEnabled(false);

    world->addSimpleFrame(frame_ground);
}        

class OneStepProgress : public osgGA::GUIEventHandler {
   public:
    OneStepProgress(ANYmalWorldNode* worldnode) : worldnode_(worldnode) {}

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
    ANYmalWorldNode* worldnode_;
};

void _setJointLimitConstraint(dart::dynamics::SkeletonPtr robot) {
    for (int i = 0; i < robot->getNumJoints(); ++i) {
        dart::dynamics::Joint* joint = robot->getJoint(i);
        joint->setPositionLimitEnforced(true);
    }
}

void _setTransparency(dart::dynamics::SkeletonPtr robot) {
    // for (int i = 0; i < robot->getNumBodyNodes(); ++i) {
    for (int i = 0; i < ANYmalBodyNode::ur3_base; ++i) { // only anymal
        dart::dynamics::BodyNodePtr bn = robot->getBodyNode(i);
        auto sns = bn->getShapeNodesWith<dart::dynamics::VisualAspect>();
        for (auto sn : sns) {
            sn->getShape()->addDataVariance(
                dart::dynamics::Shape::DYNAMIC_COLOR);
            sn->getVisualAspect()->setAlpha(0.1);
        }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
    }
}

void _setInitialConfiguration(dart::dynamics::SkeletonPtr robot, 
                            const Eigen::VectorXd& q_v = Eigen::VectorXd::Zero(6),
                            Eigen::VectorXd q_leg_init = Eigen::VectorXd::Zero(3),
                            Eigen::VectorXd q_arm_init = Eigen::VectorXd::Zero(6)) {       

    Eigen::VectorXd q = robot->getPositions();
    q.segment(0,6) = q_v.head(6);

    if( q_leg_init.norm() < 1e-5 ){
        // q2 +  q3 + pi/2 = 0
        q_leg_init(1) =  1./10.*M_PI_2; // -1./10.*M_PI_2;
        q_leg_init(2) = -11./10.*M_PI_2; // -9./10.*M_PI_2;
    }

    q[ANYmalDoF::LF_HAA] = q_leg_init(0);
    q[ANYmalDoF::LF_HFE] = q_leg_init(1);
    q[ANYmalDoF::LF_KFE] = -q_leg_init(2);

    q[ANYmalDoF::RF_HAA] = -q_leg_init(0);
    q[ANYmalDoF::RF_HFE] = q_leg_init(1);
    q[ANYmalDoF::RF_KFE] = -q_leg_init(2);

    q[ANYmalDoF::LH_HAA] = q_leg_init(0);
    q[ANYmalDoF::LH_HFE] = -q_leg_init(1);
    q[ANYmalDoF::LH_KFE] = q_leg_init(2);

    q[ANYmalDoF::RH_HAA] = -q_leg_init(0);
    q[ANYmalDoF::RH_HFE] = -q_leg_init(1);
    q[ANYmalDoF::RH_KFE] = q_leg_init(2);

    q.tail(6) = q_arm_init;


    robot->setPositions(q);

}

int main(int argc, char** argv) {
    double servo_rate;
    bool isRecord;
    bool b_show_joint_frame;
    bool b_show_link_frame;
    std::string ground_file;
    std::string robot_file;
    Eigen::VectorXd q_floating_base_init = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd q_leg_init = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd q_arm_init = Eigen::VectorXd::Zero(6);
    double q_temp;
    double coef_fric;
    //std::ostringstream ground_file;
    YAML::Node simulation_cfg;
    try {
        simulation_cfg =
            YAML::LoadFile(THIS_COM "config/ANYmal/SIMULATION.yaml");
        my_utils::readParameter(simulation_cfg, "servo_rate", servo_rate);
        my_utils::readParameter(simulation_cfg, "is_record", isRecord);
        my_utils::readParameter(simulation_cfg, "show_joint_frame", b_show_joint_frame);
        my_utils::readParameter(simulation_cfg, "show_link_frame", b_show_link_frame);
        my_utils::readParameter(simulation_cfg, "ground", ground_file);
        my_utils::readParameter(simulation_cfg, "robot", robot_file);

        my_utils::readParameter(simulation_cfg, "initial_pose", q_floating_base_init);   
        my_utils::readParameter(simulation_cfg, "initial_leg_config", q_leg_init);
        my_utils::readParameter(simulation_cfg, "initial_arm_config", q_arm_init);  
               
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }

    // =========================================================================
    // Generate world and add skeletons
    // =========================================================================
    std::cout << ground_file << std::endl;
    std::cout << robot_file << std::endl;

    dart::simulation::WorldPtr world(new dart::simulation::World);
    dart::utils::DartLoader urdfLoader;
    ground_file.insert(0, THIS_COM);
    robot_file.insert(0, THIS_COM);
    dart::dynamics::SkeletonPtr ground 
                        = urdfLoader.parseSkeleton(ground_file);
    dart::dynamics::SkeletonPtr robot 
                        = urdfLoader.parseSkeleton(robot_file);

    world->addSkeleton(ground);
    world->addSkeleton(robot);

    Eigen::Vector3d gravity(0.0, 0.0, -9.81);
    world->setGravity(gravity);
    world->setTimeStep(servo_rate);

    // =========================================================================
    // Display Joints Frame
    // =========================================================================
    if (b_show_joint_frame) displayJointFrames(world, robot);
    if (b_show_link_frame) displayLinkFrames(world, robot);

    // =========================================================================
    // Initial configuration
    // =========================================================================
    
    _setInitialConfiguration(robot, q_floating_base_init, q_leg_init, q_arm_init);
    // TODO
    // =========================================================================
    // Enabel Joit Limits
    // =========================================================================
    // _setJointLimitConstraint(robot);

    // =========================================================================
    // Set Transparency
    // =========================================================================
    // _setTransparency(robot);

    // =========================================================================
    // Wrap a worldnode
    // =========================================================================
    osg::ref_ptr<ANYmalWorldNode> node;
    node = new ANYmalWorldNode(world);
    ((ANYmalWorldNode*)node)->setParameters(simulation_cfg);
    node->setNumStepsPerCycle(30);


    // =========================================================================
    // Create and Set Viewer
    // =========================================================================
    dart::gui::osg::Viewer viewer;
    viewer.addWorldNode(node);
    viewer.simulate(false);
    viewer.switchHeadlights(false);
    ::osg::Vec3 p1(1.0, 0.2, 1.0);
    p1 = p1 * 0.7;
    viewer.getLightSource(0)->getLight()->setPosition(
        ::osg::Vec4(p1[0], p1[1], p1[2], 0.0));
    viewer.getCamera()->setClearColor(osg::Vec4(0.93f, 0.95f, 1.0f, 0.95f));
    viewer.getCamera()->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    viewer.addEventHandler(new OneStepProgress(node));

    if (isRecord) {
        std::cout << "[Video Record Enable]" << std::endl;
        viewer.record(THIS_COM "/ExperimentVideo");
    }

    // small window
    // viewer.setUpViewInWindow(1440, 0, 500, 500);
    // viewer.getCameraManipulator()->setHomePosition(
    //     ::osg::Vec3(5.14, 2.28, 3.0) * 1.5, ::osg::Vec3(0.0, 0.2, 0.5),
    //     ::osg::Vec3(0.0, 0.0, 1.0));

    // large window
    // viewer.setUpViewInWindow(0, 0, 2880, 1800);
    // viewer.getCameraManipulator()->setHomePosition(
    //     ::osg::Vec3(2.2, -2.0, 1.5), ::osg::Vec3(0.0, -0.5, 0.8),
    //     ::osg::Vec3(0.0, 0.0, 1.0));

    // back view
    viewer.setUpViewInWindow(0, 0, 2880, 1800);
    viewer.getCameraManipulator()->setHomePosition(
        ::osg::Vec3(-2.2, 2.0, 1.0), ::osg::Vec3(0.0, -0.5, 0.8),
        ::osg::Vec3(0.0, 0.0, 1.0));
    viewer.setCameraManipulator(viewer.getCameraManipulator());
    viewer.run();
}
