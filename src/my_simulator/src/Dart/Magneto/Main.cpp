#include <../my_utils/Configuration.h>
#include <my_simulator/Dart/Magneto/MagnetoWorldNode.hpp>
#include <my_utils/IO/IOUtilities.hpp>
#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

void displayJointFrames(const dart::simulation::WorldPtr& world,
                        const dart::dynamics::SkeletonPtr& robot) {
    for (std::size_t i = 0; i < robot->getNumBodyNodes(); ++i) {
        dart::dynamics::BodyNode* bn = robot->getBodyNode(i);
        for (std::size_t j = 0; j < bn->getNumChildJoints(); ++j) {
            const dart::dynamics::Joint* joint = bn->getChildJoint(j);
            const Eigen::Isometry3d offset =
                joint->getTransformFromParentBodyNode();

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
    LinkNametoDisplay.push_back("AL_foot_link");
    LinkNametoDisplay.push_back("AR_foot_link");
    LinkNametoDisplay.push_back("BL_foot_link");
    LinkNametoDisplay.push_back("BR_foot_link");
    // LinkNametoDisplay.push_back("base_link");

    for(int i=0; i<LinkNametoDisplay.size(); i++) {
        dart::dynamics::BodyNode* bn = robot->getBodyNode(LinkNametoDisplay[i]);
        dart::gui::osg::InteractiveFramePtr frame =
            std::make_shared<dart::gui::osg::InteractiveFrame>(
                bn, bn->getName() + "/frame");

        // for (const auto type : {dart::gui::osg::InteractiveTool::ANGULAR,
        //                         dart::gui::osg::InteractiveTool::PLANAR})
        //     for (std::size_t i = 0; i < 3; ++i)
        //         frame->getTool(type, i)->setEnabled(false);
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
    OneStepProgress(MagnetoWorldNode* worldnode) : worldnode_(worldnode) {}

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
    MagnetoWorldNode* worldnode_;
};

void _setJointLimitConstraint(dart::dynamics::SkeletonPtr robot) {
    for (int i = 0; i < robot->getNumJoints(); ++i) {
        dart::dynamics::Joint* joint = robot->getJoint(i);
        joint->setPositionLimitEnforced(true);
    }
}

void _setTransparency(dart::dynamics::SkeletonPtr robot) {
    for (int i = 0; i < robot->getNumBodyNodes(); ++i) {
        dart::dynamics::BodyNodePtr bn = robot->getBodyNode(i);
        auto sns = bn->getShapeNodesWith<dart::dynamics::VisualAspect>();
        for (auto sn : sns) {
            sn->getShape()->addDataVariance(
                dart::dynamics::Shape::DYNAMIC_COLOR);
            sn->getVisualAspect()->setAlpha(0.4);
        }
    }
}

void _setInitialConfiguration(dart::dynamics::SkeletonPtr robot, 
                            const Eigen::VectorXd& q_v = Eigen::VectorXd::Zero(6),
                            Eigen::VectorXd q_leg_init = Eigen::VectorXd::Zero(6)) {

    int AL_coxa_joint = robot->getDof("AL_coxa_joint")->getIndexInSkeleton();
    int AL_femur_joint = robot->getDof("AL_femur_joint")->getIndexInSkeleton();
    int AL_tibia_joint = robot->getDof("AL_tibia_joint")->getIndexInSkeleton();
    int AL_foot_joint_1 = robot->getDof("AL_foot_joint_1")->getIndexInSkeleton();
    int AL_foot_joint_2 = robot->getDof("AL_foot_joint_2")->getIndexInSkeleton();
    int AL_foot_joint_3 = robot->getDof("AL_foot_joint_3")->getIndexInSkeleton();

    int AR_coxa_joint = robot->getDof("AR_coxa_joint")->getIndexInSkeleton();
    int AR_femur_joint = robot->getDof("AR_femur_joint")->getIndexInSkeleton();
    int AR_tibia_joint = robot->getDof("AR_tibia_joint")->getIndexInSkeleton();
    int AR_foot_joint_1 = robot->getDof("AR_foot_joint_1")->getIndexInSkeleton();
    int AR_foot_joint_2 = robot->getDof("AR_foot_joint_2")->getIndexInSkeleton();
    int AR_foot_joint_3 = robot->getDof("AR_foot_joint_3")->getIndexInSkeleton();

    int BL_coxa_joint = robot->getDof("BL_coxa_joint")->getIndexInSkeleton();
    int BL_femur_joint = robot->getDof("BL_femur_joint")->getIndexInSkeleton();
    int BL_tibia_joint = robot->getDof("BL_tibia_joint")->getIndexInSkeleton();
    int BL_foot_joint_1 = robot->getDof("BL_foot_joint_1")->getIndexInSkeleton();
    int BL_foot_joint_2 = robot->getDof("BL_foot_joint_2")->getIndexInSkeleton();
    int BL_foot_joint_3 = robot->getDof("BL_foot_joint_3")->getIndexInSkeleton();

    int BR_coxa_joint = robot->getDof("BR_coxa_joint")->getIndexInSkeleton();
    int BR_femur_joint = robot->getDof("BR_femur_joint")->getIndexInSkeleton();
    int BR_tibia_joint = robot->getDof("BR_tibia_joint")->getIndexInSkeleton();
    int BR_foot_joint_1 = robot->getDof("BR_foot_joint_1")->getIndexInSkeleton();
    int BR_foot_joint_2 = robot->getDof("BR_foot_joint_2")->getIndexInSkeleton();
    int BR_foot_joint_3 = robot->getDof("BR_foot_joint_3")->getIndexInSkeleton();
    

    Eigen::VectorXd q = robot->getPositions();
    q.segment(0,6) = q_v.head(6);

    if( q_leg_init.norm() < 1e-5 ){
        // q2 +  q3 + pi/2 = 0
        q_leg_init(1) =  1./10.*M_PI_2; // -1./10.*M_PI_2;
        q_leg_init(2) = -11./10.*M_PI_2; // -9./10.*M_PI_2;
    }

    q[AL_coxa_joint] = -q_leg_init(0);
    q[AL_femur_joint] = q_leg_init(1);
    q[AL_tibia_joint] = q_leg_init(2);
    q[AL_foot_joint_1] = -q_leg_init(3);
    q[AL_foot_joint_2] = -q_leg_init(4);
    q[AL_foot_joint_3] = q_leg_init(5);

    q[AR_coxa_joint] = q_leg_init(0);
    q[AR_femur_joint] = q_leg_init(1);
    q[AR_tibia_joint] = q_leg_init(2);
    q[AR_foot_joint_1] = q_leg_init(3);
    q[AR_foot_joint_2] = q_leg_init(4);
    q[AR_foot_joint_3] = q_leg_init(5);

    q[BL_coxa_joint] = q_leg_init(0);
    q[BL_femur_joint] = q_leg_init(1);
    q[BL_tibia_joint] = q_leg_init(2);
    q[BL_foot_joint_1] = q_leg_init(3);
    q[BL_foot_joint_2] = q_leg_init(4);
    q[BL_foot_joint_3] = q_leg_init(5);

    q[BR_coxa_joint] = -q_leg_init(0);
    q[BR_femur_joint] = q_leg_init(1);
    q[BR_tibia_joint] = q_leg_init(2);
    q[BR_foot_joint_1] = -q_leg_init(3);
    q[BR_foot_joint_2] = -q_leg_init(4);
    q[BR_foot_joint_3] = q_leg_init(5);

    robot->setPositions(q);

    // TODO JE: set passive joint???
    // dart::dynamics::Joint* jointsss = robot->getJoint("AL_foot_joint_1");
    // switch(jointsss->getActuatorType()){
    //  case dart::dynamics::Joint::ActuatorType::FORCE : 
    //     printf("force \n");
    //     break;
    //  case dart::dynamics::Joint::ActuatorType::PASSIVE : 
    //     printf("passive \n");
    //     break;
    //  default: 
    //     printf("nothing \n");
    //     break;
    // }

}

int main(int argc, char** argv) {
    double servo_rate;
    bool isRecord;
    bool b_show_joint_frame;
    bool b_show_link_frame;
    std::string ground_file;
    std::string robot_file;
    Eigen::VectorXd q_floating_base_init = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd q_leg_init = Eigen::VectorXd::Zero(6);
    double q_temp;
    double coef_fric;
    //std::ostringstream ground_file;
    YAML::Node simulation_cfg;
    try {
        simulation_cfg =
            YAML::LoadFile(THIS_COM "config/Magneto/SIMULATION.yaml");
        my_utils::readParameter(simulation_cfg, "servo_rate", servo_rate);
        my_utils::readParameter(simulation_cfg, "is_record", isRecord);
        my_utils::readParameter(simulation_cfg, "show_joint_frame", b_show_joint_frame);
        my_utils::readParameter(simulation_cfg, "show_link_frame", b_show_link_frame);
        my_utils::readParameter(simulation_cfg, "ground", ground_file);
        my_utils::readParameter(simulation_cfg, "robot", robot_file);

        my_utils::readParameter(simulation_cfg, "initial_pose", q_floating_base_init);   
        my_utils::readParameter(simulation_cfg, "initial_leg_config", q_leg_init);          
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }

    my_utils::pretty_print(q_floating_base_init, std::cout, "q_floating_base_init");    

    // =========================================================================
    // Generate world and add skeletons
    // =========================================================================
    dart::simulation::WorldPtr world(new dart::simulation::World);
    dart::utils::DartLoader urdfLoader;
    ground_file.insert(0, THIS_COM);
    robot_file.insert(0, THIS_COM);
    std::cout << ground_file << std::endl;
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
    
    _setInitialConfiguration(robot, q_floating_base_init, q_leg_init);
    // TODO
    // =========================================================================
    // Enabel Joit Limits
    // =========================================================================
    _setJointLimitConstraint(robot);

    // =========================================================================
    // Set Transparency
    // =========================================================================
    // _setTransparencye(robot);

    // =========================================================================
    // Wrap a worldnode
    // =========================================================================
    osg::ref_ptr<MagnetoWorldNode> node;
    node = new MagnetoWorldNode(world);
    ((MagnetoWorldNode*)node)->setParameters(simulation_cfg);
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

    // viewer.setUpViewInWindow(1440, 0, 500, 500);
    // viewer.getCameraManipulator()->setHomePosition(
    //     ::osg::Vec3(5.14, 2.28, 3.0) * 1.5, ::osg::Vec3(0.0, 0.2, 0.5),
    //     ::osg::Vec3(0.0, 0.0, 1.0));

    viewer.setUpViewInWindow(0, 0, 2880, 1800);
    viewer.getCameraManipulator()->setHomePosition(
        ::osg::Vec3(2.2, -2.0, 1.5), ::osg::Vec3(0.0, -0.5, 0.8),
        ::osg::Vec3(0.0, 0.0, 1.0));
    viewer.setCameraManipulator(viewer.getCameraManipulator());
    viewer.run();
}
