#include <my_simulator/RosSim/SimulatorFunc.hpp>

void displayJointFrames(dart::simulation::WorldPtr world,
                        const dart::dynamics::SkeletonPtr robot) {
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

void displayLinkFrames(dart::simulation::WorldPtr world,
                        const dart::dynamics::SkeletonPtr robot,
                        const std::vector<std::string>& link_list) {
    for(auto &bn_name : link_list) {
        dart::dynamics::BodyNode* bn = robot->getBodyNode(bn_name);
        dart::gui::osg::InteractiveFramePtr frame =
            std::make_shared<dart::gui::osg::InteractiveFrame>(
                bn, bn->getName() + "/frame");

        for (const auto type : {dart::gui::osg::InteractiveTool::ANGULAR,
                                dart::gui::osg::InteractiveTool::PLANAR})
            for (std::size_t i = 0; i < 3; ++i)
                frame->getTool(type, i)->setEnabled(false);
        world->addSimpleFrame(frame);
    }

}

void displayLinkFrames(dart::simulation::WorldPtr world,
                        const dart::dynamics::SkeletonPtr robot) {
    // -- DISPLAY WHOLE LINKS
    for (std::size_t i = 0; i < robot->getNumBodyNodes(); ++i) {
        dart::dynamics::BodyNode* bn = robot->getBodyNode(i);
        dart::gui::osg::InteractiveFramePtr frame =
                std::make_shared<dart::gui::osg::InteractiveFrame>(
                    bn, bn->getName() + "/frame");

        for (const auto type : {dart::gui::osg::InteractiveTool::ANGULAR,
                                dart::gui::osg::InteractiveTool::PLANAR})
            for (std::size_t i = 0; i < 3; ++i)
                frame->getTool(type, i)->setEnabled(false);

        world->addSimpleFrame(frame);        
    }
}

void displayGroundFrames(dart::simulation::WorldPtr world,
                        const dart::dynamics::SkeletonPtr ground) {
    dart::dynamics::BodyNode* bn = ground->getBodyNode("world_frame");
    dart::gui::osg::InteractiveFramePtr frame =
    std::make_shared<dart::gui::osg::InteractiveFrame>(bn, bn->getName() + "/frame");

    for (const auto type : {dart::gui::osg::InteractiveTool::ANGULAR,
                            dart::gui::osg::InteractiveTool::PLANAR})
        for (std::size_t i = 0; i < 3; ++i)
            frame->getTool(type, i)->setEnabled(false);

    world->addSimpleFrame(frame);
}


void setJointLimitConstraint(dart::dynamics::SkeletonPtr robot) {
    for (int i = 0; i < robot->getNumJoints(); ++i) {
        dart::dynamics::Joint* joint = robot->getJoint(i);
        joint->setPositionLimitEnforced(true);
    }
}

void setTransparency(dart::dynamics::SkeletonPtr robot) {
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

void printRobotModel(const dart::dynamics::SkeletonPtr robot) {
    for (int i = 0; i < robot->getNumBodyNodes(); ++i) {
        dart::dynamics::BodyNodePtr bn = robot->getBodyNode(i);
        std::cout << i << "th" << std::endl;
        std::cout << bn->getName() << std::endl;
        std::cout << bn->getMass() << std::endl;    
    }

    for (int i = 0; i < robot->getNumJoints(); ++i) {
        dart::dynamics::Joint* joint = robot->getJoint(i);
        std::cout << i << "th" << std::endl;
        std::cout << joint->getNumDofs() << std::endl;
    }

    for (int i = 0; i < robot->getNumDofs(); ++i) {
        dart::dynamics::DegreeOfFreedom* dof = robot->getDof(i);
        std::cout << i << "th" << std::endl;
        std::cout << "dof name : " << dof->getName() << std::endl;
        // std::cout << "child body node name and mass : "
        //<< dof->getChildBodyNode()->getName() << " , "
        //<< dof->getChildBodyNode()->getMass() << std::endl;
    }

    std::cout << "num dof: " << robot->getNumDofs() << std::endl;
    std::cout << "num joint: " << robot->getNumJoints() << std::endl;
    // std::cout << "mass mat row" << std::endl;
    // std::cout << robot->getMassMatrix().rows() << std::endl;
    // std::cout << robot->getMassMatrix().cols() << std::endl;
    // std::cout << "robot total mass" << std::endl;
    // std::cout << robot->getMass() << std::endl;
    // std::cout << "robot position" << std::endl;
    // std::cout << robot->getPositions() << std::endl;

    exit(0);
}
