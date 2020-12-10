#include <dart/dart.hpp>
#include <dart/utils/utils.hpp>
#include <dart/gui/osg/osg.hpp>
#include <my_utils/IO/IOUtilities.hpp>


void displayJointFrames(dart::simulation::WorldPtr world,
                        const dart::dynamics::SkeletonPtr robot);
void displayLinkFrames(dart::simulation::WorldPtr world,
                        const dart::dynamics::SkeletonPtr robot,
                        const std::vector<std::string>& link_list);
void displayLinkFrames(dart::simulation::WorldPtr world,
                        const dart::dynamics::SkeletonPtr robot) ;

void displayGroundFrames(dart::simulation::WorldPtr world,
                        const dart::dynamics::SkeletonPtr ground);

void setJointLimitConstraint(dart::dynamics::SkeletonPtr robot);
void setTransparency(dart::dynamics::SkeletonPtr robot);
void printRobotModel(const dart::dynamics::SkeletonPtr robot);