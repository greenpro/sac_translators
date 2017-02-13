#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <pluginlib/class_loader.h>
#include <std_msgs/Float64.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <boost/scoped_ptr.hpp>

#include <iostream>

//#define ANDREAS_ARM
#define SCORBOT

#ifdef SCORBOT
    #ifdef ANDREAS_ARM
        #error SCORBOT and ANDREAS_ARM cannot be defined at the same time.
    #endif
#endif

#ifdef ANDREAS_ARM
    #define PLANNING_GROUP "arm"
    #define TARGET_JOINT "epsilon_link"
#endif

#ifdef SCORBOT
    #define PLANNING_GROUP "main_group"
    #define TARGET_JOINT "roll_joint"
#endif

#define NODE_NAME "Jacobian"

using namespace std;

#ifdef ANDREAS_ARM
    ros::Publisher   alphaPub;
    ros::Publisher    betaPub;
    ros::Publisher   gammaPub;
    ros::Publisher   deltaPub;
    ros::Publisher epsilonPub;
    ros::Publisher    zetaPub;
#endif

#ifdef SCORBOT
    ros::Publisher     basePub;
    ros::Publisher shoulderPub;
    ros::Publisher    elbowPub;
    ros::Publisher    pitchPub;
    ros::Publisher     rollPub;
#endif

void callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    ROS_INFO_NAMED(NODE_NAME, "Hit");
    moveit::planning_interface::MoveGroupInterface moveGroup(PLANNING_GROUP);
    ROS_INFO_NAMED(NODE_NAME, "group");

    double x = msg->position.x;
    double y = msg->position.y;
    double z = msg->position.z;

    double roll  = msg->orientation.x;
    double pitch = msg->orientation.y;
    double yaw   = msg->orientation.z;

    moveGroup.setPositionTarget(   x,     y,   z, TARGET_JOINT);
    moveGroup.setRPYTarget(     roll, pitch, yaw, TARGET_JOINT);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = moveGroup.plan(plan);
    ROS_INFO_NAMED(NODE_NAME, "planned");
    ROS_INFO_NAMED(NODE_NAME, "Planning %s", success ? "done" : "failed");

    if (!success)
        return;

    vector<double> joints = moveGroup.getCurrentJointValues();
    
#ifdef ANDREAS_ARM
    std_msgs::Float64 alphaMsg;
    std_msgs::Float64 betaMsg;
    std_msgs::Float64 gammaMsg;
    std_msgs::Float64 deltaMsg;
    std_msgs::Float64 epsilonMsg;
    std_msgs::Float64 zetaMsg;
    
    alphaMsg.data   = joints[0];
    betaMsg.data    = joints[1];
    gammaMsg.data   = joints[2];
    deltaMsg.data   = joints[3];
    epsilonMsg.data = joints[4];
    zetaMsg.data    = joints[5];

    ROS_INFO_NAMED(NODE_NAME, "Alpha   %f", joints[0]);
    ROS_INFO_NAMED(NODE_NAME, "Beta    %f", joints[1]);
    ROS_INFO_NAMED(NODE_NAME, "Gamma   %f", joints[2]);
    ROS_INFO_NAMED(NODE_NAME, "Delta   %f", joints[3]);
    ROS_INFO_NAMED(NODE_NAME, "Epsilon %f", joints[4]);
    ROS_INFO_NAMED(NODE_NAME, "Zeta    %f", joints[5]);

    alphaPub.publish(    alphaMsg);
    betaPub.publish(      betaMsg);
    gammaPub.publish(    gammaMsg);
    deltaPub.publish(    deltaMsg);
    epsilonPub.publish(epsilonMsg);
    zetaPub.publish(      zetaMsg);
#endif

#ifdef SCORBOT
    std_msgs::Float64 baseMsg;
    std_msgs::Float64 shoulderMsg;
    std_msgs::Float64 elbowMsg;
    std_msgs::Float64 pitchMsg;
    std_msgs::Float64 rollMsg;
    
    baseMsg.data     = joints[0];
    shoulderMsg.data = joints[1];
    elbowMsg.data    = joints[2];
    pitchMsg.data    = joints[3];
    rollMsg.data     = joints[4];

    ROS_INFO_NAMED(NODE_NAME, "Base     %f", joints[0]);
    ROS_INFO_NAMED(NODE_NAME, "Shoulder %f", joints[1]);
    ROS_INFO_NAMED(NODE_NAME, "Elbow    %f", joints[2]);
    ROS_INFO_NAMED(NODE_NAME, "Pitch    %f", joints[3]);
    ROS_INFO_NAMED(NODE_NAME, "Roll     %f", joints[4]);

    basePub.publish(    baseMsg);
    shoulderPub.publish(shoulderMsg);
    elbowPub.publish(   elbowMsg);
    pitchPub.publish(   pitchMsg);
    rollPub.publish(    rollMsg);
#endif
}

int main(int argc, char **argv)
{
    ROS_INFO_NAMED(NODE_NAME, "starting");
    ros::init(argc, argv, NODE_NAME);
    
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("moveto", 1000, callback);

#ifdef ANDREAS_ARM
    alphaPub   = nh.advertise<std_msgs::Float64>(  "alphaMotor", 1000);
    betaPub    = nh.advertise<std_msgs::Float64>(   "betaMotor", 1000);
    gammaPub   = nh.advertise<std_msgs::Float64>(  "gammaMotor", 1000);
    deltaPub   = nh.advertise<std_msgs::Float64>(  "deltaMotor", 1000);
    epsilonPub = nh.advertise<std_msgs::Float64>("epsilonMotor", 1000);
    zetaPub    = nh.advertise<std_msgs::Float64>(   "zetaMotor", 1000);
#endif

#ifdef SCORBOT
    basePub     = nh.advertise<std_msgs::Float64>(    "baseMotor", 1000);
    shoulderPub = nh.advertise<std_msgs::Float64>("shoulderMotor", 1000);
    elbowPub    = nh.advertise<std_msgs::Float64>(   "elbowMotor", 1000);
    pitchPub    = nh.advertise<std_msgs::Float64>(   "pitchMotor", 1000);
    rollPub     = nh.advertise<std_msgs::Float64>(    "rollMotor", 1000);
#endif

    ros::spin();

    return 0;
}
