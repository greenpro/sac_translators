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

#define NODE_NAME "Scorbot Jacobian"

using namespace std;

ros::Publisher     basePub;
ros::Publisher shoulderPub;
ros::Publisher    elbowPub;
ros::Publisher    pitchPub;
ros::Publisher     rollPub;

void callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    double x = msg->linear.x;
    double y = msg->linear.y;
    double z = msg->linear.z;

    double roll  = msg->angular.x;
    double pitch = msg->angular.y;
    double yaw   = msg->angular.z;

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
}

int main(int argc, char **argv)
{
    ROS_INFO_NAMED(NODE_NAME, "starting");
    ros::init(argc, argv, NODE_NAME);
    
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("moveto", 1000, callback);

    basePub     = nh.advertise<std_msgs::Float64>(    "baseMotor", 1000);
    shoulderPub = nh.advertise<std_msgs::Float64>("shoulderMotor", 1000);
    elbowPub    = nh.advertise<std_msgs::Float64>(   "elbowMotor", 1000);
    pitchPub    = nh.advertise<std_msgs::Float64>(   "pitchMotor", 1000);
    rollPub     = nh.advertise<std_msgs::Float64>(    "rollMotor", 1000);

    ros::spin();

    return 0;
}
