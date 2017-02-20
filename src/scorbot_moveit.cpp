// Moveit!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

//#include <moveit_visual_tools/moveit_visual_tools.h>

#define NODE_NAME "scorbot_moveit"
#define PLANNING_GROUP "arm"

int main(int argc, char **argv)
{
    // initialize the node
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // add the group to plan for
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // used for collision objects (not used with this node)
    // moveit::planning_interface::PlanningSceneInterface psi;

    // plan a motion for the end-effector
    geometry_msgs::Pose target;
    target.orientation.w = 1.0;
    target.position.x = 0.11;
    target.position.y = 0.11;
    target.position.z = 0.11;
    move_group.setPoseTarget(target);

    // make a plan to move to the point
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success = move_group.plan(plan);

    ROS_INFO_NAMED(NODE_NAME, "Planning %s", success ? "done" : "Failed");

    // get the joint positions
}
