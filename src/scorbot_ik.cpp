#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

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

#define NODE_NAME "IK"
#define PI 3.14159265

using namespace std;

ros::Publisher       basePub;
ros::Publisher   shoulderPub;
ros::Publisher      elbowPub;
ros::Publisher  wristRollPub;
ros::Publisher wristPitchPub;

void transMat(float x, float y, float z, float** out)
{
    out[0][0] = 1;
    out[0][1] = 0;
    out[0][2] = 0;
    out[0][3] = x;

    out[1][0] = 0;
    out[1][1] = 1;
    out[1][2] = 0;
    out[1][3] = y;

    out[2][0] = 0;
    out[2][1] = 0;
    out[2][2] = 1;
    out[2][3] = z;

    out[3][0] = 0;
    out[3][1] = 0;
    out[3][2] = 0;
    out[3][3] = 1;
}

void rollMat(float theta, float** out)
{
    out[0][0] = 1;
    out[0][1] = 0;
    out[0][2] = 0;
    out[0][3] = 0;

    out[1][0] = 0;
    out[1][1] = cos(theta);
    out[1][2] = 0 - sin(theta);
    out[1][3] = 0;

    out[2][0] = 0;
    out[2][1] = sin(theta);
    out[2][2] = cos(theta);
    out[2][3] = 0;

    out[3][0] = 0;
    out[3][1] = 0;
    out[3][2] = 0;
    out[3][3] = 1;
}

void pitchMat(float theta, float** out)
{
    out[0][0] = cos(theta);
    out[0][1] = 0;
    out[0][2] = sin(theta);
    out[0][3] = 0;

    out[1][0] = 0;
    out[1][1] = 1;
    out[1][2] = 0;
    out[1][3] = 0;

    out[2][0] = 0 - sin(theta);
    out[2][1] = 0;
    out[2][2] = cos(theta);
    out[2][3] = 0;

    out[3][0] = 0;
    out[3][1] = 0;
    out[3][2] = 0;
    out[3][3] = 1;
}

void yawMat(float theta, float** out)
{
    out[0][0] = cos(theta);
    out[0][1] = 0 - sin(theta);
    out[0][2] = 0;
    out[0][3] = 0;

    out[1][0] = sin(theta);
    out[1][1] = cos(theta);
    out[1][2] = 0;
    out[1][3] = 0;

    out[2][0] = 0;
    out[2][1] = 0;
    out[2][2] = 1;
    out[2][3] = 0;

    out[3][0] = 0;
    out[3][1] = 0;
    out[3][2] = 0;
    out[3][3] = 1;
}

void matMult(float** mat0, float** mat1, float** out)
{
    float mat[4][4];
    
    for (int x = 0; x < 4; x++)
    {
        for (int y = 0; y < 4; y++)
        {
            out[y][x] = mat[y][x] * mat[x][y];
        }
    }
}

void callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_INFO_NAMED(NODE_NAME, "Hit");

    float roll  = msg->angular.x;
    float pitch = msg->angular.y;
    float yaw   = msg->angular.z;

    float x = msg->linear.x;
    float y = msg->linear.y;
    float z = msg->linear.z;

    // Transformation Matrix
    float **tmat;
    transMat(x, y, z, tmat);

    float **rmat;
    rollMat(roll, rmat);

    float **pmat;
    pitchMat(pitch, pmat);

    float **ymat;
    yawMat(yaw, ymat);

    float **out1;
    matMult(ymat, pmat, out1);
    
    float **out2;
    matMult(out1, rmat, out2);

    float **out3;
    matMult(out2, tmat, out3);

    // Matrix variables
    float nx = out3[0][0];
    float ny = out3[0][1];
    float nz = out3[0][2];

    float ox = out3[1][0];
    float oy = out3[1][1];
    float oz = out3[1][2];
    
    float ax = out3[2][0];
    float ay = out3[2][1];
    float az = out3[2][2];

    float px = out3[0][4];
    float py = out3[1][4];
    float pz = out3[2][4];

    // Distances
    float d1 = 0.22;
    float d2 = 0.22;

    float a1 = 0;
    float a2 = 0;
    float a3 = 0;
    float a4 = 0;

    // equations
    float sqrt_pos = sqrt(pow(px, 2) + pow(py, 2) - pow(d2, 2));
    float sqrt_neg = 0 - sqrt_neg;
    float theta1_pos = atan2(py, px) - atan2(d2, sqrt_pos);
    float theta1_neg = atan2(py, px) - atan2(d2, sqrt_neg);
    // to change between pos and neg
    float theta1 = theta1_pos;

    float pz_prime = pz + a4;
    float top = pow(cos(theta1) * px + sin(theta1) * py - a1, 2) + pow(d1 - pz_prime + a4, 2) - pow(a2, 2) - pow(a3, 2);
    float bot = 2 * a2 * a3;
    float theta3_pos = acos( top / bot );
    float theta3_neg = 0 - theta3_pos;
    // to change between pos and neg
    float theta3 = theta3_pos;

    sqrt_pos = sqrt( pow(d1 - pz_prime + a4, 2) + pow(cos(theta1) * px + sin(theta1) * py - a1, 2) - pow(a3 * cos(theta3) + a2, 2) );
    sqrt_neg = 0 - sqrt_neg;
    float top_pos = (d1 - pz_prime + a4) + sqrt_pos;
    float top_neg = (d1 - pz_prime + a4) + sqrt_neg;
    bot = (cos(theta1) * px + sin(theta1) * py - a1) + (a3 * cos(theta3) + a2);
    float theta2_pos = (top_pos / bot);
    float theta2_neg = (top_neg / bot);
    // to change between pos and neg
    float theta2 = theta2_pos;

    float theta4 = (PI / 2) - theta2 - theta3;

    std_msgs::Float64 baseMsg;
    std_msgs::Float64 shoulderMsg;
    std_msgs::Float64 elbowMsg;
    std_msgs::Float64 wristPitchMsg;
    std_msgs::Float64 wristRollMsg;

    baseMsg.data        = theta1;
    shoulderMsg.data    = theta2;
    elbowMsg.data       = theta3;
    wristPitchMsg.data  = theta4;
    wristRollMsg.data   = 0;

    ROS_INFO_NAMED(NODE_NAME, "Base:        %f", theta1);
    ROS_INFO_NAMED(NODE_NAME, "Shoulder:    %f", theta2);
    ROS_INFO_NAMED(NODE_NAME, "Elbow:       %f", theta3);
    ROS_INFO_NAMED(NODE_NAME, "Wrist Pitch: %f", theta4);
    ROS_INFO_NAMED(NODE_NAME, "Wrist Roll:  %f", 0);

    basePub.publish(            baseMsg);
    shoulderPub.publish(    shoulderMsg);
    elbowPub.publish(          elbowMsg);
    wristPitchPub.publish(wristPitchMsg);
    wristRollPub.publish( wristRollMsg );
}

int main(int argc, char **argv)
{
    ROS_INFO_NAMED(NODE_NAME, "starting");
    ros::init(argc, argv, NODE_NAME);
    
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("moveto", 1000, callback);

    basePub       = nh.advertise<std_msgs::Float64>(      "baseMotor", 1000);
    shoulderPub   = nh.advertise<std_msgs::Float64>(  "shoulderMotor", 1000);
    elbowPub      = nh.advertise<std_msgs::Float64>(     "elbowMotor", 1000);
    wristPitchPub = nh.advertise<std_msgs::Float64>("wristPitchMotor", 1000);
    wristRollPub  = nh.advertise<std_msgs::Float64>( "wristRollMotor", 1000);

    ros::spin();

    return 0;
}
