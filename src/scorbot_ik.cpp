#include <ros/ros.h>
#include <sac_msgs/Target.h>
#include <sac_msgs/MotorPos.h>

namespace ik 
{
    // constants
    const char *nodeName = "IK";
    const char *subscribe = "moveto";
    const float pi = 3.1415926535898;

    // arm constants
    const float baselbowMin = -2.7053;
    const float baselbowMax = 2.7053;
    const float shoulderMin = -0.6109;
    const float shoulderMax = 2.2689;
    const float elbowMin = -2.2689;
    const float elbowMax = 2.2689;
    const float pitchMin = -2.2689;
    const float pitchMax = 2.2689;

    const float d0 = 0.360;
    const float d1 = 0.030;
    const float d2 = 0.220;
    const float d3 = 0.220;
    const float d4 = 0.140; // to the end of the gripper

    ros::Publisher       basePub;
    ros::Publisher   shoulderPub;
    ros::Publisher      elbowPub;
    ros::Publisher  wristRollPub;
    ros::Publisher wristPitchPub;
}

bool checkAngles(float angles[5])
{
    if (angles[0] < ik::baselbowMin)
    {
        ROS_INFO("Base angle (%f) is less than the minimum angle (%f)",  angles[0], ik::baselbowMin);
        return false;
    }
    if (angles[0] > ik::baselbowMax)
    {
        ROS_INFO("Base angle (%f) is greater than the maximum angle (%f)",  angles[0], ik::baselbowMax);
        return false;
    }

    if (angles[1] < ik::shoulderMin)
    {
        ROS_INFO("Shoulder angle (%f) is less than the minimum angle (%f)",  angles[1], ik::shoulderMin);
        return false;
    }
    if (angles[1] > ik::shoulderMax)
    {
        ROS_INFO("Shoulder angle (%f) is greater than the maximum angle (%f)",  angles[1], ik::shoulderMax);
        return false;
    }

    if (angles[2] < ik::elbowMin)
    {
        ROS_INFO("Elbow angle (%f) is less than the minimum angle (%f)",  angles[2], ik::elbowMin);
        return false;
    }
    if (angles[2] > ik::elbowMax)
    {
        ROS_INFO("Elbow angle (%f) is greater than the maximum angle (%f)",  angles[2], ik::elbowMax);
        return false;
    }
    
    if (angles[3] < ik::pitchMin)
    {
        ROS_INFO("Wrist pitch angle (%f) is less than the minimum angle (%f)",  angles[3], ik::pitchMin);
        return false;
    }
    if (angles[3] > ik::pitchMax)
    {
        ROS_INFO("Wrist pitch angle (%f) is greater than the maximum angle (%f)",  angles[3], ik::pitchMax);
        return false;
    }

    return true;
}

// NOTE :: All angles are in radians.
// NOTE :: All distances in this function are in meters to follow the ros standard.
// NOTE :: For this function angles contain capitolized letters distances are lower case and modifications to the variable are after the "_".
bool ikSolve(float x, float y, float z, float ROLL, float PITCH, float out[5])
{
    // check if the end effector is beneath the floor.
    if (z < 0)
    {
        ROS_INFO("Invaled point");
        return false;
    }

    // Equations
    float d4z = sin(PITCH) * ik::d4;

    // check if the wrist joint or the back of the end effector is beneath the floor.
    // the 0.004 is to account for the radius of the joint.
    if (z + d4z - 0.004 < 0 || z + 2 * d4z < 0)
    {
        ROS_INFO("Invalid wrist joint position - joint Z: %f end Z: %f", z - d4z - 0.004, z - 2 * d4z);
        return false;
    }
    
    float d4r = cos(PITCH) * ik::d4;
    float x_squared = pow(x, 2);
    float y_squared = pow(y, 2);
    float r = sqrt(x_squared + y_squared);
    float er = r - ik::d1 - d4r;
    
    if (er > ik::d2 + ik::d3)
    {
        ROS_INFO("The point is too far away.");
        return false;
    }
    
    float ez = z + d4z - ik::d0;
    float er_squared = pow(er, 2);
    float ez_squared = pow(ez, 2);
    float d2_squared = pow(ik::d2, 2);
    float d3_squared = pow(ik::d3, 2);
    float E_top = er_squared + ez_squared - d2_squared - d3_squared;
    float E_bot = 2 * ik::d2 * ik::d3;
    float E = atan2(sqrt(1 - pow(E_top / E_bot, 2)), E_top / E_bot);
    float e_squared = er_squared + ez_squared;
    float e = sqrt(e_squared);
    float Ez_top = ez_squared - e_squared - er_squared;
    float Ez_bot = -2 * e * er;
    float Ez = acos(Ez_top / Ez_bot);
    float A_top = d3_squared - d2_squared - e_squared;
    float A_bot = -2 * ik::d2 * e;
    float A = acos(A_top / A_bot); 
    float S = A - Ez;
    float P =  PITCH - (E - S);
    float B = atan2(y, x);
    float R = ROLL;

    // assign the outputs
    out[0] = B;
    out[1] = S;
    out[2] = E;
    out[3] = P;
    out[4] = R;

    ROS_INFO("");
    ROS_INFO("Inputs");
    ROS_INFO("--------------------");
    ROS_INFO("X:           %f",   x);
    ROS_INFO("Y:           %f",   y);
    ROS_INFO("Z:           %f",   z);
    ROS_INFO("roll:        %f",   ROLL);
    ROS_INFO("pitch:       %f\n", PITCH);
    
    ROS_INFO("Knowns");
    ROS_INFO("--------------------");
    ROS_INFO("base x:      %f", ik::d1);
    ROS_INFO("base z:      %f", ik::d0);
    ROS_INFO("shoulder x:  %f", ik::d2);
    ROS_INFO("elbow x:     %f", ik::d3);
    ROS_INFO("hand x:      %f\n", ik::d4);

    ROS_INFO("Equations");
    ROS_INFO("--------------------");
    ROS_INFO("d4z: %f", d4z);
    ROS_INFO("d4r: %f", d4r);
    ROS_INFO("x^2: %f", x_squared);
    ROS_INFO("y^2: %f", y_squared);
    ROS_INFO("r: %f", r);
    ROS_INFO("er: %f", er);
    ROS_INFO("ez: %f", ez);
    ROS_INFO("er^2: %f", er_squared);
    ROS_INFO("ez^2: %f", ez_squared);
    ROS_INFO("d2^2: %f", d2_squared);
    ROS_INFO("d3^2: %f", d3_squared);
    ROS_INFO("E_top: %f", E_top);
    ROS_INFO("E_bot: %f", E_bot);
    ROS_INFO("E: %f", E);
    ROS_INFO("S: %f", S);
    ROS_INFO("B: %f", B);
    ROS_INFO("P: %f", P);
    ROS_INFO("R: %f", R);
    ROS_INFO("A: %f", A);
    ROS_INFO("Ez: %f", Ez);
    
    ROS_INFO("Outputs");
    ROS_INFO("--------------------");
    ROS_INFO("Base:        %f - %f",   out[0], out[0] * 180 / ik::pi);
    ROS_INFO("Shoulder:    %f - %f",   out[1], out[1] * 180 / ik::pi);
    ROS_INFO("Elbow:       %f - %f",   out[2], out[2] * 180 / ik::pi);
    ROS_INFO("Wrist Pitch: %f - %f",   out[3], out[3] * 180 / ik::pi);
    ROS_INFO("Wrist Roll:  %f - %f\n", out[4], out[4] * 180 / ik::pi);

    if (checkAngles(out))
        return true;
    ROS_INFO("Invalid joint angles");

    float H = ik::pi / 2 - S;
    float Salt = (E - S);
    float Ealt = -E;
    float Palt = (H + PITCH) - ik::pi;

    out[1] = Salt;
    out[2] = Ealt;
    out[3] = Palt;

    ROS_INFO("Alternate");
    ROS_INFO("Base:        %f - %f",   out[0], out[0] * 180 / ik::pi);
    ROS_INFO("Shoulder:    %f - %f",   out[1], out[1] * 180 / ik::pi);
    ROS_INFO("Elbow:       %f - %f",   out[2], out[2] * 180 / ik::pi);
    ROS_INFO("Wrist Pitch: %f - %f",   out[3], out[3] * 180 / ik::pi);
    ROS_INFO("Wrist Roll:  %f - %f\n", out[4], out[4] * 180 / ik::pi);

    if (checkAngles(out))
        return true;
    ROS_INFO("Invalid alternate joint angles");
    return false;
}

void callback(const sac_msgs::Target::ConstPtr& msg)
{
    float roll  = msg->roll;
    float pitch = msg->pitch;

    float x = msg->x;
    float y = msg->y;
    float z = msg->z;

    float out[5];
    
    if (!ikSolve(x, y, z, roll, pitch, out))
        return;

    ROS_INFO("--------------publishing");
    sac_msgs::MotorPos pos;
    pos.speed = msg->speed;
    
    pos.pos = out[0];
    ik::basePub.publish(pos);

    pos.pos = out[1];
    ik::shoulderPub.publish(pos);
    
    pos.pos = out[2];
    ik::elbowPub.publish(pos);

    pos.pos = out[3];
    ik::wristPitchPub.publish(pos);
    
    pos.pos = out[4];
    ik::wristRollPub.publish(pos);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, ik::nodeName);
    
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(ik::subscribe, 1000, callback);

    ik::basePub       = nh.advertise<sac_msgs::MotorPos>(    "/baseMotor", 1000);
    ik::shoulderPub   = nh.advertise<sac_msgs::MotorPos>("/shoulderMotor", 1000);
    ik::elbowPub      = nh.advertise<sac_msgs::MotorPos>(   "/elbowMotor", 1000);
    ik::wristPitchPub = nh.advertise<sac_msgs::MotorPos>(   "/pitchMotor", 1000);
    ik::wristRollPub  = nh.advertise<sac_msgs::MotorPos>(    "/rollMotor", 1000);

    ros::spin();

    return 0;
}
