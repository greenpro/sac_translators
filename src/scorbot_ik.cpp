#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <sac_msgs/Target.h>
#include <sac_msgs/MotorPos.h>

#define NODE_NAME "IK"
#define PI 3.14159265

#define B_MIN (-2.7053)
#define B_MAX (2.7053)
#define S_MIN (-0.6109)
#define S_MAX (2.2689)
#define E_MIN (-2.2689)
#define E_MAX (2.2689)
#define WP_MIN (-2.2689)
#define WP_MAX (2.2689)

ros::Publisher       basePub;
ros::Publisher   shoulderPub;
ros::Publisher      elbowPub;
ros::Publisher  wristRollPub;
ros::Publisher wristPitchPub;

bool checkAngles(float angles[5])
{
    if (angles[0] < B_MIN)
    {
        ROS_INFO("Base angle (%f) is less than the minimum angle (%f)",  angles[0], B_MIN);
        return false;
    }
    if (angles[0] > B_MAX)
    {
        ROS_INFO("Base angle (%f) is greater than the maximum angle (%f)",  angles[0], B_MAX);
        return false;
    }

    if (angles[1] < S_MIN)
    {
        ROS_INFO("Shoulder angle (%f) is less than the minimum angle (%f)",  angles[1], S_MIN);
        return false;
    }
    if (angles[1] > S_MAX)
    {
        ROS_INFO("Shoulder angle (%f) is greater than the maximum angle (%f)",  angles[1], S_MAX);
        return false;
    }

    if (angles[2] < E_MIN)
    {
        ROS_INFO("Elbow angle (%f) is less than the minimum angle (%f)",  angles[2], E_MIN);
        return false;
    }
    if (angles[2] > E_MAX)
    {
        ROS_INFO("Elbow angle (%f) is greater than the maximum angle (%f)",  angles[2], E_MAX);
        return false;
    }
    
    if (angles[3] < WP_MIN)
    {
        ROS_INFO("Wrist pitch angle (%f) is less than the minimum angle (%f)",  angles[3], WP_MIN);
        return false;
    }
    if (angles[3] > WP_MAX)
    {
        ROS_INFO("Wrist pitch angle (%f) is greater than the maximum angle (%f)",  angles[3], WP_MAX);
        return false;
    }

    return true;
}

// NOTE :: All angles are in radians.
// NOTE :: All distances in this function are in meters to follow the ros standard.
// NOTE :: For this function angles contain capitolized letters distances are lower case and modifications to the variable are after the "_".
bool ik(float x, float y, float z, float R, float P, float out[5])
{
    // check if the end effector is beneath the floor.
    if (z < 0)
    {
        ROS_INFO("Invaled point");
        return false;
    }

    // knowns
    float d0 = 0.350;
    float d1 = 0.030;
    float d2 = 0.220;
    float d3 = 0.220;
    float d4 = 0.160;

    ROS_INFO("x: %f, y: %f, z: %f, roll: %f, pitch: %f", x, y, z, R, P);
    float d4z = sin(P) * d4;
    ROS_INFO("d4z: %f", d4z);

    // check if the wrist joint or the back of the end effector is beneath the floor.
    // the 0.004 is to account for the radius of the joint.
    if (z + d4z - 0.004 < 0 || z + 2 * d4z < 0)
    {
        ROS_INFO("Invalid wrist joint position - joint Z: %f end Z: %f", z - d4z - 0.004, z - 2 * d4z);
        return false;
    }
    
    float d4r = cos(P) * d4;
    ROS_INFO("d4r: %f", d4r);
    float x_squared = pow(x, 2);
    ROS_INFO("x^2: %f", x_squared);
    float y_squared = pow(y, 2);
    ROS_INFO("y^2: %f", y_squared);
    float r = sqrt(x_squared + y_squared);
    ROS_INFO("r: %f", r);
    float er = r - d1 - d4r;
    ROS_INFO("er: %f", er);
    float ez = z + d4z - d0;
    ROS_INFO("ez: %f", ez);
    float er_squared = pow(er, 2);
    ROS_INFO("er^2: %f", er_squared);
    float ez_squared = pow(ez, 2);
    ROS_INFO("ez^2: %f", ez_squared);
    float e_squared = er_squared + ez_squared;
    ROS_INFO("e^2: %f", e_squared);
    float e = sqrt(e_squared);
    ROS_INFO("e: %f", e);
    float d2_squared = pow(d2, 2);
    ROS_INFO("d2^2: %f", d2_squared);
    float d3_squared = pow(d3, 2);
    ROS_INFO("d3^2: %f", d3_squared);
    float A_top = d3_squared - d2_squared - e_squared;
    ROS_INFO("A_top: %f", A_top);
    float A_bot = -2 * d2 * e;
    ROS_INFO("A_bot: %f", A_bot);
    float A = acos(A_top / A_bot);
    ROS_INFO("A: %f", A);
    float E_top = e_squared - d3_squared - d2_squared;
    ROS_INFO("E_top: %f", E_top);
    float E_bot = -2 * d2 * d3;
    ROS_INFO("E_bot: %f", E_bot);
    float E = PI - acos(E_top / E_bot);
    ROS_INFO("E: %f", E);
    float Ez_top = ez_squared - e_squared - er_squared;
    ROS_INFO("Ez_top: %f", Ez_top);
    float Ez_bot = -2 * e * er;
    ROS_INFO("Ez_bot: %f", Ez_bot);
    float Ez = acos(Ez_top / Ez_bot);
    ROS_INFO("Ez: %f", Ez);
    float S = A + atan2(z, x) - atan2(d3 * sin(E), d2 + d3 * cos(E));//z + d4z < z ? A + Ez : A - Ez;
    ROS_INFO("S: %f", S);
    float Salt = (E - S);
    ROS_INFO("Salt: %f", Salt);
    float B = atan2(y, x);
    ROS_INFO("B: %f", B);
    float PI_div2 = PI / 2;
    ROS_INFO("PI_div2: %f", PI_div2);
    float G = PI_div2 - Salt;
    ROS_INFO("G: %f", G);
    float Wp =  P - (S + E);//z < d4z ? PI_div2 + G - P: PI_div2 - G + P;
    ROS_INFO("Wp: %f", Wp);
    float Wr = R;
    ROS_INFO("Wr: %f", Wr);
    
    if (er > d2 + d3)
    {
        ROS_INFO("The point is too far away.");
        return false;
    }

    // assign the outputs
    out[0] = B;
    out[1] = S;
    out[2] = E;
    out[3] = Wp;
    out[4] = Wr;

    ROS_INFO("Base:        %f - %f",   out[0], out[0] * 180 / PI);
    ROS_INFO("Shoulder:    %f - %f",   out[1], out[1] * 180 / PI);
    ROS_INFO("Elbow:       %f - %f",   out[2], out[2] * 180 / PI);
    ROS_INFO("Wrist Pitch: %f - %f",   out[3], out[3] * 180 / PI);
    ROS_INFO("Wrist Roll:  %f - %f\n", out[4], out[4] * 180 / PI);

    if (checkAngles(out))
        return true;
    ROS_INFO("Invalid joint angles");

    float H = PI_div2 - S;
    float Ealt = -E;
    float Wpalt = (H + P) - PI;

    out[1] = Salt;
    out[2] = Ealt;
    out[3] = Wpalt;

    ROS_INFO("Alternate");
    ROS_INFO("Base:        %f - %f",   out[0], out[0] * 180 / PI);
    ROS_INFO("Shoulder:    %f - %f",   out[1], out[1] * 180 / PI);
    ROS_INFO("Elbow:       %f - %f",   out[2], out[2] * 180 / PI);
    ROS_INFO("Wrist Pitch: %f - %f",   out[3], out[3] * 180 / PI);
    ROS_INFO("Wrist Roll:  %f - %f\n", out[4], out[4] * 180 / PI);

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
    
    if (!ik(x, y, z, roll, pitch, out))
        return;

    sac_msgs::MotorPos pos;
    pos.speed = msg->speed;
    
    pos.pos = out[0];
    basePub.publish(pos);

    pos.pos = out[1];
    shoulderPub.publish(pos);
    
    pos.pos = out[2];
    elbowPub.publish(pos);

    pos.pos = out[3];
    wristPitchPub.publish(pos);
    
    pos.pos = out[4];
    wristRollPub.publish(pos);
}

int main(int argc, char **argv)
{
    ROS_INFO_NAMED(NODE_NAME, "starting");
    ros::init(argc, argv, NODE_NAME);
    
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("moveto", 1000, callback);

    basePub       = nh.advertise<sac_msgs::MotorPos>(      "baseMotor", 1000);
    shoulderPub   = nh.advertise<sac_msgs::MotorPos>(  "shoulderMotor", 1000);
    elbowPub      = nh.advertise<sac_msgs::MotorPos>(     "elbowMotor", 1000);
    wristPitchPub = nh.advertise<sac_msgs::MotorPos>("wristPitchMotor", 1000);
    wristRollPub  = nh.advertise<sac_msgs::MotorPos>( "wristRollMotor", 1000);

    ros::spin();

    return 0;
}
