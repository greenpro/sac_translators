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

void transMat(float x, float y, float z, float out[4][4])
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

void rollMat(float theta, float out[4][4])
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

void pitchMat(float theta, float out[4][4])
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

void yawMat(float theta, float out[4][4])
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

void matMult(float mat0[4][4], float mat1[4][4], float out[4][4])
{
    ROS_INFO_NAMED(NODE_NAME, "############################################");
    
    ROS_INFO_NAMED(NODE_NAME, "_mat0_______________________________________");
    ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", mat0[0][0], mat0[0][1], mat0[0][2], mat0[0][3]);  
    ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", mat0[1][0], mat0[1][1], mat0[1][2], mat0[1][3]);  
    ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", mat0[2][0], mat0[2][1], mat0[2][2], mat0[2][3]);  
    ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", mat0[3][0], mat0[3][1], mat0[3][2], mat0[3][3]);  
    ROS_INFO_NAMED(NODE_NAME, "L__________________________________________J");
    ROS_INFO_NAMED(NODE_NAME, "_mat1_______________________________________");
    ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", mat1[0][0], mat1[0][1], mat1[0][2], mat1[0][3]);  
    ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", mat1[1][0], mat1[1][1], mat1[1][2], mat1[1][3]);  
    ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", mat1[2][0], mat1[2][1], mat1[2][2], mat1[2][3]);  
    ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", mat1[3][0], mat1[3][1], mat1[3][2], mat1[3][3]);  
    ROS_INFO_NAMED(NODE_NAME, "L__________________________________________J");
    for (int ox = 0; ox < 4; ox++)
    {
        for (int oy = 0; oy < 4; oy++)
        {
            out[oy][ox] = 0;
            for (int i = 0; i < 4; i++)
            {
                out[oy][ox] += mat0[oy][i] * mat1[i][ox];
            }
        }
    }
    ROS_INFO_NAMED(NODE_NAME, "_out_=_mat0_*_mat1__________________________");
    ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", out[0][0], out[0][1], out[0][2], out[0][3]);  
    ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", out[1][0], out[1][1], out[1][2], out[1][3]);  
    ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", out[2][0], out[2][1], out[2][2], out[2][3]);  
    ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", out[3][0], out[3][1], out[3][2], out[3][3]);  
    ROS_INFO_NAMED(NODE_NAME, "L__________________________________________J");
}

void matAdd(float mat0[4][4], float mat1[4][4], float out[4][4])
{
    for (int x=0; x<4; x++)
    {
        for (int y=0; y<4; y++)
        {
            if (mat0[y][x] == 1)
                mat0[y][x] = 0;

            if (mat1[y][x] == 1)
                mat1[y][x] = 0;

            out[y][x] = mat0[y][x] + mat1[y][x]; 
        }
    }
}

bool checkAngles(float angles[5])
{
    ROS_INFO("0: %f - %f",  B_MIN,  B_MAX);
    ROS_INFO("1: %f - %f",  S_MIN,  S_MAX);
    ROS_INFO("2: %f - %f",  E_MIN,  E_MAX);
    ROS_INFO("3: %f - %f", WP_MIN, WP_MAX);
    ROS_INFO("4: -INF - INF\n");
    
    // For the scorbot er-III there is no limit for the roll.
    if (angles[0] < B_MIN || angles[0] > B_MAX ||
            angles[1] < S_MIN || angles[1] > B_MAX ||
            angles[2] < E_MIN || angles[2] > E_MAX ||
            angles[3] < WP_MIN || angles[3] > WP_MAX //||
            //angles[4] < WR_MIN || angles[4] > WR_MIN
            )
        return false;
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
    float d0 = 0.349;
    float d1 = 0.016;
    float d2 = 0.220;
    float d3 = 0.220;
    float d4 = 0.045;

    float d4z = sin(P) * d4;

    // check if the wrist joint or the back of the end effector is beneath the floor.
    // the 0.004 is to account for the radius of the joint.
    if (z - d4z - 0.004 < 0 || z - 2 * d4z - 0.004 < 0)
    {
        ROS_INFO("Invalid wrist joint position");
        return false;
    }
    
    float d4r = cos(P) * d4;
    float x_squared = pow(x, 2);
    float y_squared = pow(y, 2);
    float r = sqrt(x_squared + y_squared);
    float er = r - d1 - d4r;
    float ez = z - d0 - d4z;
    float er_squared = pow(er, 2);
    float ez_squared = pow(ez, 2);
    float e_squared = er_squared + ez_squared;
    float e = sqrt(e_squared);
    float d2_squared = pow(d2, 2);
    float d3_squared = pow(d3, 2);
    float A_top = d3_squared - d2_squared - e_squared;
    float A_bot = -2 * d2 * e;
    float A = acos(A_top / A_bot);
    float E_top = e_squared - d3_squared - d2_squared;
    float E_bot = -2 * d2 * d3;
    float E = acos(E_top / E_bot) - PI;
    float Ez_top = ez_squared - e_squared - er_squared;
    float Ez_bot = -2 * e * er;
    float Ez = acos(Ez_top / Ez_bot);
    float S = A + Ez;
    float Salt = (E - S);
    float B = atan2(y, x);
    float PI_div2 = PI / 2;
    float G = PI_div2 - Salt;
    float Wp = PI_div2 - G + P;
    float Wr = R;
    
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
    ROS_INFO_NAMED(NODE_NAME, "Hit");

    float roll  = msg->roll;
    float pitch = msg->pitch;

    float x = msg->x;
    float y = msg->y;
    float z = msg->z;

    float out[5];
    ROS_INFO_NAMED(NODE_NAME, "ik call");
    ik(x, y, z, roll, pitch, out);

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
