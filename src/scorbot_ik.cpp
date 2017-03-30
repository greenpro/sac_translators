#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <sac_msgs/target.h>

#define NODE_NAME "IK"
#define PI 3.14159265

#define B_MIN (-2.7053)
#define B_MAX (2.7053)
#define S_MIN (-0.6109)
#define S_MAX (2.2689)
#define E_MIN (-1.1349)
#define E_MAX (1.1349)
#define WP_MIN (-1.1349)
#define WP_MAX (1.1349)

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

//void ik(float ax, float ay, float az, float px, float py, float pz, float pitch, float roll)
//{
//    float theta1   = atan2(py, px);
//
//    float c1 = cos(theta1);
//    float s1 = sin(theta1);
//    float theta234 = 0 - atan2(c1 * ax + s1 * ay, az);
//    
//    float s234 = sin(theta234);
//
//    float c234 = cos(theta234);
//
//    float c3_top = pow(c1 * px + s1 * py - s234 * D5, 2) + pow(pz + c234 * D5, 2) - pow(A2, 2) - pow(A3, 2);
//    float c3_bot = 2 * A2 * A3;
//    float c3 = c3_top / c3_bot;
//
//    float s3 = sqrt(1 + pow(c3, 2));
//
//    float theta3   = atan2(s3, c3);
//
//    float c2_top = (c1 * px + s1 * py - s234 * D5) * (A3 * c3 + A2) + (pz + c234 * D5) * s3 * A3;
//    float c2_bot = pow(A3 * c3 + A2, 2) + pow(s3, 2) * pow(A3, 2);
//    float c2 = c2_top / c2_bot;
//    
//    float s2_top = (c1 * px + s1 * py - s234 * D5) * (A3 * s3) + (pz + c234 * D5) * (A3 * c3 + A2);
//    float s2_bot = pow(A3 * c3 + A2, 2) + pow(s3, 2) * pow(A3, 2);
//    float s2 = s2_top / s2_bot;
//
//    float theta2   = atan2(s2, c2);
//
//    float theta4   = pitch;
//    float theta5   = roll;
//}

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
//        return false;
    }

    // knowns
    float d0 = 0.200;
    float d1 = 0.100;
    float d2 = 0.220;
    float d3 = 0.220;
    float d4 = 0.150;

    float d4z = sin(P) * d4;

    // check if the wrist joint or the back of the end effector is beneath the floor.
    // the 0.004 is to account for the radius of the joint.
    if (z - d4z - 0.004 < 0 || z - 2 * d4z - 0.004 < 0)
    {
        ROS_INFO("Invalid wrist joint position");
//        return false;
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
    float G = PI - Salt;
    float H = PI - S;
    float Wp = PI / 2 - G + P;
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
;//        return true;
    ROS_INFO("Invalid joint angles");

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

void callback(const sac_msgs::target::ConstPtr& msg)
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

    // Transformation Matrix
    //float tmat[4][4];
    //transMat(x, y, z, tmat);
    ////ROS_INFO_NAMED(NODE_NAME, "_tmat_______________________________________");
    ////ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", tmat[0][0], tmat[0][1], tmat[0][2], tmat[0][3]);  
    ////ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", tmat[1][0], tmat[1][1], tmat[1][2], tmat[1][3]);  
    ////ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", tmat[2][0], tmat[2][1], tmat[2][2], tmat[2][3]);  
    ////ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", tmat[3][0], tmat[3][1], tmat[3][2], tmat[3][3]);  
    ////ROS_INFO_NAMED(NODE_NAME, "L__________________________________________J");

    //float rmat[4][4];
    //rollMat(roll, rmat);
    ////ROS_INFO_NAMED(NODE_NAME, "_rmat_______________________________________");
    ////ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", rmat[0][0], rmat[0][1], rmat[0][2], rmat[0][3]);  
    ////ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", rmat[1][0], rmat[1][1], rmat[1][2], rmat[1][3]);  
    ////ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", rmat[2][0], rmat[2][1], rmat[2][2], rmat[2][3]);  
    ////ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", rmat[3][0], rmat[3][1], rmat[3][2], rmat[3][3]);  
    ////ROS_INFO_NAMED(NODE_NAME, "L__________________________________________J");

    //float pmat[4][4];
    //pitchMat(pitch, pmat);
    ////ROS_INFO_NAMED(NODE_NAME, "_pmat_______________________________________");
    ////ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", pmat[0][0], pmat[0][1], pmat[0][2], pmat[0][3]);  
    ////ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", pmat[1][0], pmat[1][1], pmat[1][2], pmat[1][3]);  
    ////ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", pmat[2][0], pmat[2][1], pmat[2][2], pmat[2][3]);  
    ////ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", pmat[3][0], pmat[3][1], pmat[3][2], pmat[3][3]);  
    ////ROS_INFO_NAMED(NODE_NAME, "L__________________________________________J");

    //float ymat[4][4];
    //yawMat(yaw, ymat);
    ////ROS_INFO_NAMED(NODE_NAME, "_ymat_______________________________________");
    ////ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", ymat[0][0], ymat[0][1], ymat[0][2], ymat[0][3]);  
    ////ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", ymat[1][0], ymat[1][1], ymat[1][2], ymat[1][3]);  
    ////ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", ymat[2][0], ymat[2][1], ymat[2][2], ymat[2][3]);  
    ////ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", ymat[3][0], ymat[3][1], ymat[3][2], ymat[3][3]);  
    ////ROS_INFO_NAMED(NODE_NAME, "L__________________________________________J");

    //float out1[4][4];
    //matAdd(ymat, pmat, out1);
    //
    //float out2[4][4];
    //matAdd(out1, rmat, out2);
    ////ROS_INFO_NAMED(NODE_NAME, "_out2_=_rmat_*_out1_________________________");
    ////ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", out2[0][0], out2[0][1], out2[0][2], out2[0][3]);  
    ////ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", out2[1][0], out2[1][1], out2[1][2], out2[1][3]);  
    ////ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", out2[2][0], out2[2][1], out2[2][2], out2[2][3]);  
    ////ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", out2[3][0], out2[3][1], out2[3][2], out2[3][3]);  
    ////ROS_INFO_NAMED(NODE_NAME, "L__________________________________________J");

    //float out3[4][4];
    //matAdd(out2, tmat, out3);
    ////ROS_INFO_NAMED(NODE_NAME, "_out3_=_tmat_*_out2_________________________");
    ////ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", out3[0][0], out3[0][1], out3[0][2], out3[0][3]);  
    ////ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", out3[1][0], out3[1][1], out3[1][2], out3[1][3]);  
    ////ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", out3[2][0], out3[2][1], out3[2][2], out3[2][3]);  
    ////ROS_INFO_NAMED(NODE_NAME, "|%f %f %f %f|", out3[3][0], out3[3][1], out3[3][2], out3[3][3]);  
    ////ROS_INFO_NAMED(NODE_NAME, "L__________________________________________J");

    //// Matrix variables
    //float nx = out3[0][0];
    //float ny = out3[1][0];
    //float nz = out3[2][0];
    //                    
    //float ox = out3[0][1];
    //float oy = out3[1][1];
    //float oz = out3[2][1];
    //                    
    //float ax = out3[0][2];
    //float ay = out3[1][2];
    //float az = out3[2][2];

    //float px = out3[0][3];
    //float py = out3[1][3];
    //float pz = out3[2][3];

    ////TODO :: set the px,y,z and ax,y,z to inputs or constants and see what it does with other inputs

    //// IK
    //float theta1   = atan2(px, py);

    //float c1 = cos(theta1);
    //float s1 = sin(theta1);
    //float theta234 = 0 - atan2(c1 * ax + s1 * ay, az);
    //
    //float s234 = sin(theta234);

    //float c234 = cos(theta234);

    //float c3_top = pow(c1 * px + s1 * py - s234 * D5, 2) + pow(pz + c234 * D5, 2) - pow(A2, 2) - pow(A3, 2);
    //float c3_bot = 2 * A2 * A3;
    //float c3 = c3_top / c3_bot;

    //float s3 = sqrt(1 + pow(c3, 2));

    //float theta3   = atan2(s3, c3);

    //float c2_top = (c1 * px + s1 * py - s234 * D5) * (A3 * c3 + A2) + (pz + c234 * D5) * s3 * A3;
    //float c2_bot = pow(A3 * c3 + A2, 2) + pow(s3, 2) * pow(A3, 2);
    //float c2 = c2_top / c2_bot;
    //
    //float s2_top = (c1 * px + s1 * py - s234 * D5) * (A3 * s3) + (pz + c234 * D5) * (A3 * c3 + A2);
    //float s2_bot = pow(A3 * c3 + A2, 2) + pow(s3, 2) * pow(A3, 2);
    //float s2 = s2_top / s2_bot;

    //float theta2   = atan2(s2, c2);

    //float theta4   = pitch;
    //float theta5   = roll;

    ////// equations
    ////float sqrt_pos = sqrt(pow(px, 2) + pow(py, 2) - pow(D2, 2));
    ////float sqrt_neg = 0 - sqrt_neg;
    ////float theta1_pos = atan2(py, px) - atan2(D2, sqrt_pos);
    ////float theta1_neg = atan2(py, px) - atan2(D2, sqrt_neg);
    ////// to change between pos and neg
    ////float theta1 = theta1_pos;

    ////float pz_prime = pz + A4;
    ////float top = pow(cos(theta1) * px + sin(theta1) * py - A1, 2) + pow(D1 - pz_prime + A4, 2) - pow(A2, 2) - pow(A3, 2);
    ////float bot = 2 * A2 * A3;
    ////float theta3_pos = acos( top / bot );
    ////float theta3_neg = 0 - theta3_pos;
    ////// to change between pos and neg
    ////float theta3 = theta3_pos;

    ////sqrt_pos = sqrt( pow(D1 - pz_prime + A4, 2) + pow(cos(theta1) * px + sin(theta1) * py - A1, 2) - pow(A3 * cos(theta3) + A2, 2) );
    ////sqrt_neg = 0 - sqrt_neg;
    ////float top_pos = (D1 - pz_prime + A4) + sqrt_pos;
    ////float top_neg = (D1 - pz_prime + A4) + sqrt_neg;
    ////bot = (cos(theta1) * px + sin(theta1) * py - A1) + (A3 * cos(theta3) + A2);
    ////float theta2_pos = (top_pos / bot);
    ////float theta2_neg = (top_neg / bot);
    ////// to change between pos and neg
    ////float theta2 = theta2_pos;

    ////float theta4 = (PI / 2) - theta2 - theta3;

    //std_msgs::Float64 baseMsg;
    //std_msgs::Float64 shoulderMsg;
    //std_msgs::Float64 elbowMsg;
    //std_msgs::Float64 wristPitchMsg;
    //std_msgs::Float64 wristRollMsg;

    //baseMsg.data        = theta1;
    //shoulderMsg.data    = theta2;
    //elbowMsg.data       = theta3;
    //wristPitchMsg.data  = theta4;
    //wristRollMsg.data   = theta5;

    //ROS_INFO_NAMED(NODE_NAME, "----- INPUTS -----");
    //ROS_INFO_NAMED(NODE_NAME, "px:    %f", px);
    //ROS_INFO_NAMED(NODE_NAME, "py:    %f", py);
    //ROS_INFO_NAMED(NODE_NAME, "pz:    %f", pz);
    //ROS_INFO_NAMED(NODE_NAME, "ax:    %f", ax);
    //ROS_INFO_NAMED(NODE_NAME, "ay:    %f", ay);
    //ROS_INFO_NAMED(NODE_NAME, "az:    %f", az);
    //ROS_INFO_NAMED(NODE_NAME, "pitch: %f", pitch);
    //ROS_INFO_NAMED(NODE_NAME, "roll:  %f", roll);
    //ROS_INFO_NAMED(NODE_NAME, "----- JOINTS -----");
    //ROS_INFO_NAMED(NODE_NAME, "Base:        %f", theta1 * 180 / PI);
    //ROS_INFO_NAMED(NODE_NAME, "Shoulder:    %f", theta2 * 180 / PI);
    //ROS_INFO_NAMED(NODE_NAME, "Elbow:       %f", theta3 * 180 / PI);
    //ROS_INFO_NAMED(NODE_NAME, "Wrist Pitch: %f", theta4 * 180 / PI);
    //ROS_INFO_NAMED(NODE_NAME, "Wrist Roll:  %f", theta5 * 180 / PI);

    //basePub.publish(            baseMsg);
    //shoulderPub.publish(    shoulderMsg);
    //elbowPub.publish(          elbowMsg);
    //wristPitchPub.publish(wristPitchMsg);
    //wristRollPub.publish(  wristRollMsg);
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
