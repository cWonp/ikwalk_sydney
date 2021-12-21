#ifndef SOLVE_HPP_
#define SOLVE_HPP_

#include <iostream>
#include <fstream>
#include <cmath>
#include "ros/ros.h"

//msg---------------------------------------------
#include <msg_generate/Motor_msg.h>
#include "imu_pos_control.hpp"
#include "pid_control_float.h"


#define PI 3.141592653589793
#define deg2rad PI/(double)180
#define rad2deg (double)180.0/PI

#define Leg 10
#define All 0

using namespace std;

class IK_solve
{
public:

    ros::Publisher motor_pub;

    void solve(double pX_r, double pY_r, double pZ_r, double RYaw, double pX_l, double pY_l, double pZ_l, double LYaw, int body);
    double ang2pos(double angle);
    void init_save();
    void motor_packet(int body, int limit);

    struct motor_ang
    {
        //1 ~ 4096
        //pitch
        int ang14 = 0;
        int ang15 = 0;

        int ang16 = 0;
        int ang17 = 0;

        int ang18 = 0;
        int ang19 = 0;

        //yaw
        int ang12 = 0;
        int ang13 = 0;

        //roll
        int ang20 = 0;
        int ang21 = 0;

        int ang10 = 0;
        int ang11 = 0;
    };
    motor_ang ang;
    motor_ang _ang;

    int now_15_Angle_value;
    int now_16_Angle_value;

private:

    unsigned int g_DXL_ID_position[30] = {0, }; //output
    unsigned int g_DXL_ID_Save_position[30] = {0, };
    double InvLegAngle[12] = {0, };

    int Motion_Data[400] = {0, };

    bool ikinit_flag = true;
};


#endif /* SOLVE_HPP_ */
