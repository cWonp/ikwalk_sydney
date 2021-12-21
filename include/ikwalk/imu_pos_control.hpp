#ifndef IMU_POS_CONTROL_HPP_
#define IMU_POS_CONTROL_HPP_

#include <iostream>
#include <fstream>
#include <cmath>
#include "pid_control_float.h"

PID pitch_control;
PID roll_control;

using namespace std;

double Pitch_ADD_Angle = 0;
double Roll_ADD_Angle = 0;
double Push_re_pos_target = 18.0;
double Push_re_neg_target = 6.0;

double R_Ankle_horizon_value = 0;
double L_Ankle_horizon_value = 0;

class IK_pos_control
{
public:

    void cout_imu_data(float receive_r,float receive_p,float receive_y);
    void PD_Pitch_control(double input,double pitch_GP,double pitch_GI,double pitch_GD,double pitch_ELIMIT,double pitch_OLIMIT,double pitch_neg_Target,double pitch_pos_Target,double pitch_acc);
    void PD_Roll_control(double input,double roll_GP,double roll_GI,double roll_GD,double roll_ELIMIT,double roll_OLIMIT,double roll_neg_Target,double roll_pos_Target);

private:

};

#endif /* IMU_POS_CONTROL_HPP_ */
