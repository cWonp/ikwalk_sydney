#ifndef IKWALK_HPP_
#define IKWALK_HPP_

#include "ros/ros.h"
#include <iostream>
#include "cmath"
#include <fstream>

#include "walkpattern.hpp"
#include "solve.hpp"
#include "imu_pos_control.hpp"
#include "pid_control_float.h"

//msg header=====================================================
#include "IKparameter.h"
#include "test1.h"
#include <msg_generate/ikcoordinate_msg.h>
#include <msg_generate/Motor_msg.h>
#include <msg_generate/ik_msg.h>
#include <msg_generate/ikend_msg.h>
//Balancing.........................
#include <msg_generate/imu_msg.h>
#include "balance.h"
//msg============================================================
ros::Publisher test_pub;
ros::Publisher ikend_pub;
//Balancing.........................
ros::Subscriber imuSub;
ros::Subscriber balanceSub;
//var============================================================

msg_generate::imu_msg imuInfo;
set_walk::balance balanceInfo;


IK_pos_control IK_P;
IK_solve IK;


double t = 0; //timer time
double t_2 = 0; //Timing_control

//SIDE Tuning var
double mul_r = 1.0;
double mul_l = 1.0;
double mulswing_r = 1.0;
double mulswing_l = 1.0;

double linear_time_decrease;
double linear_time_increase;

double max_param;
double min_param;

int tctl_R_flag = 0;
int tctl_L_flag = 0;
int tctl_C_flag = 0;

int SR_flag = 0;

int flag = 0;

struct humanoid
{
    double l_0 = 60.5;
    double l_2 = 170.0;
    double init_z_up = 8.0;
};

humanoid model;

class walk_parameters
{
public:
    //IKflag---------------------------------------------------------
    bool IKflag = false;

    //control time(10ms)----------------------------------------------
    double ent = 130.0;
    double freq = 1.0;
    //START END
    double ent_START = 0.0; //**
    double ent_END = 0.0; //**

    //support ratio--------------------------------------------------
    double D_support = 0.0; //double support
    double S_support = 1.0; //single support

    struct x_value//-------------------------------------------------
    {
        //x value
        double X = 0.0;
        //tuning x
        double tuning_X = 0.0;

        double PA_X_f = 1.0;//**
        double PA_X_b = 1.0;//**

        //default x
        double default_x_r = 0.0;
        double default_x_l = 0.0;
    };
    struct y_value//-------------------------------------------------
    {
        //side value
        double SIDE = 0.0;
        //tuning side
        double tuning_SIDE = 0.0;

        double swing_SIDE_r = 0.0;//**
        double swing_SIDE_l = 0.0;//**
        double mul_SIDEr_r = 0.0;//**
        double mul_SIDEr_l = 0.0;//**
        double mul_SIDEl_r = 0.0;//**
        double mul_SIDEl_l = 0.0;//**

        //default y
        double default_y_r = -(model.l_0);
        double default_y_l = model.l_0;
        //Swing magnitude
        double swing_r = 0.0;
        double swing_l = 0.0;
        double swing_START = 0.0;//**
        double swing_END = 0.0;//**
    };
    struct z_value//-------------------------------------------------
    {
        //default z
        double default_z_r = -(model.l_2+model.l_2-model.init_z_up);
        double default_z_l = -(model.l_2+model.l_2-model.init_z_up);
        //z magnitude
        double rise_r = 0.0;
        double rise_l = 0.0;
        //START END
        double rise_START = 0.0;//**
        double rise_END = 0.0;//**
    };
    struct yaw_value//-------------------------------------------------
    {
        //yaw_value
        double YAW = 0.0;
        //tuning yaw
        double tuning_yaw = 0.0;
        double PA_YAW_ccw = 0.0;//**
        double X_YAW_ccw = 0.0;//**
        double PA_YAW_cw = 0.0;//**
        double X_YAW_cw = 0.0;//**
    };

    double shoulder_swing = 0.0;//**

    //---------------------------------------------------------------
    x_value x;
    y_value y;
    z_value z;
    yaw_value yaw_r;
    yaw_value yaw_l;
};

walk_parameters param;
walk_parameters _param;

//func========================================================
//Callback....................................................
void master2ik_callback(const msg_generate::ik_msg::ConstPtr& msg);
void set2ik_callback(const set_walk::IKparameter::ConstPtr& msg);
//Balancing
void imuCallback(const msg_generate::imu_msg::ConstPtr&);
void balance_callback(const set_walk::balance::ConstPtr&msg);
//...........................................................
void get_parameters();
//============================================================


class run
{
public:

  void walk_run(walk_parameters & param);
  void generate_pattern(walk_parameters & param);

  ros::Publisher ikcoordinate_pub;

  //time range 0 to 1
  double t_r = 0.0;
  double t_l = 0.0;
  double t_l_2 = 0.0;

private:

  //start and end of walk
  int IKflag_past = 0;
  int start_cnt = 0;
  int end_cnt = 4;

  bool start_flag = false;
  bool end_flag = false;

  bool startvar = false;
  bool endvar = false;

  double pX_r;
  double pY_r;
  double pZ_r;
  double RYaw;
  double pX_l;
  double pY_l;
  double pZ_l;
  double LYaw;
};
#endif /* IKWALK_HPP_ */
