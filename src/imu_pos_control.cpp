#include "../include/ikwalk/imu_pos_control.hpp"
#include "../include/ikwalk/ikwalk.hpp"

//2019.5.09 pitch_control_success
//2019.5.10 roll_control_try_01

double Knee_Add_Angle   = 0;
double pos_pitch_OLIMIT = 0;
double neg_pitch_OLIMIT = 0;

int Push_re_front = 0;
int Push_re_back  = 0;

extern int Push_re_balance_flag;
extern int Push_re_cnt_flag;

using namespace std;

void IK_pos_control::cout_imu_data(float receive_r,float receive_p,float receive_y)
{
    //cout << receive_r << endl;
    //cout << receive_p << endl;
    //cout << receive_y << endl;
}

void IK_pos_control::PD_Pitch_control(double input,double pitch_GP,double pitch_GI,double pitch_GD,double pitch_ELIMIT,double pitch_OLIMIT,double pitch_neg_Target,double pitch_pos_Target,double pitch_acc)
{
    if(pitch_pos_Target >= input && pitch_neg_Target <= input)
    {
        pitch_OLIMIT = 1;
    }

    PID_Control_init(&pitch_control, pitch_GP, pitch_GI, pitch_GD, pitch_ELIMIT, pitch_OLIMIT);

    //--------------balance---------------
    if(input > pitch_pos_Target)
    {
        PID_Control_Float(&pitch_control, pitch_pos_Target, input);
        Pitch_ADD_Angle = pitch_control.nowOutput;

        //balance
        if(Push_re_cnt_flag == 1 && Impact_flag == 1)
        {
            Push_re_front        = 0;
            Push_re_back         = 1;
        }
    }
    else if(input < pitch_neg_Target)
    {
        PID_Control_Float(&pitch_control, pitch_neg_Target, input);
        Pitch_ADD_Angle = pitch_control.nowOutput;

        //balance
        if(Push_re_cnt_flag == 1 && Impact_flag == 1)
        {
            Push_re_front        = 1;
            Push_re_back         = 0;
        }
    }
    else
    {
        Pitch_ADD_Angle = 1;
    }

//    cout << "====================================="                   << endl;
//    cout << "pitch_pos_Target        >> " << pitch_pos_Target         << endl;
//    cout << "pitch_neg_Target        >> " << pitch_neg_Target         << endl;
//    cout << "pitch_Olimit            >> " << pitch_OLIMIT             << endl;
//    cout << "-------------------------------------"                   << endl;
//    cout << "picch_pos_Target + 5    >> " << (pitch_pos_Target + 5.0) << endl;
//    cout << "pitch_neg_Target - 5    >> " << (pitch_neg_Target - 5.0) << endl;
//    cout << "now_pitch_value         >> " << input                    << endl;
//    cout << "Push_re_front           >> " << Push_re_front            << endl;
//    cout << "Push_re_back            >> " << Push_re_back             << endl;
//    cout << "====================================="                   << endl;

    ikwalk::test1 _test;
    _test.x_l = input;
    _test.x_r = pitch_pos_Target;
    _test.yaw_l = pitch_neg_Target;
    test_pub.publish(_test);
}

void IK_pos_control::PD_Roll_control(double input,double roll_GP,double roll_GI,double roll_GD,double roll_ELIMIT,double roll_OLIMIT,double roll_neg_Target,double roll_pos_Target)
{

    if(roll_pos_Target >= input && roll_neg_Target <= input)
    {
        roll_OLIMIT = 1;
    }

    PID_Control_init(&roll_control,roll_GP, roll_GI, roll_GD, roll_ELIMIT, roll_OLIMIT);

    if(input > roll_pos_Target)
    {
        PID_Control_Float(&roll_control, roll_pos_Target, input);
        Roll_ADD_Angle = roll_control.nowOutput;
    }
    else if(input < roll_neg_Target)
    {
        PID_Control_Float(&roll_control, roll_neg_Target, input);
        Roll_ADD_Angle = roll_control.nowOutput;
    }
    else
    {
        Roll_ADD_Angle = 1;
    }

//    cout << "====================================="                 << endl;
//    cout << "roll_pos_Target     >>"      << roll_pos_Target        << endl;
//    cout << "roll_neg_Target     >> "     << roll_neg_Target        << endl;
//    cout << "roll_control.input     >> "  << input                  << endl;
//    cout << "roll_control.nowoutput >> "  << roll_control.nowOutput << endl;
//    cout << "pos_Target // input     >>"  << input                  << endl;
//    cout << "pos_Target // nowOutput >> " << roll_control.nowOutput << endl;
//    cout << "neg_Target // input     >>"  << input                  << endl;
//    cout << "neg_Target // nowOutput >> " << roll_control.nowOutput << endl;
//    cout << "====================================="                 << endl;
}


