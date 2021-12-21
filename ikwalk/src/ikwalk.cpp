#include "../include/ikwalk/ikwalk.hpp"

using namespace std;
run walk;


void timer_callback(const ros::TimerEvent&)
{
    static int ms_1 = 0;
    ms_1++;

    if(ms_1 >= param.freq)
    {
        t  += param.freq;

        ms_1 = 0;

        if(t >= param.ent)
        {
            t = 0;
        }
        walk.walk_run(param);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ikwalk");
    ros::NodeHandle n;

    //msg-----------------------------------------------------------------------------------
    ros::Subscriber master_sub = n.subscribe("master2ik", 100, master2ik_callback);
    ros::Subscriber set2ik_sub = n.subscribe("parameter", 100, set2ik_callback);
    IK.motor_pub = n.advertise<msg_generate::Motor_msg>("Dynamixel", 100);
    ikend_pub = n.advertise<msg_generate::ikend_msg>("ikend", 100);
    //Local
    walk.ikcoordinate_pub = n.advertise<msg_generate::ikcoordinate_msg>("ikcoordinate", 100);
    //Balancing
    imuSub = n.subscribe("imu", 100, imuCallback);
    balanceSub = n.subscribe("balance",100,balance_callback);
    //test
    test_pub = n.advertise<ikwalk::test1>("test", 100);
    //--------------------------------------------------------------------------------------

    //timer
    ros::Timer timer = n.createTimer(ros::Duration(0.01), timer_callback);  //10ms

    IK.init_save();
    sleep(1);
    get_parameters();

    IK.solve(0.0, param.y.default_y_r, param.z.default_z_r, 0.0, 0.0, param.y.default_y_l, param.z.default_z_l, 0.0, All);

    cout<<"default_y_r     "<<param.y.default_y_r<<endl;
    cout<<"default_z_r     "<<param.z.default_z_r<<endl;
    cout<<"default_y_l     "<<param.y.default_y_l<<endl;
    cout<<"default_z_l     "<<param.z.default_z_l<<endl;

    sleep(1);

    while(ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}


void run::walk_run(walk_parameters &param)
{
    if(_param.IKflag != IKflag_past)
    {
        if(_param.IKflag)//stop to start
            start_flag = true;

        else if(!_param.IKflag)//start to stop
            end_flag = true;
    }

    if(start_flag)
    {
        if(t == 0)
        {
            param.IKflag = true;
            start_cnt++;
        }
        if(start_cnt > 3)   // 1 swing 2  3
        {
            start_flag = false;
            start_cnt = 0;

            msg_generate::ikend_msg walkend;
            walkend.ikend = 0;
            ikend_pub.publish(walkend);
        }
    }

    else if(end_flag)
    {
        if(t == 0)
        {
            end_cnt--;
        }
        if(end_cnt < 1 && t == 0)   // 3 2 1 swing
        {
            end_cnt = 4;
            end_flag = false;

            param.IKflag = false;
            IKflag_past = false;
            IK.solve(0.0, param.y.default_y_r, param.z.default_z_r, 0.0, 0.0, param.y.default_y_l, param.z.default_z_l, 0.0, All);

            msg_generate::ikend_msg walkend;
            walkend.ikend = 1;
            ikend_pub.publish(walkend);
        }
    }

    if(param.IKflag)
    {
        generate_pattern(param);
    }
}


void run::generate_pattern(walk_parameters & param)
{
    param.S_support = 1.0 - param.D_support;

    //translate t(range 0 to ent) to t_r(range 0 to 1)==============================
    double ui_ent = balanceInfo.user_time_ent;
    cout << "ui_ent >>" << ui_ent << endl;


    if(balanceInfo.time_con_onoff==1)
    {
        cout<<"==============time control=================="<<endl;

        //Right
        if(imuInfo.roll > balanceInfo.time_pos && tctl_C_flag == 1)
        {
            if(walk.t_r > 0.5)
            {
                tctl_R_flag = 1;
                tctl_C_flag = 0;
                SR_flag = 1;
            }

            if(tctl_R_flag == 1)
            {
                cout << "Right_time_control!!!!!!!" << endl;

                if(linear_time_increase < balanceInfo.amp_time)
                {
                    linear_time_increase += balanceInfo.linear_increase_value;
                    _param.ent = ui_ent + linear_time_increase;
                    max_param = _param.ent;
                }
                else
                {
                    max_param = _param.ent;
                    linear_time_increase = balanceInfo.amp_time;
                    tctl_R_flag = 0;
                }
            }

            t_r = t/param.ent;
            t_l = t/param.ent+0.5;
            linear_time_decrease = balanceInfo.amp_time;

            cout << "linear_time_increase >> " << linear_time_increase << endl;
            cout << "linear_time_decrease >> " << linear_time_decrease << endl;
            cout << "_param.ent           >> " << _param.ent << endl;
            cout << "tctl_R_flag          >> " << tctl_R_flag << endl;
        }

        //Left
        else if(imuInfo.roll < balanceInfo.time_neg && tctl_C_flag == 1)
        {
            if(walk.t_l > 0.5)
            {
                tctl_L_flag = 1;
                tctl_C_flag = 0;
                SR_flag = 1;
            }

            if(tctl_L_flag == 1)
            {
                cout << "Left_time_control!!!!!!!!" << endl;

                if(linear_time_increase < balanceInfo.amp_time)
                {
                    linear_time_increase += balanceInfo.linear_increase_value;
                    _param.ent = ui_ent + linear_time_increase;
                    max_param = _param.ent;
                }
                else
                {
                    max_param = _param.ent;
                    linear_time_increase = balanceInfo.amp_time;
                    tctl_L_flag = 0;
                }
            }

            t_r = t/param.ent;
            t_l = t/param.ent+0.5;
            linear_time_decrease = balanceInfo.amp_time;
\
            cout << "linear_time_increase >> " << linear_time_increase << endl;
            cout << "linear_time_decrease >> " << linear_time_decrease << endl;
            cout << "_param.ent           >> " << _param.ent << endl;
            cout << "tctl_L_flag          >> " << tctl_L_flag << endl;
        }

        //normal
        else
        {
            linear_time_increase = 0.0;

            if(_param.ent > ui_ent )
            {
                if(t_r > 0.8)
                {
                    _param.ent = _param.ent - balanceInfo.linear_decrease_value;
                    cout << "????????" << endl;
                }
            }
            else
            {
                _param.ent = ui_ent;
                linear_time_decrease = 0.0;
                max_param = 0.0;

                tctl_C_flag = 1;
                SR_flag = 0;
            }

            cout << "max_param >>" << max_param << endl;
            cout << "linearcalueInfo >> " << balanceInfo.linear_decrease_value << endl;

            t_r = t/param.ent;
            t_l = t/param.ent+0.5;

            cout << "linear_time_decrease >>" << linear_time_decrease << endl;
            cout << "_param.ent >> " << _param.ent << endl;
            cout << " param.ent >> " << param.ent << endl;
            cout << "normal" << endl;


        }
        cout << "t_r >> " << t_r << endl;
        cout << "t_l >> " << t_l << endl;
        cout << "===========================================" << endl;
    }
    else
    {
        t_r = t/param.ent;
        t_l = t/param.ent+0.5;
    }

    if(t_l >= 1.0)
        t_l -= 1.0;

    //get parameter=================================================================

    if(t_r == 0)
    {
        param.y.swing_r = _param.y.swing_r;
        param.y.swing_l = _param.y.swing_l;
        param.y.default_y_l = _param.y.default_y_l;
        param.y.default_y_r = _param.y.default_y_r;

        param.y.swing_r = _param.y.swing_r;

        param.z = _param.z;
        param.yaw_r = _param.yaw_r;
        param.ent = _param.ent;


        if(start_cnt == 1)//swing
            param.ent = param.ent_START;
        else if(end_cnt == 1)//swing
            param.ent = param.ent_END;
        else
        {
            //msg to local
            msg_generate::ikcoordinate_msg ikcoordinate;
            ikcoordinate.X = param.x.X;
            ikcoordinate.Y = param.y.SIDE;
            ikcoordinate_pub.publish(ikcoordinate);
        }
    }
    else if(fabs(t_r - 0.5) <= 0.02)
    {
        param.yaw_l = _param.yaw_l;
    }
    else if(fabs(t_r - 0.25) <= 0.02 || fabs(t_r - 0.75) <= 0.02)
    {
        param.y.SIDE = _param.y.SIDE;
        param.y.tuning_SIDE = _param.y.tuning_SIDE;
        param.y.swing_SIDE_l = _param.y.swing_SIDE_l;
        param.y.swing_SIDE_r = _param.y.swing_SIDE_r;
        param.y.mul_SIDEr_r = _param.y.mul_SIDEr_r;
        param.y.mul_SIDEr_l = _param.y.mul_SIDEr_l;
        param.y.mul_SIDEl_r = _param.y.mul_SIDEl_r;
        param.y.mul_SIDEl_l = _param.y.mul_SIDEl_l;

        param.x = _param.x;

        if(start_cnt <= 3 && start_flag)
        {
            param.y.SIDE = 0/*((param.y.SIDE * (start_cnt - 1)) / 3.0)*/;
            param.x.X = ((param.x.X * (start_cnt - 1)) / 3.0);
        }
        else if(end_cnt > 1 && end_flag)
        {
            param.y.SIDE = 0/*((param.y.SIDE * (end_cnt - 1)) / 3.0)*/;
            param.x.X = ((param.x.X * (end_cnt - 1)) / 3.0);
            if(end_cnt == 2)
                param.x.X = 0.0;
        }
    }

    //Spline===========================================================================================
    // time    pos     vel     acc

    foot_trajectory start_COM_pattern;
    start_COM_pattern.put_point(0.0,  0.0,  -5.5,  0.0);
    start_COM_pattern.put_point(0.5,  -1.0,  0.0,  0.0);
    start_COM_pattern.put_point(1.0,  0.0,  5.0,  0.0);

    foot_trajectory start_RISE_pattern;
    start_RISE_pattern.put_point(0.0,  0.0,  0.0,  0.0);
    start_RISE_pattern.put_point(0.5,  1.0,  0.0,  0.0);
    start_RISE_pattern.put_point(1.0,  0.0,  0.0,  0.0);

    foot_trajectory COM_pattern;
    COM_pattern.put_point(0.0,  0.0,  5.0,  0.0);
    COM_pattern.put_point(0.25,  1.0,  0.0,  0.0);
    COM_pattern.put_point(0.5,  0.0,  -5.5,  0.0);
    COM_pattern.put_point(0.75,  -1.0,  0.0,  0.0);
    COM_pattern.put_point(1.0,  0.0,  5.0,  0.0);

    foot_trajectory STEP_pattern;
    STEP_pattern.put_point(0.0,  -0.5,  0.0,  0.0);
    STEP_pattern.put_point((param.D_support/4.0),  -0.5,  0.0,  0.0);
    STEP_pattern.put_point(((param.D_support/4.0)+(param.S_support/2.0)),  0.5,  0.0,  0.0);
    STEP_pattern.put_point((((param.D_support*3.0)/4.0)+(param.S_support/2.0)),  0.5,  0.0,  0.0);
    STEP_pattern.put_point((((param.D_support*3.0)/4.0)+(param.S_support)),  -0.5,  0.0,  0.0);
    STEP_pattern.put_point(1.0,  -0.5,  0.0,  0.0);

    foot_trajectory RISE_pattern;
    RISE_pattern.put_point(0.0,  0.0,  0.0,  0.0);
    RISE_pattern.put_point((param.D_support/4.0),  0.0,  0.0,  0.0);
    RISE_pattern.put_point(((param.D_support+param.S_support)/4.0),  1.0,  0.0,  0.0);
    RISE_pattern.put_point(((param.D_support/4.0)+(param.S_support/2.0)),  0.0,  0.0,  0.0);
    RISE_pattern.put_point(0.5,   0.0,    0.0,    0.0);
    RISE_pattern.put_point(1.0,   0.0,    0.0,    0.0);

    foot_trajectory TURN_pattern;
    TURN_pattern.put_point(0.0,  0.0,  0.0,  0.0);
    TURN_pattern.put_point((param.D_support/4.0),  0.0,  0.0,  0.0);
    TURN_pattern.put_point(((param.D_support/4.0)+(param.S_support/2.0)),  -1.0,  0.0,  0.0);
    TURN_pattern.put_point((((param.D_support*3.0)/4.0)+(param.S_support/2.0)),  -1.0,  0.0,  0.0);
    TURN_pattern.put_point((((param.D_support*3.0)/4.0)+(param.S_support)),  0.0,  0.0,  0.0);
    TURN_pattern.put_point(1.0,  0.0,  0.0,  0.0);
    //==================================================================================================

    if(start_cnt == 1)
    {
        //RIGHT-------------------------------------------------------------------------------------
        pX_r = param.x.default_x_r;

        pY_r = -start_COM_pattern.result(t_r)*(param.y.swing_START) + param.y.default_y_r;

        pZ_r = param.z.default_z_r;

        RYaw = 0.0;

        //LEFT--------------------------------------------------------------------------------------
        pX_l = param.x.default_x_l;

        pY_l = -start_COM_pattern.result(t_r)*(param.y.swing_START) + param.y.default_y_l;

        cout<<"param.z.rise_START  == "<<param.z.rise_START<<endl;
        pZ_l = start_RISE_pattern.result(t_r)*(param.z.rise_START) + param.z.default_z_l;

        LYaw = 0.0;
    }
    else if(end_cnt == 1)
    {
        //RIGHT-------------------------------------------------------------------------------------
        pX_r = param.x.default_x_r;

        pY_r = start_COM_pattern.result(t_r)*(param.y.swing_END) + param.y.default_y_r;

        pZ_r = start_RISE_pattern.result(t_r)*(param.z.rise_END) + param.z.default_z_r;

        RYaw = 0.0;

        //LEFT--------------------------------------------------------------------------------------
        pX_l = param.x.default_x_l;

        pY_l = start_COM_pattern.result(t_r)*(param.y.swing_END) + param.y.default_y_l;

        pZ_l = param.z.default_z_l;

        LYaw = 0.0;
    }
    else
    {
        //RIGHT-------------------------------------------------------------------------------------

        if(balanceInfo.time_con_onoff==1 && SR_flag == 1)
        {
            pX_r = STEP_pattern.result(t_r)*(param.x.tuning_X + param.x.X) + param.x.default_x_r;

            pY_r = -COM_pattern.result(t_r)*(param.y.swing_r - 10) + param.y.default_y_r;
            pY_r += (param.y.tuning_SIDE + param.y.SIDE)*STEP_pattern.result(t_r);

            pZ_r = RISE_pattern.result(t_r)*(param.z.rise_r - 20) + param.z.default_z_r;

            RYaw = TURN_pattern.result(t_r)*(param.yaw_r.tuning_yaw + param.yaw_r.YAW);

            cout << "R_succecss !!!!!!!" << endl;
        }
        else
        {
            pX_r = STEP_pattern.result(t_r)*(param.x.tuning_X + param.x.X) + param.x.default_x_r;

            pY_r = -COM_pattern.result(t_r)*(param.y.swing_r) + param.y.default_y_r;
            pY_r += (param.y.tuning_SIDE + param.y.SIDE)*STEP_pattern.result(t_r);

            pZ_r = RISE_pattern.result(t_r)*(param.z.rise_r) + param.z.default_z_r;

            RYaw = TURN_pattern.result(t_r)*(param.yaw_r.tuning_yaw + param.yaw_r.YAW);
        }


        //LEFT--------------------------------------------------------------------------------------

        if(balanceInfo.time_con_onoff==1 && SR_flag == 1)
        {
            pX_l = STEP_pattern.result(t_l)*(param.x.tuning_X + param.x.X) + param.x.default_x_l;

            pY_l = -COM_pattern.result(t_r)*(param.y.swing_l- 10) + param.y.default_y_l;
            pY_l += (param.y.tuning_SIDE + param.y.SIDE)*STEP_pattern.result(t_l);

            pZ_l = RISE_pattern.result(t_l)*(param.z.rise_l- 20) + param.z.default_z_l;

            LYaw = TURN_pattern.result(t_l)*(param.yaw_l.tuning_yaw + param.yaw_l.YAW);
            cout << "L_succecss !!!!!!!" << endl;
        }
        else
        {
            pX_l = STEP_pattern.result(t_l)*(param.x.tuning_X + param.x.X) + param.x.default_x_l;

            pY_l = -COM_pattern.result(t_r)*(param.y.swing_l) + param.y.default_y_l;
            pY_l += (param.y.tuning_SIDE + param.y.SIDE)*STEP_pattern.result(t_l);

            pZ_l = RISE_pattern.result(t_l)*(param.z.rise_l) + param.z.default_z_l;

            LYaw = TURN_pattern.result(t_l)*(param.yaw_l.tuning_yaw + param.yaw_l.YAW);
        }

        cout << "SR_flag >> " << SR_flag << endl;
    }

    //=================================================================================================
//    cout<<endl;
//    cout<<"=====                   ent   "<<param.ent<<endl;
//    cout<<"=====                  freq   "<<param.freq<<endl;
//    cout<<"=====                    t    "<<t<<endl;
//    cout<<"=====                   t_r   "<<t_r<<endl;
//    cout<<"=====                   t_l   "<<t_l<<endl;
//    cout<<"################pX_r#######   "<<pX_r<<endl;
//    cout<<"################pX_l#######   "<<pX_l<<endl;
//    cout<<"################pY_r#######   "<<pY_r<<endl;
//    cout<<"################pY_l#######   "<<pY_l<<endl;
//    cout<<"################pZ_r#######   "<<pZ_r<<endl;
//    cout<<"################pZ_l#######   "<<pZ_l<<endl;
//    cout<<"################Yaw_r######   "<<RYaw<<endl;
//    cout<<"################Yaw_l######   "<<LYaw<<endl;
//    cout<<endl;
    //=================================================================================================

    //Balancing
    if(balanceInfo.pid_onoff==1)
    {
        cout <<"-==============pid control===================" << endl;
        cout << "PITCH_ADD_Angle >> " << Pitch_ADD_Angle << endl;
        IK_P.PD_Pitch_control(imuInfo.pitch,balanceInfo.pitch_GP,balanceInfo.pitch_GI,balanceInfo.pitch_GD,balanceInfo.pitch_ELIMIT,balanceInfo.pitch_OLIMIT,balanceInfo.pitch_neg_Target,balanceInfo.pitch_pos_Target,imuInfo.pitch_acc);
        cout<<"=============================================="<<endl;
    }
    else
    {
        Pitch_ADD_Angle = 0;
    }

    if(balanceInfo.pid_roll_onoff==1)
    {
          cout<<"==============roll control=================="<<endl;
          cout << "ROLL_ADD_Angle >> " << Roll_ADD_Angle << endl;
          IK_P.PD_Roll_control(imuInfo.roll,balanceInfo.roll_GP, balanceInfo.roll_GI,balanceInfo.roll_GD,balanceInfo.roll_ELIMIT,balanceInfo.roll_OLIMIT,balanceInfo.roll_neg_Target,balanceInfo.roll_pos_Target);
          cout<<"============================================"<<endl;
    }
    else
    {
        Roll_ADD_Angle = 0;
    }


    //Limit........................................................
    if(pY_r < -130) pY_r = -130;
    if(pY_l > 130) pY_l = 130;


    //send to Solve--------------------------------------------------
    if(param.IKflag)
        IK.solve(pX_r, pY_r, pZ_r, RYaw, pX_l, pY_l, pZ_l, LYaw, Leg);

    IKflag_past = param.IKflag;

    //test=============================================================================================
    ikwalk::test1 _test;

    _test.time = t_r;
    _test.x_r = pX_r;
    _test.y_r = pY_r;
    _test.z_r = pZ_r;
    _test.yaw_r = RYaw;
    _test.x_l = pX_l;
    _test.y_l = pY_l;
    _test.z_l = pZ_l;
    _test.yaw_l = LYaw;

    //rqt_plot
    test_pub.publish(_test);
    //=================================================================================================
}



void get_parameters()
{
    std::ifstream is;

    is.open("/home/robit/catkin_ws/src/set_walk/work/1");

    is >> _param.ent;
    param.ent = _param.ent;
    is >> _param.freq;
    is >> _param.D_support;
    is >> _param.S_support;

    is >> _param.x.tuning_X;
    is >> _param.y.tuning_SIDE;
    is >> _param.yaw_r.tuning_yaw;

    is >> _param.x.default_x_r;
    is >> _param.x.default_x_l;
    is >> _param.y.default_y_r;
    is >> _param.y.default_y_l;
    is >> _param.z.default_z_r;
    is >> _param.z.default_z_l;

    is >> _param.y.swing_r;
    is >> _param.y.swing_l;
    is >> _param.z.rise_r;
    is >> _param.z.rise_l;

    is >> IK._ang.ang10;
    is >> IK._ang.ang11;
    is >> IK._ang.ang12;
    is >> IK._ang.ang13;
    is >> IK._ang.ang14;
    is >> IK._ang.ang15;
    is >> IK._ang.ang16;
    is >> IK._ang.ang17;
    is >> IK._ang.ang18;
    is >> IK._ang.ang19;
    is >> IK._ang.ang20;
    is >> IK._ang.ang21;

    is >> param.ent_START;
    is >> param.ent_END;
    is >> param.y.swing_START;
    is >> param.y.swing_END;
    is >> param.z.rise_START;
    is >> param.z.rise_END;

    is >> _param.x.PA_X_f;
    is >> _param.x.PA_X_b;

    is >> _param.y.swing_SIDE_r;
    is >> _param.y.swing_SIDE_l;

    is >> _param.y.mul_SIDEr_r;
    is >> _param.y.mul_SIDEr_l;
    is >> _param.y.mul_SIDEl_r;
    is >> _param.y.mul_SIDEl_l;

    is >> _param.yaw_r.PA_YAW_ccw;
    is >> _param.yaw_r.X_YAW_ccw;
    is >> _param.yaw_r.PA_YAW_cw;
    is >> _param.yaw_r.X_YAW_cw;

    is >> _param.shoulder_swing;


    _param.yaw_r = _param.yaw_l;

    cout<<"param.z.rise_r == "<<param.z.rise_r<<endl;
    cout<<"param.z.rise_l == "<<param.z.rise_l<<endl;
    cout<<"param.ent == "<<param.ent<<endl;

    IK.ang = IK._ang;


    is.close();
}


//Callback================================================================================
void master2ik_callback(const msg_generate::ik_msg::ConstPtr& msg)
{
    _param.IKflag = msg->flag;

    _param.x.X = msg->X_length;
    _param.y.SIDE = msg->Y_length;
    _param.yaw_r.YAW = msg->Yaw;
    _param.yaw_l.YAW = msg->Yaw;

    //Tuning--------------------------------------------------------------------------------------------
    //X
    if(_param.x.X > 0) // front walk
    {
        IK.ang.ang14 = IK._ang.ang14 - (_param.x.X * _param.x.PA_X_f);
        IK.ang.ang15 = IK._ang.ang15 -(_param.x.X * _param.x.PA_X_f);
    }
    else if(_param.x.X < 0) // back walk
    {
        IK.ang.ang14 = IK._ang.ang14 - (_param.x.X * _param.x.PA_X_b);
        IK.ang.ang15 = IK._ang.ang15 -(_param.x.X * _param.x.PA_X_b);
    }
    //SIDE
    mul_r = 1;        mul_l = 1;
    mulswing_r = 0;   mulswing_l = 0;

    if(_param.y.SIDE > 0) // left side walk
    {
        mul_r = _param.y.mul_SIDEl_r;
        mul_l = _param.y.mul_SIDEl_l;
        if(walk.t_r < 0.5)
            mulswing_l = _param.y.SIDE * _param.y.swing_SIDE_l;
    }
    else if(_param.y.SIDE<0) // right side walk
    {
        mul_r = _param.y.mul_SIDEr_r;
        mul_l = _param.y.mul_SIDEr_l;
        if(walk.t_r > 0.5)
            mulswing_r = -(_param.y.SIDE * _param.y.swing_SIDE_r);
    }
    //YAW
    if(_param.yaw_r.YAW > 0) // ccw walk
    {
        IK.ang.ang14 = IK._ang.ang14 + (_param.yaw_r.YAW * _param.yaw_r.PA_YAW_ccw);
        IK.ang.ang15 = IK._ang.ang15 + (_param.yaw_r.YAW * _param.yaw_r.PA_YAW_ccw);

        _param.x.X -= (_param.yaw_r.YAW * _param.yaw_r.X_YAW_ccw);
    }
    else if(_param.yaw_r.YAW < 0) // cw walk
    {
        IK.ang.ang14 = IK._ang.ang14 - (_param.yaw_r.YAW * _param.yaw_r.PA_YAW_cw);
        IK.ang.ang15 = IK._ang.ang14 - (_param.yaw_r.YAW * _param.yaw_r.PA_YAW_cw);

        _param.x.X += (_param.yaw_r.YAW * _param.yaw_r.PA_YAW_cw);
    }
    //--------------------------------------------------------------------------------------------------
}

void set2ik_callback(const set_walk::IKparameter::ConstPtr& msg)
{
    _param.IKflag = msg->IKflag;

    //time
    _param.ent = msg->ent;
    param.freq = msg->freq;

    //support ratioa
    param.D_support = msg->d_support;
    param.S_support = msg->s_support;

    //x_value
    _param.x.X = msg->test_x;
    _param.x.tuning_X = msg->tuning_x;
    _param.x.default_x_r = msg->df_x_r;
    _param.x.default_x_l = msg->df_x_l;

    cout<<"_param.x.X    =====   "<<_param.x.X<<endl;
    cout<<"_param.x.tuning_X ==== "<<_param.x.tuning_X<<endl;
    //y value
    _param.y.SIDE = msg->test_side;
    _param.y.tuning_SIDE = msg->tuning_side;
    _param.y.default_y_r = msg->df_y_r;
    _param.y.default_y_l = msg->df_y_l;
    _param.y.swing_r = msg->swing_r;
    _param.y.swing_l = msg->swing_l;

    //z value
    _param.z.default_z_r = msg->df_z_r;
    _param.z.default_z_l = msg->df_z_l;
    _param.z.rise_r = msg->rise_r;
    _param.z.rise_l = msg->rise_l;

    //yaw r value
    _param.yaw_r.YAW = msg->test_yaw;
    _param.yaw_r.tuning_yaw = msg->tuning_yaw;

    //yaw l value
    _param.yaw_l.YAW = msg->test_yaw;
    _param.yaw_l.tuning_yaw = msg->tuning_yaw;

    IK.ang.ang10 = msg->ang10;
    IK.ang.ang11 = msg->ang11;
    IK.ang.ang12 = msg->ang12;
    IK.ang.ang13 = msg->ang13;
    IK.ang.ang14 = msg->ang14;
    IK.ang.ang15 = msg->ang15;
    IK.ang.ang16 = msg->ang16;
    IK.ang.ang17 = msg->ang17;
    IK.ang.ang18 = msg->ang18;
    IK.ang.ang19 = msg->ang19;
    IK.ang.ang20 = msg->ang20;
    IK.ang.ang21 = msg->ang21;

    param.ent_START = msg->ent_START;
    param.y.swing_START = msg->swing_START;
    _param.z.rise_START = msg->rise_START;
    cout<< "sibalsialb     "<<param.z.rise_START<<endl;
    param.ent_END = msg->ent_END;
    param.y.swing_END = msg->swing_END;
    _param.z.rise_END = msg->rise_END;

    _param.x.PA_X_f = msg->PA_x_f;
    _param.x.PA_X_b = msg->PA_x_b;

    _param.y.swing_SIDE_r = msg->swing_SIDE_r;
    _param.y.swing_SIDE_l = msg->swing_SIDE_l;
    _param.y.mul_SIDEr_r = msg->mul_SIDEr_r;
    _param.y.mul_SIDEr_l = msg->mul_SIDEr_l;
    _param.y.mul_SIDEl_r = msg->mul_SIDEl_r;
    _param.y.mul_SIDEl_l = msg->mul_SIDDl_l;

    _param.yaw_r.PA_YAW_ccw = msg->PA_YAW_l;
    _param.yaw_r.X_YAW_ccw = msg->X_YAW_l;
    _param.yaw_l.PA_YAW_ccw = msg->PA_YAW_l;
    _param.yaw_l.X_YAW_ccw = msg->X_YAW_l;

    _param.yaw_r.PA_YAW_cw = msg->PA_YAW_r;
    _param.yaw_r.X_YAW_cw = msg->X_YAW_r;
    _param.yaw_l.PA_YAW_cw = msg->PA_YAW_r;
    _param.yaw_l.X_YAW_cw = msg->X_YAW_r;


    //Tuning--------------------------------------------------------------------------------------------
    //X
    if(_param.x.X > 0) // front walk
    {
        IK.ang.ang14 = IK.ang.ang14 - (_param.x.X * _param.x.PA_X_f);
        IK.ang.ang15 = IK.ang.ang15 -(_param.x.X * _param.x.PA_X_f);
    }
    else if(_param.x.X < 0) // back walk
    {
        IK.ang.ang14 = IK.ang.ang14 - (_param.x.X * _param.x.PA_X_b);
        IK.ang.ang15 = IK.ang.ang15 -(_param.x.X * _param.x.PA_X_b);
    }
    //SIDE
    mul_r = 1;        mul_l = 1;
    mulswing_r = 0;   mulswing_l = 0;

    if(_param.y.SIDE > 0) // left side walk
    {
        mul_r = _param.y.mul_SIDEl_r;
        mul_l = _param.y.mul_SIDEl_l;
        if(walk.t_r < 0.5)
            mulswing_l = _param.y.SIDE * _param.y.swing_SIDE_l;
    }
    else if(_param.y.SIDE<0) // right side walk
    {
        mul_r = _param.y.mul_SIDEr_r;
        mul_l = _param.y.mul_SIDEr_l;
        if(walk.t_r > 0.5)
            mulswing_r = -(_param.y.SIDE * _param.y.swing_SIDE_r);
    }
    //YAW
    if(_param.yaw_r.YAW > 0) // ccw walk
    {
        IK.ang.ang14 = IK.ang.ang14 + (_param.yaw_r.YAW * _param.yaw_r.PA_YAW_ccw);
        IK.ang.ang15 = IK.ang.ang15 + (_param.yaw_r.YAW * _param.yaw_r.PA_YAW_ccw);

        _param.x.X -= (_param.yaw_r.YAW * _param.yaw_r.X_YAW_ccw);
    }
    else if(_param.yaw_r.YAW < 0) // cw walk
    {
        IK.ang.ang14 = IK.ang.ang14 - (_param.yaw_r.YAW * _param.yaw_r.PA_YAW_cw);
        IK.ang.ang15 = IK.ang.ang14 - (_param.yaw_r.YAW * _param.yaw_r.PA_YAW_cw);

        _param.x.X += (_param.yaw_r.YAW * _param.yaw_r.PA_YAW_cw);
    }

    cout<<"mul_r = "<<mul_r<<endl;
    cout<<"mul_l = "<<mul_l<<endl;
    //--------------------------------------------------------------------------------------------------
}

//Balancing
void imuCallback(const msg_generate::imu_msg::ConstPtr &msg)
{
    imuInfo.roll = msg->roll;
    imuInfo.pitch = msg->pitch;
    imuInfo.yaw = msg->yaw;
    imuInfo.roll_acc = msg->roll_acc;
    imuInfo.pitch_acc = msg->pitch_acc;
    imuInfo.yaw_acc = msg->yaw_acc;
}
void balance_callback(const set_walk::balance::ConstPtr &msg)
{
    balanceInfo.pitch_GP                 = msg->pitch_GP;
    balanceInfo.pitch_GI                 = msg->pitch_GI;
    balanceInfo.pitch_GD                 = msg->pitch_GD;
    balanceInfo.pitch_ELIMIT             = msg->pitch_ELIMIT;
    balanceInfo.pitch_OLIMIT             = msg->pitch_OLIMIT;
    balanceInfo.pitch_neg_Target         = msg->pitch_neg_Target;
    balanceInfo.pitch_pos_Target         = msg->pitch_pos_Target;
    balanceInfo.roll_GP                  = msg->roll_GP;
    balanceInfo.roll_GI                  = msg->roll_GI;
    balanceInfo.roll_GD                  = msg->roll_GD;
    balanceInfo.roll_ELIMIT              = msg->roll_ELIMIT;
    balanceInfo.roll_OLIMIT              = msg->roll_OLIMIT;
    balanceInfo.roll_neg_Target          = msg->roll_neg_Target;
    balanceInfo.roll_pos_Target          = msg->roll_pos_Target;
    balanceInfo.pid_onoff                = msg->pid_onoff;
    balanceInfo.pelvic_amp               = msg->pelvic_amp;
    balanceInfo.knee_amp                = msg->knee_amp;
    balanceInfo.ankle_amp                = msg->ankle_amp;
    balanceInfo.roll_pelvic_amp          = msg->roll_pelvic_amp;
    balanceInfo.roll_ankle_amp           = msg->roll_ankle_amp;
    balanceInfo.pid_roll_onoff           = msg->pid_roll_onoff;
    balanceInfo.amp_time                 = msg->amp_time;
    balanceInfo.time_pos                 = msg->time_neg;
    balanceInfo.time_neg                 = msg->time_pos;
    balanceInfo.time_con_onoff           = msg->time_con_onoff;
    balanceInfo.linear_increase_value    = msg->linear_increase_value;
    balanceInfo.linear_decrease_value    = msg->linear_decrease_value;
    balanceInfo.user_time_ent            = msg->user_time_ent;
}
