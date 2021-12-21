#include "../include/ikwalk/solve.hpp"
#include "../include/ikwalk/ikwalk.hpp"

extern int Push_Recovery_mode;
extern int Push_re_front;
extern int Push_re_back;

extern int Left_Period;
extern int Right_Period;

double IK_solve::ang2pos(double angle)
{
    return (double)((angle * 4096.0) / 360.0);
}

void IK_solve::init_save()
{
    std::string addr;

    addr = "/home/robit/catkin_ws/src/ikwalk/init/init_0624+.h";

    std::ifstream HeaderFile(addr.c_str());
    int i;
    for(i = 0; i < 2; i++)
    {
        HeaderFile.ignore(65535, '[');
    }
    HeaderFile.ignore(100, '{');

    i = 0;
    char ch;
    while(!HeaderFile.eof())
    {
        HeaderFile >> Motion_Data[i];
        HeaderFile >> ch;
        if(ch == ' ')
            break;
        if(ch == '}')
            break;
        i++;
        if(i > 400)
            break;
    }
    HeaderFile.close();

    for(int DXL_ID = 0; DXL_ID <= 22; DXL_ID++)
    {
        g_DXL_ID_position[DXL_ID] = Motion_Data[11 + DXL_ID]*4;
        g_DXL_ID_Save_position[DXL_ID] = Motion_Data[11 + DXL_ID]*4;

        cout<<"g_DXL_ID_Save_position["<<DXL_ID<<"] = "<<g_DXL_ID_Save_position[DXL_ID]<<endl;
    }
}


void IK_solve::solve(double pX_r, double pY_r, double pZ_r, double RYaw, double pX_l, double pY_l, double pZ_l, double LYaw, int body)
{
    const double l_0 = 60.5;      //      골반 너비
    const double l_1 = 60.5;      //      골반 너비
    const double l_2 = 170;      //     축간 거리
    const double l_3 = 170;      //     축간 거리

    double LYaw1;
    double RYaw1;
    double LValid = 0;
    double RValid = 0;
    double temp[4] = {0,0,0,0};
    double temp1[4] = {0,0,0,0};

    LYaw1 = (double)(LYaw*deg2rad);

    LValid = (double)(((pX_l*pX_l) + ((pY_l-l_0)*(pY_l-l_0)) + ((pZ_l)*(pZ_l)) - (l_2*l_2) - (l_3*l_3) ) / (2*l_2*l_3));

    InvLegAngle[0] = LYaw1;
    InvLegAngle[3] = (double)acos(LValid);
    InvLegAngle[1] = (double)(atan2(pZ_l, -pX_l*(double)sin(LYaw1)+(pY_l-l_0)*(double)cos(LYaw1)));

    temp[0] = (double)(-l_3*sin(InvLegAngle[3]));
    temp[1] = (double)(l_2+l_3*cos(InvLegAngle[3]));
    temp[2] = (double)(pX_l*cos(LYaw1)+(pY_l-l_0)*sin(LYaw1));
    temp[3] = (double)(-pX_l*sin(LYaw1)*cos(InvLegAngle[1])+(pY_l-l_0)*cos(LYaw1)*cos(InvLegAngle[1])+pZ_l*sin(InvLegAngle[1]));

    LValid = (double)((temp[0]*temp[3]-temp[1]*temp[2])/(temp[0]*temp[0]+temp[1]*temp[1]));

    InvLegAngle[2] = (double)asin(LValid);
    InvLegAngle[1] = (double)((double)InvLegAngle[1] + (double)(PI/2.0));

    InvLegAngle[4] = -InvLegAngle[3] - InvLegAngle[2];
    InvLegAngle[5] = (double)(1.0*(double)InvLegAngle[1]);

    //------------------------------------------------------------------------------------------------

    RYaw1 = (double)(RYaw*deg2rad);

    RValid = (double)(((pX_r*pX_r) + ((pY_r+l_0)*(pY_r+l_0)) + ((pZ_r)*(pZ_r)) - (l_2*l_2) - (l_3*l_3) ) / (2*l_2*l_3));

    InvLegAngle[6] = RYaw1;
    InvLegAngle[9] = (double)acos(RValid);
    InvLegAngle[7] = (double)(atan2(pZ_r, -pX_r*sin(RYaw1)+(l_0+pY_r)*cos(RYaw1)));

    temp1[0] = (double)(-l_3*sin(InvLegAngle[9]));
    temp1[1] = (double)(l_2+l_3*cos(InvLegAngle[9]));
    temp1[2] = (double)(pX_r*cos(RYaw1)+(pY_r+l_0)*sin(RYaw1));
    temp1[3] = (double)(-pX_r*sin(RYaw1)*cos(InvLegAngle[7])+(pY_r+l_0)*cos(RYaw1)*cos(InvLegAngle[7])+pZ_r*sin(InvLegAngle[7]));

    RValid = (double)((temp1[0]*temp1[3]-temp1[1]*temp1[2])/(temp1[0]*temp1[0]+temp1[1]*temp1[1]));

    InvLegAngle[8] = (double)asin(RValid);
    InvLegAngle[7] = (double)((double)InvLegAngle[7]+PI/2);

    InvLegAngle[10] = -InvLegAngle[9] - InvLegAngle[8];
    InvLegAngle[11] = (double)(1.0*(double)InvLegAngle[7]);


    for(int i = 0; i < 23; i++)
    {
      g_DXL_ID_position[i] = g_DXL_ID_Save_position[i];
    }

    if(Push_Recovery_mode == 1)
    {
        if(Impact_flag == 1 && Push_re_back == 1)
        {
            if(Left_Period == 1)
            {
                R_Ankle_horizon_value =  0;  // 0  5
                L_Ankle_horizon_value = 10; //10 15

//                if(Z_down_flag_2 == 1)
//                    R_Ankle_horizon_value = 10;
//                else
//                    R_Ankle_horizon_value = 0;
            }
            else if(Right_Period == 1)
            {
                R_Ankle_horizon_value = 10; //10 15
                L_Ankle_horizon_value =  0;  // 0  5

//                if(Z_down_flag_2 == 1)
//                    L_Ankle_horizon_value = 10;
//                else
//                    L_Ankle_horizon_value = 0;
            }
            else
            {
                L_Ankle_horizon_value = 0;
                R_Ankle_horizon_value = 0;
            }
        }
        else
        {
            R_Ankle_horizon_value = 0;
            L_Ankle_horizon_value = 0;
        }
    }
    else
    {
        R_Ankle_horizon_value = 0;
        L_Ankle_horizon_value = 0;
    }

//    cout<<"L_Ankle_horizon_value     ====    "<<L_Ankle_horizon_value<<endl;
//    cout<<"R_Ankle_horizon_value     ====    "<<R_Ankle_horizon_value<<endl;

    g_DXL_ID_position[12] += ang2pos(InvLegAngle[0]*rad2deg)+ ang.ang12;                                                                 //left_Yaw
    g_DXL_ID_position[14] += ang2pos(InvLegAngle[2]*rad2deg + Pitch_ADD_Angle*balanceInfo.pelvic_amp) + ang.ang14;                       //left_pitch
    g_DXL_ID_position[16] -= (ang2pos(InvLegAngle[3]*rad2deg + Pitch_ADD_Angle*balanceInfo.knee_amp)) + ang.ang16;                       //left_pitch
    g_DXL_ID_position[18] -= ang2pos(InvLegAngle[4]*rad2deg + Pitch_ADD_Angle*balanceInfo.ankle_amp - L_Ankle_horizon_value) + ang.ang18;  //left_pitch
    g_DXL_ID_position[10] += ang2pos(InvLegAngle[1]*rad2deg + Roll_ADD_Angle*balanceInfo.roll_pelvic_amp) + ang.ang10;                   //left_roll
    g_DXL_ID_position[20] -= ang2pos(-InvLegAngle[5]*rad2deg + Roll_ADD_Angle*balanceInfo.roll_ankle_amp) + ang.ang20;                   //left_roll

    g_DXL_ID_position[13] += ang2pos(InvLegAngle[6]*rad2deg) + ang.ang13;                                                                //Right_Yaw
    g_DXL_ID_position[15] -= ang2pos(InvLegAngle[8]*rad2deg + Pitch_ADD_Angle*balanceInfo.pelvic_amp) + ang.ang15;                       //right_pitch
    g_DXL_ID_position[17] += (ang2pos(InvLegAngle[9]*rad2deg + Pitch_ADD_Angle*balanceInfo.knee_amp)) + ang.ang17;                       //right_pitch
    g_DXL_ID_position[19] += ang2pos(InvLegAngle[10]*rad2deg + Pitch_ADD_Angle*balanceInfo.ankle_amp - R_Ankle_horizon_value) + ang.ang19; //right_pitch
    g_DXL_ID_position[11] += ang2pos(InvLegAngle[7]*rad2deg + Roll_ADD_Angle*balanceInfo.roll_pelvic_amp) + ang.ang11;                   //right_roll
    g_DXL_ID_position[21] -= ang2pos(-InvLegAngle[11]*rad2deg + Roll_ADD_Angle*balanceInfo.roll_ankle_amp) + ang.ang21;                  //right_roll

    now_16_Angle_value = g_DXL_ID_position[16];
    now_15_Angle_value = g_DXL_ID_position[15];

    cout << "L_Ankle_horizon_value >> " << L_Ankle_horizon_value << endl;
    cout << "R_Ankle_horizon_value >> " << R_Ankle_horizon_value << endl;


    //shoulder
//    g_DXL_ID_position[0] += ang2pos(ang.ang12);
//    g_DXL_ID_position[1] -= ang2pos(ang.ang12);

    if(body == All)
      motor_packet(All, 23);
    else if(body == Leg)
      motor_packet(Leg, 22);

    if(ikinit_flag)
    {
        cout<<endl<<"====  walk init ===="<<endl<<endl;
        unsigned char DXL_ID;
        for(DXL_ID = 0; DXL_ID <= 22; DXL_ID++)
        {
            std::cout << "g_DXL_ID_position[" << (int)DXL_ID << "] = " << g_DXL_ID_position[DXL_ID] << std::endl;
        }
        ikinit_flag = false;
    }
}

void IK_solve::motor_packet(int body, int limit)
{
    msg_generate::Motor_msg dxMsg;

    if(body == Leg)
    {
        for(int i = 0; i < 2; i++)
        {
            dxMsg.id.push_back(i);
            int pos = g_DXL_ID_position[i];
            dxMsg.position.push_back(pos);

            dxMsg.speed.push_back(400);
        }
    }

    for(int i = body; i < limit; i++)
    {
        dxMsg.id.push_back(i);
        int pos = g_DXL_ID_position[i];
        dxMsg.position.push_back(pos);

        dxMsg.speed.push_back(400);
    }

    dxMsg.length = dxMsg.id.size();
    dxMsg.mode = 3;


    motor_pub.publish(dxMsg);
}

