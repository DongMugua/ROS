/*
Biped LIPMWalk based LIPM
LIPMWalk.cpp
*/
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string.h>
#include "LIPMWalk.h"
#include "Matrix3x3.h"
#include "bl_line.h"
#include "Kinematics.h"

using namespace Robot;
using namespace GaitManager;
using namespace std;

Bline g_Bline;//贝塞尔曲线

LIPMWalk::LIPMWalk()
{
    double  x = 0.05;
    double  y = 0.05;

    double  t = 0.6;
    double  h = 0.04;

    double timestep = 20; //步长时间 单位ms
    double  TC_x = 7;//7.5;

    double  DSP = 0.3;
    double  TC_y = 6.9;//6.9with hull   6.0without hull

    stepH = 0.03;
    stepL = 0.10;
    Gradient = 0; //-0.125;//  hight/length
    //步态参数第一次初始化
    StepCountTarget = 1000;
    Step_TC_x = TC_x;
    Step_TC_y = TC_y;
    X0 = x;
    Y0 = y;
    T_circle = t;
    Swing_H = h;
    TorsoHight = Kinematics::GetInstance()->TorsoHight;
    TorsoCoM_x = -Kinematics::GetInstance()->xTorsoToHip;
    TorsoCoM_z = Kinematics::GetInstance()->zTorsoToHip;

    TimeStep = 0.001*timestep;
    DSPRatio = DSP;

    ////////side walk//////
    Y_sidewalk = -0.02;
    Y0_begin = 0.05;
    /////////////////////////

    PatternInit();
    KinematicsInit();
    InverseKinematics();

    IsWalking = false;
}

LIPMWalk::~LIPMWalk() {
}

bool LIPMWalk::start(double x, int target, double *waypoint_Yaw) {
    if(RobotState != Walk_standed)
        return false;
    IsWalking = true;
    X0 = x;
    StepCountTarget = target;
    for(int i= 0; i<target; i++) {
        WayPoint_Yaw[i] = *(waypoint_Yaw+i);
    }
    PatternInit();
    RobotState = Walk_start;
    return true;
}

bool LIPMWalk::stopWalking() {
    if(!IsWalking)
        return false;

    IsWalking = false;
    if(RobotState==Walk_forward)
        RobotState=Walk_stop;
    else if(RobotState==Walk_forward_slope)
        RobotState=Walk_stop_slope;
    else if(RobotState==Walk_forward_stair)
        RobotState=Walk_stop_stair;
    else
        return false;

    for(int i= 0; i<StepCountTarget; i++)
        WayPoint_Yaw[i] = 0;


    return true;
}

void LIPMWalk::RobotYaw_Update()
{
    RobotYaw_Old = RobotYaw_Present;
    RobotYaw_Present = RobotYaw_New;
    RobotYaw_New = WayPoint_Yaw[StepCount];
}


void LIPMWalk::run() {

    switch(RobotState) {
    case Walk_start_side:
        start_side();
        break;
    case Walk_stop_side:
        stop_side();
        break;
    case Walk_side:
        Step_side();
        break;

    case Walk_forward:
        Step_forward();
        break;
    case Walk_start:
        start_right();
        break;
    case Walk_stop:
        stop();
        break;

    case Walk_forward_stair:
        Step_forward_stair();
        break;
    case Walk_start_stair:
        start_right_stair();
        break;
    case Walk_stop_stair:
        stop_stair();
        break;

    case Walk_forward_slope:
        Step_forward_slope();
        break;
    case Walk_start_slope:
        start_right_slope();
        break;
    case Walk_stop_slope:
        stop_slope();
        break;

    case Walk_stand:
        stand();
        break;
    case Action_UpDown:
        SingleStand_UpDown();
        break;
    case Action_Bezier:
        Bezier();
        break;
    case Action_LiftFoot:
        LiftFoot();
        break;
    case Action_SwingAround:
        SwingAround();
        break;
    case Action_YawAround:
        YawAround();
        break;
    case Action_SlopeStand:
        SlopeStand();
        break;

    }
    InverseKinematics();
}


void LIPMWalk::Step_side()
{
    if(StepRhythm==0) {
        X0_Y0_V0Cal_side();
    }

    StepRhythm+=1;
    CoMcal(StepRhythm);
    Foot_xyzCal_side(StepRhythm);
    if(StepRhythm>=RhythmCount)
    {
        StepRhythm=0;
        StepState = (StepState == Rfoot_stance)?Lfoot_stance:Rfoot_stance;
        StepCount++;
    }
}

void LIPMWalk::Step_forward()
{
    if(StepRhythm==0) {
        RobotYaw_Update();
        X0_Y0_V0Cal();
    }
    if(StepRhythm==0.5*RhythmCount)
        StepPaused = true;
    else
        StepPaused = false;

    StepRhythm+=1;

    CoMcal(StepRhythm);
    Foot_xyzCal(StepRhythm);

    if(StepRhythm>=RhythmCount)
    {
        StepRhythm=0;
        StepState = (StepState == Rfoot_stance)?Lfoot_stance:Rfoot_stance;
        StepCount++;
    }
}


void LIPMWalk::Step_forward_stair()
{
    if(StepRhythm==0)
        X0_Y0_V0Cal();

    if(StepRhythm==0.5*RhythmCount)
        StepPaused = true;
    else
        StepPaused = false;

    StepRhythm+=1;
    CoMcal(StepRhythm);
    Foot_xyzCal_stair(StepRhythm);

    if(StepRhythm>=RhythmCount)
    {
        StepRhythm=0;
        if(StepState == Rfoot_stance)
            StepState=Lfoot_stance;
        else
            StepState=Rfoot_stance;
        StepCount++;
    }
}
void LIPMWalk::Step_forward_slope()
{
    if(StepRhythm==0) {
        RobotYaw_Update();
        X0_Y0_V0Cal();
    }
    if(StepRhythm==0.5*RhythmCount)
        StepPaused = true;
    else
        StepPaused = false;

    StepRhythm+=1;

    CoMcal(StepRhythm);
    Foot_xyzCal_slope(StepRhythm);
    if(StepRhythm>=RhythmCount)
    {
        StepRhythm=0;
        if(StepState == Rfoot_stance)
            StepState=Lfoot_stance;
        else
            StepState=Rfoot_stance;
        StepCount++;
    }
}


void LIPMWalk::start_side()
{
    if(StepRhythm == 0) {
        StepState = (Y_sidewalk>0)?Rfoot_stance:Lfoot_stance;
        X0_Y0_V0Cal_side();
    }
    if(StepRhythm<=0.5*RhythmCount)
    {
        StepRhythm+=1;
        CoMcal(StepRhythm);

        double t=TimeStep*0.5*RhythmCount;
        double y_end = (CoM_y_RelaToP[0]*cosh(Step_TC_y*t) + V0_y_RelaToP[0]/Step_TC_y*sinh(Step_TC_y*t));
        Torso_y = -(D_cal-y_end)*(1+sin(Pi*StepRhythm/(0.5*RhythmCount)-Pi/2))/2;

        double swing_z = Swing_H*sin(Pi*StepRhythm/(0.5*RhythmCount)-Pi/2);
        if(swing_z<0)
            swing_z = 0;
        if(StepState == Rfoot_stance)
            Lfoot_z=swing_z;
        else
            Rfoot_z=swing_z;
    }
    else if(StepRhythm<RhythmCount)
    {
        StepRhythm+=1;
        CoMcal(StepRhythm);
        Foot_xyzCal_side(StepRhythm);
    }

    if(StepRhythm>=RhythmCount)
    {
        StepRhythm=0;
        RobotState = Walk_side;
        StepState = (StepState == Rfoot_stance)?Lfoot_stance:Rfoot_stance;
        StepCount++;
    }
}

void LIPMWalk::start_left()
{
    if(StepRhythm == 0) {
        StepState = Rfoot_stance;
        X0_Y0_V0Cal();
    }
    if(StepRhythm<=0.5*RhythmCount)
    {
        StepRhythm+=1;
        CoMcal(StepRhythm);

        double t=TimeStep*0.5*RhythmCount;
        double y_end = (CoM_y_RelaToP[0]*cosh(Step_TC_y*t) + V0_y_RelaToP[0]/Step_TC_y*sinh(Step_TC_y*t));
        Torso_y = -(D_cal-y_end)*(1+sin(Pi*StepRhythm/(0.5*RhythmCount)-Pi/2))/2;
        Lfoot_z = Swing_H*sin(Pi*StepRhythm/(0.5*RhythmCount)-Pi/2);
        if(Lfoot_z<0)   Lfoot_z = 0;
    }
    else if(StepRhythm<RhythmCount)
    {
        StepRhythm+=1;
        CoMcal(StepRhythm);
        Foot_xyzCal(StepRhythm);
    }
    if(StepRhythm>=RhythmCount)
    {
        StepRhythm=0;
        RobotState = Walk_forward;
        StepState = Lfoot_stance;
        StepCount++;
    }
}

void LIPMWalk::start_right()
{
    if(StepRhythm == 0) {
        StepState = Lfoot_stance;
        X0_Y0_V0Cal();
    }
    if(StepRhythm<=0.5*RhythmCount)
    {
        StepRhythm+=1;
        CoMcal(StepRhythm);

        double t=TimeStep*0.5*RhythmCount;
        double y_end = (CoM_y_RelaToP[0]*cosh(Step_TC_y*t) + V0_y_RelaToP[0]/Step_TC_y*sinh(Step_TC_y*t));
        Torso_y = -(D_cal-y_end)*(1+sin(Pi*StepRhythm/(0.5*RhythmCount)-Pi/2))/2;
        Rfoot_z = Swing_H*sin(Pi*StepRhythm/(0.5*RhythmCount)-Pi/2);
        if(Rfoot_z<0)   Rfoot_z = 0;
    }
    else if(StepRhythm<RhythmCount)
    {
        StepRhythm+=1;
        CoMcal(StepRhythm);
        Foot_xyzCal(StepRhythm);
    }
    if(StepRhythm>=RhythmCount)
    {
        StepRhythm=0;
        RobotState = Walk_forward;
        StepState = Rfoot_stance;
        StepCount++;
    }
}

void LIPMWalk::stop()
{
    if(StepPaused == false) {
        Step_forward();
        return ;
    }
    StepRhythm++;
    double t=TimeStep*0.5*RhythmCount;
    double y_begin = (CoM_y_RelaToP[0]*cosh(Step_TC_y*t) + V0_y_RelaToP[0]/Step_TC_y*sinh(Step_TC_y*t));
    Torso_y = y_begin + (D_cal-y_begin)*(1+sin(Pi*(StepRhythm-0.5*RhythmCount)/(0.5*RhythmCount)-Pi/2))/2;
    Torso_x = 0;//tiaoshi
    Lfoot_x = 0;
    Rfoot_x = 0;

    double foot_z = Swing_H*sin(Pi*(RhythmCount - StepRhythm)/(0.5*RhythmCount)-Pi/2);
    if(foot_z<0)        foot_z = 0;
    if(StepState == Rfoot_stance)
        Lfoot_z=foot_z;
    else
        Rfoot_z=foot_z;

    if(StepRhythm>=RhythmCount)
    {
        StepRhythm = 0;
        StepState=Rfoot_stance;
        RobotState = Walk_stand;
        StepCount = 0;//停止一次 步数计零
    }
}

void LIPMWalk::stop_side()
{
    if(StepRhythm <= 0.5*RhythmCount ) {
        Step_side();
        return ;
    }

    StepRhythm++;
    double t=TimeStep*0.5*RhythmCount;
    double y_begin = (CoM_y_RelaToP[0]*cosh(Step_TC_y*t) + V0_y_RelaToP[0]/Step_TC_y*sinh(Step_TC_y*t));
    Torso_y = y_begin + (-D_cal_new-y_begin)*(1+sin(Pi*(StepRhythm-0.5*RhythmCount)/(0.5*RhythmCount)-Pi/2))/2;

    double rhythm = (double)(StepRhythm-RhythmCount*DSPRatio/2)/(double)(RhythmCount*(1-DSPRatio));
    if(rhythm<0)    rhythm = 0;
    if(rhythm>1)    rhythm = 1;
    double swing_y = swing_y_old+(swing_y_new-swing_y_old)*rhythm;
    double swing_z=Swing_H*sin(Pi*rhythm);
    // double swing_z = Swing_H*sin(Pi*(RhythmCount - StepRhythm)/(0.5*RhythmCount)-Pi/2);
    if(swing_z<0)
        swing_z = 0;
    if(StepState == Rfoot_stance) {
        Lfoot_z=swing_z;
        Lfoot_y=swing_y;
    }
    else {
        Rfoot_z=swing_z;
        Rfoot_y=swing_y;
    }


    if(StepRhythm>=RhythmCount)
    {
        StepRhythm = 0;
        StepState=Rfoot_stance;
        RobotState = Walk_stand;
        StepCount = 0;//停止一次 步数计零
    }
}

void LIPMWalk::stand()
{
    KinematicsInit();
    RobotState=Walk_standed;
}


void LIPMWalk::start_right_stair()
{
    if(StepRhythm == 0) {
        StepState = Rfoot_stance;
        X0_Y0_V0Cal();
    }
    if(StepRhythm<=0.5*RhythmCount)
    {
        StepRhythm+=1;
        CoMcal(StepRhythm);
        double t=TimeStep*0.5*RhythmCount;
        double y_end = (CoM_y_RelaToP[0]*cosh(Step_TC_y*t) + V0_y_RelaToP[0]/Step_TC_y*sinh(Step_TC_y*t));
        if(StepRhythm<0.3*RhythmCount)
            Torso_y = -(D_cal-y_end)*(1+sin(Pi*StepRhythm/(0.3*RhythmCount)-Pi/2))/2;

        Lfoot_z = (0.6*Swing_H+stepH)*sin(Pi*StepRhythm/(0.5*RhythmCount)-Pi/2);
        if(Lfoot_z<0)   Lfoot_z = 0;
    }
    else if(StepRhythm<RhythmCount)
    {
        StepRhythm+=1;
        CoMcal(StepRhythm);
        Torso_x = CoM_x_RelaToP[StepRhythm];
        Torso_y = CoM_y_RelaToP[StepRhythm];

        Lfoot_x = -2*F_cal*(1+sin(Pi*(StepRhythm-0.5*RhythmCount)/(0.5*RhythmCount)-Pi/2))/2;
        Lfoot_y = 2*D_cal;
        Lfoot_z = stepH + 0.6*Swing_H*sin(Pi/2*StepRhythm/(0.5*RhythmCount));

        Rfoot_x = 0;
        Rfoot_y = 0;
        Rfoot_z = 0;

    }
    if(StepRhythm>=RhythmCount)
    {
        StepRhythm=0;
        RobotState = Walk_forward_stair;
        StepState = Lfoot_stance;
        StepCount++;
    }
}

void LIPMWalk::stop_stair()
{
    static double swing_x_old , swing_z_old , swing_P_old;
    static bool stopping = false;
    if(StepRhythm <= 0.8*RhythmCount && stopping == false) {
        Step_forward_stair();
        if(StepState == Rfoot_stance) {
            swing_x_old = Lfoot_x;
            swing_z_old = Lfoot_z;
            swing_P_old = Lfoot_P;
        }
        else if(StepState == Lfoot_stance) {
            swing_x_old = Rfoot_x;
            swing_z_old = Rfoot_z;
            swing_P_old = Rfoot_P;
        }
        if(StepRhythm == 0.8*RhythmCount)
            stopping = true;
        return ;
    }

    static int stoprhythm = 0;
    stoprhythm++;
    double t=TimeStep*0.8*RhythmCount;
    double y_begin = (CoM_y_RelaToP[0]*cosh(Step_TC_y*t) + V0_y_RelaToP[0]/Step_TC_y*sinh(Step_TC_y*t));
    double x_begin = (CoM_x_RelaToP[0]*cosh(Step_TC_x*t) + V0_x_RelaToP[0]/Step_TC_x*sinh(Step_TC_x*t));

    double swing_x, swing_z, swing_P;
    if(stoprhythm <= 20) {
        double rhythm = (double)stoprhythm/20;
        swing_z = swing_z_old + (0 - swing_z_old)*rhythm;
        swing_P = swing_P_old + (0 - swing_P_old)*rhythm;
        swing_x = swing_x_old + (0 - swing_x_old)*rhythm;
    }
    if(stoprhythm > 20) {
        Torso_y = y_begin + (D_cal-y_begin)*(1+sin(Pi*(stoprhythm-20)/30-Pi/2))/2;
        Torso_x = x_begin + (0-x_begin)*(1+sin(Pi*(stoprhythm-20)/30-Pi/2))/2;
    }
    if(StepState == Rfoot_stance) {
        Lfoot_x=swing_x;
        Lfoot_z=swing_z;
        Lfoot_P = swing_P;
    }
    else if(StepState == Lfoot_stance) {
        Rfoot_x=swing_x;
        Rfoot_z=swing_z;
        Rfoot_P = swing_P;
    }

    if(stoprhythm>=50)
    {
        StepRhythm = 0;
        StepState=Rfoot_stance;
        RobotState = Walk_stand;
        StepCount = 0;//停止一次 步数计零
    }
}

void LIPMWalk::start_right_slope()
{
    if(StepRhythm == 0) {
        StepState = Rfoot_stance;
        X0_Y0_V0Cal();
    }
    if(StepRhythm<=0.5*RhythmCount)
    {
        StepRhythm+=1;
        CoMcal(StepRhythm);
        // double H = fabs(2*F_cal)*Gradient;

        double t=TimeStep*0.5*RhythmCount;
        double y_end = (CoM_y_RelaToP[0]*cosh(Step_TC_y*t) + V0_y_RelaToP[0]/Step_TC_y*sinh(Step_TC_y*t));
        Torso_y = -(D_cal-y_end)*(1+sin(Pi*StepRhythm/(0.5*RhythmCount)-Pi/2))/2;
        Lfoot_z = Swing_H*sin(Pi*StepRhythm/(0.5*RhythmCount)-Pi/2);
        if(Lfoot_z<0)   Lfoot_z = 0;
    }
    else if(StepRhythm<RhythmCount)
    {
        StepRhythm+=1;
        CoMcal(StepRhythm);
        Foot_xyzCal_slope(StepRhythm);
    }
    if(StepRhythm>=RhythmCount)
    {
        StepRhythm=0;
        RobotState = Walk_forward_slope;
        StepState = Lfoot_stance;
        StepCount++;
    }
}

void LIPMWalk::stop_slope()
{
    if(StepPaused == false) {
        Step_forward_slope();
        return ;
    }
    StepRhythm++;
    double t=TimeStep*0.5*RhythmCount;
    double y_begin = (CoM_y_RelaToP[0]*cosh(Step_TC_y*t) + V0_y_RelaToP[0]/Step_TC_y*sinh(Step_TC_y*t));
    Torso_y = y_begin + (D_cal-y_begin)*(1+sin(Pi*(StepRhythm-0.5*RhythmCount)/(0.5*RhythmCount)-Pi/2))/2;
    Torso_x = 0;//tiaoshi
    Lfoot_x = 0;
    Rfoot_x = 0;

    double foot_z = Swing_H*sin(Pi*(RhythmCount - StepRhythm)/(0.5*RhythmCount)-Pi/2);
    if(foot_z<0)        foot_z = 0;
    if(StepState == Rfoot_stance)
        Lfoot_z=foot_z;
    else
        Rfoot_z=foot_z;
    if(StepRhythm>=RhythmCount)
    {
        StepRhythm = 0;
        StepState=Rfoot_stance;
        RobotState = Walk_stand;
        StepCount = 0;//停止一次 步数计零
    }
}


//输入：躯干位置和姿态、两脚位置和姿态
//输出：脚部12个舵机值
//姿态变换顺序为 Roll-Pitch-Yaw
//世界坐标系位于站立时的髋关节中间在地面的投影处
void LIPMWalk::KinematicsInit()
{
    Torso_x=0;
    Torso_y=0;
    Torso_z=TorsoHight;
    Lfoot_x=0;
    Lfoot_y=0.05;//HipWidth+SoleWidth;
    Rfoot_x=0;
    Rfoot_y=-0.05;//-(HipWidth+SoleWidth);
    Lfoot_z=0;
    Rfoot_z=0;
    Lfoot_P=atan(Gradient);
    Rfoot_P=atan(Gradient);

    RArm_R = -20;
    RArm_P = -20;
    RArm_elbow = -20;
    LArm_R = -20;
    LArm_P = -20;
    LArm_elbow = -20;
    RArm_Rdf = -20;
    RArm_Pdf = -20;
    RArm_elbowdf = -20;
    LArm_Rdf = -20;
    LArm_Pdf = -20;
    LArm_elbowdf = -20;

}
void LIPMWalk::InverseKinematics()
{
    double L_x,L_y,L_z,R_x,R_y,R_z;
    L_x = Lfoot_x + IKoffset_L_x;
    L_y = Lfoot_y + IKoffset_L_y;
    L_z = Lfoot_z + IKoffset_L_z;
    R_x = Rfoot_x + IKoffset_R_x;
    R_y = Rfoot_y + IKoffset_R_y;
    R_z = Rfoot_z + IKoffset_R_z;

    Matrix3x3 R1x, R1y, R1z,    R7xL, R7yL, R7zL,    R7xR, R7yR, R7zR,        R1, R7L, R7R;
    Vector3D p1(Torso_x, Torso_y, Torso_z);
    Vector3D p7L(L_x,L_y,L_z);
    Vector3D p7R(R_x,R_y,R_z);
    Vector3D p1To2L(0, HipWidth+SoleWidth, -(sqrt(TorsoCoM_z*TorsoCoM_z+TorsoCoM_x*TorsoCoM_x)));
    Vector3D p1To2R(0, -(HipWidth+SoleWidth), -(sqrt(TorsoCoM_z*TorsoCoM_z+TorsoCoM_x*TorsoCoM_x)));
    Vector3D p7To2L,p7To2L_Ankle;
    Vector3D p7To2R,p7To2R_Ankle;
    bool ZAdjusted=false;
    double Angle[20];
    double length_L, length_R;

    R1x.SetRx(Torso_R);
    R1y.SetRy(Torso_P);
    R1z.SetRz(Torso_Y);
    R1 = R1z*R1y*R1x;
    while(true)
    {
        p1.Z=Torso_z;
        p7To2L = p1+R1.Transform(p1To2L)-p7L;
        p7To2R = p1+R1.Transform(p1To2R)-p7R;
        length_L = p7To2L.Length();
        length_R = p7To2R.Length();
        //腿长限制
        if(length_L>(Thigh+Shank)||length_R>(Thigh+Shank)) {
            Torso_z -= 0.002;
            ZAdjusted =true;
            printf("left leg over length   length_L = %f    length_R = %f\n",length_L , length_R);
        }
        else {
            if(ZAdjusted == true) {
                printf("StepRhythm = %d   RhythmCount = %d\n",StepRhythm,  RhythmCount );
                printf("final length_L = %f  length_R = %f    final Torso_z = %f\n",length_L ,  length_R, Torso_z);
            }
            break;
        }
    }

    Angle[L_KNEE] = -acos((Thigh*Thigh+Shank*Shank-length_L*length_L)/(2*Thigh*Shank)) + Pi;
    Angle[R_KNEE] = -acos((Thigh*Thigh+Shank*Shank-length_R*length_R)/(2*Thigh*Shank)) + Pi;

    R7xL.SetRx(Lfoot_R);
    R7yL.SetRy(Lfoot_P);
    R7zL.SetRz(Lfoot_Y);
    R7L = R7zL*R7yL*R7xL;
    R7xR.SetRx(Rfoot_R);
    R7yR.SetRy(Rfoot_P);
    R7zR.SetRz(Rfoot_Y);
    R7R = R7zR*R7yR*R7xR;

    Matrix3x3 R7L_T = R7L.Transpose();
    p7To2L_Ankle = R7L_T.Transform(p7To2L);
    Matrix3x3 R7R_T = R7R.Transpose();
    p7To2R_Ankle = R7R_T.Transform(p7To2R);

    double x,y,z;
    x=p7To2L_Ankle.X,  y=p7To2L_Ankle.Y,  z=p7To2L_Ankle.Z;
    Angle[L_ANKLE_ROLL] = atan(y/z);
    Angle[L_ANKLE_PITCH] = -atan(x/(sqrt(y*y+z*z))) - Angle[L_KNEE]/2;

    x=p7To2R_Ankle.X,  y=p7To2R_Ankle.Y,  z=p7To2R_Ankle.Z;
    Angle[R_ANKLE_ROLL] = atan(y/z);
    Angle[R_ANKLE_PITCH] = -atan(x/(sqrt(y*y+z*z))) - Angle[R_KNEE]/2;

    Matrix3x3 R1To1real;
    R1To1real.SetRy(atan(TorsoCoM_x/TorsoCoM_z));
    Matrix3x3 R1real = R1*R1To1real;
    Matrix3x3 R1real_T = R1real.Transpose();

    Matrix3x3 Rxq7L,Ryq5L_q6L;
    Rxq7L.SetRx(-Angle[L_ANKLE_ROLL]);
    Ryq5L_q6L.SetRy(-Angle[L_KNEE] -Angle[L_ANKLE_PITCH]);
    Matrix3x3 R234L = R1real_T*R7L*Rxq7L*Ryq5L_q6L;
    Angle[L_HIP_YAW] = atan(-R234L.m[m01]/R234L.m[m11]);
    Angle[L_HIP_ROLL] = atan(R234L.m[m21]/(-R234L.m[m01]*sin(Angle[L_HIP_YAW])+R234L.m[m11]*cos(Angle[L_HIP_YAW])));
    Angle[L_HIP_PITCH] = atan(-R234L.m[m20]/R234L.m[m22]);

    Matrix3x3 Rxq7R,Ryq5R_q6R;
    Rxq7R.SetRx(-Angle[R_ANKLE_ROLL]);
    Ryq5R_q6R.SetRy(-Angle[R_KNEE] -Angle[R_ANKLE_PITCH]);
    Matrix3x3 R234R = R1real_T*R7R*Rxq7R*Ryq5R_q6R;
    Angle[R_HIP_YAW] = atan(-R234R.m[m01]/R234R.m[m11]);
    Angle[R_HIP_ROLL] = atan(R234R.m[m21]/(-R234R.m[m01]*sin(Angle[R_HIP_YAW])+R234R.m[m11]*cos(Angle[R_HIP_YAW])));
    Angle[R_HIP_PITCH] = atan(-R234R.m[m20]/R234R.m[m22]);

    for(int i=7; i<=18; i++)
        Angle[i] *= Rad2Deg;

    JointValue[1] = RArm_P*AngleToValue+2048;//肩部前后摆动，中间为2048，向前摆RArm_P增大
    JointValue[2] = -LArm_P*AngleToValue+2048;
    JointValue[3] = RArm_R*AngleToValue+2048;//肩部侧面方向上下摆动，斜向下为2048，向上RArm_R增大
    JointValue[4] = -LArm_R*AngleToValue+2048;
    JointValue[5] = RArm_elbow*AngleToValue+2048; //伸直为1024，弯曲时RArm_elbow增大
    JointValue[6] = -LArm_elbow*AngleToValue+2048; //伸直为3072


    //IK mistake:
    if(Angle[11] > 70)
        Angle[11]  = Angle[11]  - 180;
    if(Angle[12] > 70)
        Angle[12]  = Angle[12]  - 180;

    JointValue[7] = int(-Angle[7]*AngleToValue+2048);
    JointValue[8] = int(-Angle[8]*AngleToValue+2048);
    JointValue[9] = int(-Angle[9]*AngleToValue+2048);
    JointValue[10] = int(-Angle[10]*AngleToValue+2048);
    JointValue[11] = int(Angle[11]*AngleToValue+2048);
    JointValue[12] = int(-Angle[12]*AngleToValue+2048);
    JointValue[13] = int(Angle[13]*AngleToValue+2048);
    JointValue[14] = int(-Angle[14]*AngleToValue+2048);
    JointValue[15] = int(-Angle[15]*AngleToValue+2048);
    JointValue[16] = int(Angle[16]*AngleToValue+2048);
    JointValue[17] = int(Angle[17]*AngleToValue+2048);
    JointValue[18] = int(Angle[18]*AngleToValue+2048);

    // JointValue[19] = 2048;
    // JointValue[20] = 2048;
    //for(int id=7;id<19;id++)
    //printf("id = %d  JointValue = %f\n",id, JointValue[id] );
}

/*******************************Pattern*********************************/
/*******************************Pattern*********************************/
/*******************************Pattern*********************************/
/*******************************Pattern*********************************/

////////side walk//////
void LIPMWalk::PatternInit_side()
{
    Y0_begin =Y_sidewalk/(fabs(Y_sidewalk))*fabs(Y0_begin);
    Y0_end = Y0_begin+Y_sidewalk;

    double SINH = sinh(Step_TC_y*T_circle);
    double COSH = cosh(Step_TC_y*T_circle);
    V0y_begin = (Y0_end*Step_TC_y-Y0_begin*Step_TC_y*COSH)/SINH;
    V0y_end = Y0_begin*Step_TC_y*SINH + V0y_begin*COSH;
}
void LIPMWalk::X0_Y0_V0Cal_side()
{
    double Vy;
    if(StepState==Rfoot_stance) {
        if(Y_sidewalk<0) {
            D_cal = fabs(Y0_end);
            D_cal_new = -fabs(Y0_begin);
            Vy = -fabs(V0y_end);
            swing_y_old = 2*fabs(Y0_end);
            swing_y_new = 2*fabs(Y0_begin);
        }
        else if(Y_sidewalk>=0) {
            D_cal = fabs(Y0_begin);
            D_cal_new = -fabs(Y0_end);
            Vy = -fabs(V0y_begin);
            swing_y_old = 2*fabs(Y0_begin);
            swing_y_new = 2*fabs(Y0_end);
        }
    }
    else if(StepState==Lfoot_stance) {
        if(Y_sidewalk<0) {
            D_cal = -fabs(Y0_begin);
            D_cal_new = fabs(Y0_end);
            Vy = fabs(V0y_begin);
            swing_y_old = -2*fabs(Y0_begin);
            swing_y_new = -2*fabs(Y0_end);
        }
        else if(Y_sidewalk>=0) {
            D_cal = -fabs(Y0_end);
            D_cal_new = fabs(Y0_begin);
            Vy = fabs(V0y_end);
            swing_y_old = -2*fabs(Y0_end);
            swing_y_new = -2*fabs(Y0_begin);
        }
    }
    CoM_y_RelaToP[0] = D_cal;
    V0_y_RelaToP[0] = Vy;
}
/////////////////////////


void LIPMWalk::PatternInit()
{
    //步态初始化
    RobotState = Walk_stand;
    StepState = Rfoot_stance;//初始为右脚支撑，X0为负，Y0为负
    SwingState = Swing_air;

    V0_x = (X0*Step_TC_x*(1+cosh(Step_TC_x*T_circle))/(sinh(Step_TC_x*T_circle)));
    V0_y = -(Y0*(Step_TC_y)*(1-cosh(Step_TC_y*T_circle))/(sinh(Step_TC_y*T_circle)));
    V0 = sqrt(V0_x*V0_x+V0_y*V0_y);
    Angle_VxVy = atan(V0_x/V0_y)*Rad2Deg;
    RhythmCount = T_circle/TimeStep;//周期除以仿真步长，为周期内控制次数
    StepRhythm = 0;
}

//倒立摆坐标系与世界坐标系原点重合
//倒立摆坐标系与世界坐标系正负方向相同，夹角为RobotYaw_Present/2
void LIPMWalk::X0_Y0_V0Cal()
{
    //倒立摆坐标系内
    double Vx, Vy, Vx_old, Vy_old, Vx_new, Vy_new;
    if(StepState==Rfoot_stance) {
        double temp=(Angle_VxVy-RobotYaw_Present/2)/Rad2Deg;
        Vx = V0*sin(temp);
        Vy = -V0*cos(temp);

        double temp_old = (Angle_VxVy+RobotYaw_Old/2)/Rad2Deg;
        Vx_old = V0*sin(temp_old);
        Vy_old = V0*cos(temp_old);

        double temp_new = (Angle_VxVy+RobotYaw_New/2)/Rad2Deg;
        Vx_new = V0*sin(temp_new);
        Vy_new = V0*cos(temp_new);
    }
    else if(StepState==Lfoot_stance) {
        double temp=(Angle_VxVy+RobotYaw_Present/2)/Rad2Deg;
        Vx = V0*sin(temp);
        Vy = V0*cos(temp);

        double temp_old=(Angle_VxVy-RobotYaw_Old/2)/Rad2Deg;
        Vx_old = V0*sin(temp_old);
        Vy_old = -V0*cos(temp_old);

        double temp_new=(Angle_VxVy-RobotYaw_New/2)/Rad2Deg;
        Vx_new = V0*sin(temp_new);
        Vy_new = -V0*cos(temp_new);
    }
    double Vx_end = Vx;
    double Vy_end = -Vy;
    double Vx_end_old = Vx_old;
    double Vy_end_old = -Vy_old;
    double Vx_end_new = Vx_new;
    double Vy_end_new = -Vy_new;

    D_cal_old = (Vy_end_old-Vy_old*cosh(Step_TC_y*T_circle))/(Step_TC_y*sinh(Step_TC_y*T_circle));
    F_cal_old = (Vx_end_old-Vx_old*cosh(Step_TC_x*T_circle))/(Step_TC_x*sinh(Step_TC_x*T_circle));
    D_cal = (Vy_end-Vy*cosh(Step_TC_y*T_circle))/(Step_TC_y*sinh(Step_TC_y*T_circle));
    F_cal = (Vx_end-Vx*cosh(Step_TC_x*T_circle))/(Step_TC_x*sinh(Step_TC_x*T_circle));
    D_cal_new = (Vy_end_new-Vy_new*cosh(Step_TC_y*T_circle))/(Step_TC_y*sinh(Step_TC_y*T_circle));
    F_cal_new = (Vx_end_new-Vx_new*cosh(Step_TC_x*T_circle))/(Step_TC_x*sinh(Step_TC_x*T_circle));

    CoM_x_RelaToP[0] = F_cal;
    CoM_y_RelaToP[0] = D_cal;
    V0_x_RelaToP[0] = Vx;
    V0_y_RelaToP[0] = Vy;
}

void LIPMWalk::CoMcal(int mRhythm)
{
    //倒立摆坐标系内
    double t=TimeStep;
    CoM_x_RelaToP[mRhythm] = (CoM_x_RelaToP[mRhythm-1]*cosh(Step_TC_x*t) + V0_x_RelaToP[mRhythm-1]/Step_TC_x*sinh(Step_TC_x*t));
    CoM_y_RelaToP[mRhythm] = (CoM_y_RelaToP[mRhythm-1]*cosh(Step_TC_y*t) + V0_y_RelaToP[mRhythm-1]/Step_TC_y*sinh(Step_TC_y*t));
    V0_x_RelaToP[mRhythm] = (CoM_x_RelaToP[mRhythm-1]*Step_TC_x*sinh(Step_TC_x*t) + V0_x_RelaToP[mRhythm-1]*cosh(Step_TC_x*t));
    V0_y_RelaToP[mRhythm] = (CoM_y_RelaToP[mRhythm-1]*Step_TC_y*sinh(Step_TC_y*t) + V0_y_RelaToP[mRhythm-1]*cosh(Step_TC_y*t));
}

void LIPMWalk::Foot_xyzCal_side(int mRhythm) {

    Torso_y = CoM_y_RelaToP[mRhythm];

    double rhythm = (double)(mRhythm-RhythmCount*DSPRatio/2)/(double)(RhythmCount*(1-DSPRatio));
    if(rhythm<0)    rhythm = 0;
    if(rhythm>1)    rhythm = 1;

    double stance_y, stance_z, swing_y, swing_z;
    stance_y = 0;
    stance_z = 0;

    swing_z=Swing_H*sin(Pi*rhythm);
    swing_y = swing_y_old+(swing_y_new-swing_y_old)*rhythm;

    if(StepState == Rfoot_stance) {
        Lfoot_y=swing_y;
        Lfoot_z=swing_z;
        Rfoot_y=stance_y;
        Rfoot_z=stance_z;
    }
    else if(StepState == Lfoot_stance) {
        Rfoot_y=swing_y;
        Rfoot_z=swing_z;
        Lfoot_y=stance_y;
        Lfoot_z=stance_z;
    }
}

void LIPMWalk::Foot_xyzCal(int mRhythm)
{
    Torso_x = CoM_x_RelaToP[mRhythm];
    Torso_y = CoM_y_RelaToP[mRhythm];
    Torso_z = TorsoHight;
    Torso_Y = (- RobotYaw_Present/2 + RobotYaw_Present*((double)(mRhythm)/(double)RhythmCount))/Rad2Deg;

    double rhythm = (double)(mRhythm-RhythmCount*DSPRatio/2)/(double)(RhythmCount*(1-DSPRatio));
    if(rhythm<0)    rhythm = 0;
    if(rhythm>1)    rhythm = 1;

    double stance_x, stance_y, stance_z, stance_Yaw, swing_x, swing_y, swing_z, swing_Yaw;

    stance_z = 0;
    stance_x = 0;
    stance_y = 0;
    stance_Yaw = RobotYaw_Present/2/Rad2Deg;

    swing_z=Swing_H*sin(Pi*rhythm);
    double a,x,y;
    a = (RobotYaw_Present/2 + RobotYaw_Old/2)/Rad2Deg;
    x=cos(a)*(-F_cal_old)+sin(a)*D_cal_old;
    y=-sin(a)*(-F_cal_old)+cos(a)*D_cal_old;//旋转
    double x_old = -x + F_cal;
    double y_old = -y + D_cal;
    a = -(RobotYaw_Present/2 + RobotYaw_New/2)/Rad2Deg;
    x=cos(a)*F_cal_new+sin(a)*D_cal_new;
    y=-sin(a)*F_cal_new+cos(a)*D_cal_new;//旋转
    double x_new = -x + (-F_cal);
    double y_new = -y + D_cal;
    double Yaw_old = -(RobotYaw_Present/2 + RobotYaw_Old/2)/Rad2Deg;
    double Yaw_new = (RobotYaw_New + RobotYaw_Present/2)/Rad2Deg;

    swing_x = x_old+(x_new-x_old)*rhythm;
    swing_y = y_old+(y_new-y_old)*rhythm;
    swing_Yaw = Yaw_old + (Yaw_new-Yaw_old)*rhythm;

    // double L_z = 0.05*sin(JY901Roll/Rad2Deg);//斜坡上原地踏步
    // double R_z = -0.05*sin(JY901Roll/Rad2Deg);
    if(StepState == Rfoot_stance) {
        Lfoot_x=swing_x;
        Lfoot_y=swing_y;
        Lfoot_z=swing_z;
        Lfoot_Y = swing_Yaw;
        Rfoot_x=stance_x;
        Rfoot_y=stance_y;
        Rfoot_z=stance_z;
        Rfoot_Y = stance_Yaw;
    }
    else if(StepState == Lfoot_stance) {
        Rfoot_x=swing_x;
        Rfoot_y=swing_y;
        Rfoot_z=swing_z;
        Rfoot_Y = swing_Yaw;
        Lfoot_x=stance_x;
        Lfoot_y=stance_y;
        Lfoot_z=stance_z;
        Lfoot_Y = stance_Yaw;
    }

    //Lfoot_P = JY901Pitch/Rad2Deg; Rfoot_P = JY901Pitch/Rad2Deg;
    //Lfoot_R = JY901Roll/Rad2Deg;  Rfoot_R = JY901Roll/Rad2Deg;

}

//torsoH = 0.255   t = 1.6;    h = 0.05;
void LIPMWalk::Foot_xyzCal_stair(int mRhythm)
{

    Torso_x = CoM_x_RelaToP[mRhythm];
    Torso_y = CoM_y_RelaToP[mRhythm];

    double rhythm = (double)mRhythm/(double)RhythmCount;

    if(rhythm<=0.5)
        Torso_z = TorsoHight-stepH+2*stepH*rhythm;

    double stance_x, stance_y, stance_z, stance_P, swing_x, swing_y, swing_z, swing_P;
    stance_z = 0;
    stance_x = 0;
    stance_y = 0;
    stance_P = 0;
    swing_y = 2*D_cal;

    static double swing_x_old , swing_z_old , swing_P_old;
    double z , y, x;
    if(rhythm<=0.5) {
        z = -stepH;
        x = 2*F_cal+SoleLength;
        y = 2*D_cal;
        double detaH = Torso_z - z - sqrt(TorsoCoM_z*TorsoCoM_z+TorsoCoM_x*TorsoCoM_x);
        double detaX = Torso_x - x;
        double detaY = fabs(Torso_y - y) - (HipWidth+SoleWidth);
        if( sqrt(detaY*detaY+detaH*detaH + (detaX+SoleLength)*(detaX+SoleLength)) < (Thigh+Shank-0.001) ) {
            swing_x = x-SoleLength;
            swing_z = z;
            swing_P = 0;
        }
        else {
            double t1 = sqrt(detaH*detaH + detaX*detaX);
            double t2 = sqrt((Thigh+Shank-0.001)*(Thigh+Shank-0.001)-detaY*detaY);
            double t3 = SoleLength;
            double P = acos((t1*t1+t3*t3-t2*t2)/(2*t1*t3))+acos(detaX/t1);
            swing_x = x + SoleLength*cos(P);
            swing_z = z + SoleLength*sin(P);
            swing_P = Pi - P;
            swing_x_old = swing_x;
            swing_z_old = swing_z;
            swing_P_old = swing_P;
        }
    }
    else {
        swing_x = swing_x_old;
        swing_z = swing_z_old + (stepH - swing_z_old)*2*(rhythm-0.5) +0.03*sin(Pi*2*(rhythm-0.5));
        swing_P = swing_P_old;
    }
    if(rhythm>=0.7) {
        swing_P = swing_P_old + (0 - swing_P_old)*(1/0.3)*(rhythm-0.7)-Pi/4*sin(Pi*(1/0.3)*(rhythm-0.7));
        swing_x = swing_x_old + (-2*F_cal_new - swing_x_old)*(1/0.3)*(rhythm-0.7);
    }
    if(StepState == Rfoot_stance) {
        Lfoot_x=swing_x;
        Lfoot_z=swing_z;
        Lfoot_P = swing_P;
        Lfoot_y = swing_y;
        Rfoot_x=stance_x;
        Rfoot_z=stance_z;
        Rfoot_P = stance_P;
        Rfoot_y = stance_y;
    }
    else if(StepState == Lfoot_stance) {
        Rfoot_x=swing_x;
        Rfoot_z=swing_z;
        Rfoot_P = swing_P;
        Rfoot_y = swing_y;
        Lfoot_x=stance_x;
        Lfoot_z=stance_z;
        Lfoot_P = stance_P;
        Lfoot_y = stance_y;
    }
}

void LIPMWalk::Foot_xyzCal_slope(int mRhythm)
{
    double H = fabs(2*F_cal)*Gradient;
    Torso_x = CoM_x_RelaToP[mRhythm];
    Torso_y = CoM_y_RelaToP[mRhythm];
    Torso_z = TorsoHight + H/2 - H*((double)mRhythm/(double)RhythmCount);
    Torso_Y = (- RobotYaw_Present/2 + RobotYaw_Present*((double)(mRhythm)/(double)RhythmCount))/Rad2Deg;

    double rhythm = (double)(mRhythm-RhythmCount*DSPRatio/2)/(double)(RhythmCount*(1-DSPRatio));
    if(rhythm<0)    rhythm = 0;
    if(rhythm>1)    rhythm = 1;

    double stance_x, stance_y, stance_z, stance_P, swing_x, swing_y, swing_z, swing_P;
    stance_z = 0;
    stance_x = 0;
    stance_y = 0;
    stance_P = atan(Gradient);

    swing_P = atan(Gradient);
    // swing_z=Swing_H*sin(Pi*((double)mRhythm/(double)RhythmCount))+H*(1-2*((double)mRhythm/(double)RhythmCount));
    // swing_z=Swing_H*sin(Pi*rhythm)+H*(1-2*((double)mRhythm/(double)RhythmCount));

    swing_z=Swing_H*sin(Pi*rhythm)+H*(1-2*rhythm);

    double a,x,y;
    a = (RobotYaw_Present/2 + RobotYaw_Old/2)/Rad2Deg;
    x=cos(a)*(-F_cal_old)+sin(a)*D_cal_old;
    y=-sin(a)*(-F_cal_old)+cos(a)*D_cal_old;//旋转
    double x_old = -x + F_cal;
    double y_old = -y + D_cal;
    a = -(RobotYaw_Present/2 + RobotYaw_New/2)/Rad2Deg;
    x=cos(a)*F_cal_new+sin(a)*D_cal_new;
    y=-sin(a)*F_cal_new+cos(a)*D_cal_new;//旋转
    double x_new = -x + (-F_cal);
    double y_new = -y + D_cal;

    swing_x = x_old+(x_new-x_old)*rhythm;
    swing_y = y_old+(y_new-y_old)*rhythm;

    if(StepState == Rfoot_stance) {
        Lfoot_x=swing_x;
        Lfoot_y=swing_y;
        Lfoot_z=swing_z;
        Lfoot_P = swing_P;
        Rfoot_x=stance_x;
        Rfoot_y=stance_y;
        Rfoot_z=stance_z;
        Rfoot_P = stance_P;
    }
    else if(StepState == Lfoot_stance) {
        Rfoot_x=swing_x;
        Rfoot_y=swing_y;
        Rfoot_z=swing_z;
        Lfoot_P = swing_P;
        Lfoot_x=stance_x;
        Lfoot_y=stance_y;
        Lfoot_z=stance_z;
        Rfoot_P = stance_P;
    }
}


//没有转弯角度时的双足支撑相
void LIPMWalk::DdoubleSup(int mRhythm)
{
    static int DSrhythm = 0;
    static double y_begin, x_begin, detaX, detaY;
    static int DScount;
    if(DSrhythm == 0) {
        double t=TimeStep*(1-DSPRatio/2)*RhythmCount;
        double vx = (CoM_x_RelaToP[0]*Step_TC_x*sinh(Step_TC_x*t) + V0_x_RelaToP[0]*cosh(Step_TC_x*t));
        double vy = (CoM_y_RelaToP[0]*Step_TC_y*sinh(Step_TC_y*t) + V0_y_RelaToP[0]*cosh(Step_TC_y*t));
        x_begin = (CoM_x_RelaToP[0]*cosh(Step_TC_x*t) + V0_x_RelaToP[0]/Step_TC_x*sinh(Step_TC_x*t));
        y_begin = (CoM_y_RelaToP[0]*cosh(Step_TC_y*t) + V0_y_RelaToP[0]/Step_TC_y*sinh(Step_TC_y*t));
        DScount = fabs(D_cal - y_begin)/fabs(vy)/TimeStep;
        detaX = -F_cal - x_begin;
        detaY = D_cal - y_begin;
    }
    DSrhythm++;
    // Torso_x = x_begin + 2*detaX/DScount*DSrhythm;
    // Torso_y = y_begin + 2*detaY/DScount*DSrhythm;

    printf("StepRhythm = %d\n",StepRhythm);
    printf("D_cal = %f   F_cal = %f  \n",D_cal, F_cal);
    printf("x_begin = %f   y_begin = %f  \n", x_begin, y_begin);


    if(DSrhythm == DScount) {
        DSfinish = true;
        DSrhythm = 0;
    }

}


/*******************************Motion*********************************/
/*******************************Motion*********************************/
/*******************************Motion*********************************/
/*******************************Motion*********************************/

//TC_y = 9;   t = 0.6;   y = 0.045;
void LIPMWalk::SingleStand_UpDown()
{
    if(StepPaused==false)
        return;

    double H, x, y;
    static int UpDownRhythm = 0;

    UpDownRhythm ++;

    Torso_z = TorsoHight - 0.03*sin(Pi*UpDownRhythm/40);
    if(UpDownCount==0 && UpDownRhythm<=7) {
        Torso_y = CoM_y_RelaToP[RhythmCount/2] - D_cal/3*UpDownRhythm/7;
    }

    if(UpDownCount==2 && UpDownRhythm>33) {
        Torso_y = CoM_y_RelaToP[RhythmCount/2] - D_cal/3 + D_cal/3*(UpDownRhythm-30)/7;
    }

    printf("Torso_y = %f \n",D_cal - Torso_y );

    if(UpDownRhythm >= 40)
    {
        UpDownRhythm = 0;
        UpDownCount ++;
    }
}

void LIPMWalk::YawAround()
{
    static int YawCount;
    static int i;
    int count = 135;
    i++;
    Torso_Y = Pi/4*sin(2*Pi*i/count);
    if (i == count)
    {
        i = 1;
        YawCount++;
    }
    if(YawCount == 0)
    {
        RArm_R = RArm_Rdf + (45 - RArm_Rdf)*i/count;
        RArm_P = RArm_Pdf + (0 - RArm_Pdf)*i/count;
        RArm_elbow = RArm_elbowdf + (-80 - RArm_elbowdf)*i/count;
        LArm_R = LArm_Rdf + (45 - LArm_Rdf)*i/count;
        LArm_P = LArm_Pdf + (0 - LArm_Pdf)*i/count;
        LArm_elbow = LArm_elbowdf + (-80 - LArm_elbowdf)*i/count;
    }
    else if(YawCount ==1||YawCount ==2)
    {
        double t = (1+sin(4*Pi*i/count-Pi/2))/2;
        RArm_R = 45 - 80*t;
        LArm_R = 45 - 80*t;
        RArm_P = 80*sin(2*Pi*i/count);
        LArm_P = - 80*sin(2*Pi*i/count);
    }
    if(YawCount == 3)
    {
        RArm_R = 45 + (RArm_Rdf-45)*i/count;
        RArm_P = RArm_Pdf*i/count;
        RArm_elbow = -80 + (RArm_elbowdf-(-80))*i/count;
        LArm_R = 45 + (LArm_Rdf-45)*i/count;
        LArm_P = LArm_Pdf*i/count;
        LArm_elbow = -80 + (LArm_elbowdf-(-80))*i/count;
    }
    if (YawCount == 4)
    {
        ActionDone = true;
        RobotState = Walk_stand;
    }
}

void LIPMWalk::SwingAround()
{
    ActionDone = false;

    static int i=1;
    double t = sqrt(TorsoCoM_z*TorsoCoM_z+TorsoCoM_x*TorsoCoM_x);
    double a = 0.02/(2*Pi);
    double sita,x,y;
    int RCount = 3;
    if(i<=90)//螺旋线开始
    {
        i++;
        sita = 2*Pi*i/90;
        x = a*sita*cos(sita);
        y = a*sita*sin(sita);
        Torso_x = 0;
        Torso_y = -0.045+0.045*i/90;
        Torso_z = 0.26;
    }
    else if(i<=90*(RCount+1))//绕圆
    {
        i++;
        sita = 2*Pi*i/90;
        x = 0.02*cos(sita);
        y = 0.02*sin(sita);
    }
    else if(i<=90*(RCount+2))//螺旋线结束
    {
        i++;
        sita = -2*Pi*(90*(RCount+2)-i)/90;
        x = a*sita*cos(sita+Pi);
        y = a*sita*sin(sita+Pi);
    }
    double t_V = sqrt(t*t-x*x-y*y);
    Torso_R = asin(y/t_V);
    Torso_P = asin(x/t_V);

    if(i>90*(RCount+2)) {
        ActionDone = true;
        RobotState = Action_YawAround;
    }
}


void LIPMWalk::LiftFoot()
{
    ActionDone = false;

    static int LiftFootCount;
    static int i=1;
    static int count = 90;
    i++;
    double t = (1+sin(2*Pi*i/count-Pi/2))/2;

    Torso_z = 0.26 - 0.05*t;
    Torso_y = -0.045 - 0.02*t;
    Torso_x = 0.01*t;
    LArm_R = LArm_Rdf+Rad2Deg*Pi/6*t;
    LArm_P = LArm_Pdf+Rad2Deg*Pi/4*t;

    Lfoot_z = 0.07*t;
    Lfoot_y = HipWidth+SoleWidth + 0.06*t;
    // Lfoot_x = - 0.05*t;
    if(LiftFootCount == 2&&i>45) {
        Torso_y = -(0.02+0.045)*t;
        Torso_z = 0.25 - 0.04*t;

        if(i == count) {
            ActionDone = true;
            RobotState = Action_SwingAround;
        }

    }
    if(i ==count)
    {
        i=1;
        LiftFootCount++;
        printf("LiftFootCount = %d\n",LiftFootCount );
    }
}

void LIPMWalk::Bezier()
{
    ActionDone = false;
    const Bline::Real keys[] = {
        // (Bline::Real)0, (Bline::Real)0, (Bline::Real)0.260,
        (Bline::Real)-0.001, (Bline::Real)0.045, (Bline::Real)0.26,
        (Bline::Real)0.001, (Bline::Real)0, (Bline::Real)0.240,
        (Bline::Real)-0.001, (Bline::Real)-0.045, (Bline::Real)0.26,
        // (Bline::Real)0, (Bline::Real)0, (Bline::Real)0.260,
    };
    g_Bline.build(keys, sizeof(keys)/(3*sizeof(keys[0])));

    static int BezierCount;
    static int i=1;
    i++;
    static int count=180;
    double t = (1+sin(2*Pi*i/count))/2;
    if(i==count)    {
        i=1;
        BezierCount ++;
        count = 90;
        printf("BezierCount = %d\n", BezierCount);
    }
    if(BezierCount==2&&i==count/4) {
        ActionDone = true;
        RobotState = Action_SwingAround;
    }

    g_Bline.getPoint(t , g_Bline.point, g_Bline.tan);
    Torso_x = g_Bline.point.x;
    Torso_y = g_Bline.point.y;
    Torso_z = g_Bline.point.z;
    Lfoot_x=0;
    Lfoot_y=HipWidth+SoleWidth;
    Lfoot_z=0;
    Rfoot_x=0;
    Rfoot_y=-(HipWidth+SoleWidth);
    Rfoot_z=0;
}

void LIPMWalk::SlopeStand()
{
    Lfoot_P = JY901Pitch/Rad2Deg;
    Rfoot_P = JY901Pitch/Rad2Deg;
    Lfoot_R = JY901Roll/Rad2Deg;
    Rfoot_R = JY901Roll/Rad2Deg;
    Lfoot_z = (HipWidth+SoleWidth)*sin(JY901Roll/Rad2Deg);
    Rfoot_z = -(HipWidth+SoleWidth)*sin(JY901Roll/Rad2Deg);

    // printf("in SlopeStand %f\n",JY901Pitch);
}
