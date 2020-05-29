/*
LIPMWalk.cpp
*/

#ifndef WALK_HPP
#define WALK_HPP

#define Pi 3.141592653
#define Rad2Deg 57.296
#define AngleToValue 11.377778

#define HipWidth 0.037
#define SoleWidth 0.008
#define SoleLength 0.052

#define Thigh 0.093
#define Shank 0.093

#define LiftPhaseRatio 0.2
#define AirPhaseRatio 0.6
#define TouchPhaseRatio 0.2

#define R_SHOULDER_PITCH 1
#define L_SHOULDER_PITCH 2
#define R_SHOULDER_ROLL 3
#define L_SHOULDER_ROLL 4
#define R_ELBOW 5
#define L_ELBOW 6
#define R_HIP_YAW 7
#define L_HIP_YAW 8
#define R_HIP_ROLL 9
#define L_HIP_ROLL 10
#define R_HIP_PITCH 11
#define L_HIP_PITCH 12
#define R_KNEE 13
#define L_KNEE 14
#define R_ANKLE_PITCH 15
#define L_ANKLE_PITCH 16
#define R_ANKLE_ROLL 17
#define L_ANKLE_ROLL 18
#define HEAD_PAN 19
#define HEAD_TILT 20

//原版补偿值    用于InverseKinematics()
#define IKoffset_L_x  0
#define IKoffset_L_y  0
#define IKoffset_L_z  0
#define IKoffset_R_x   0//0.006  
#define IKoffset_R_y   0//0.002
#define IKoffset_R_z  0//-0.002

// //原版补偿值
// #define IKoffset_L_x  0
// #define IKoffset_L_y  0 //  + 0.01
#define IKoffset_L_H  0
// #define IKoffset_R_x   0.002
// #define IKoffset_R_y   0.008  //+0.01
#define IKoffset_R_H  0.002

namespace GaitManager {
class LIPMWalk {
public:
    LIPMWalk();
    ~LIPMWalk();
    void run();
    bool start(double x, int target, double *waypoint_Yaw);
    bool stopWalking();

    double CoM_Vx, CoM_Vy;
    double CoM_x_RelaToW[10], CoM_y_RelaToW[10];
    double CoM_x_ideal[10], CoM_y_ideal[10];
    //, CoM_x_ideal_last, CoM_y_ideal_last;
    double CoM_x_measured, CoM_y_measured, CoM_H_measured;
    double Kp_x, Ki_x, Kd_x, Kp_y, Ki_y, Kd_y;
    double P_x, I_x, D_x, P_y, I_y, D_y, PID_x, PID_y;
    double error_com_x, error_com_y, error_com_x_last, error_com_y_last;
    int MeasureLagCircle;
    bool JointReadOK;
    bool IsAdjusted;
    bool ActionDone;
    bool IsWalking;

    double WayPoint_Yaw[1000];
    int StepCount;
    int StepCountTarget;
    double Hip_Yaw_L, Hip_Yaw_R;
    double Lfoot_H, Rfoot_H;
    double Swing_H;
    double JointValue[21];//二十个舵机  0号不用
    double CurrentJointValue[21];

    //定义右手系，机器人前方为x轴正方形，左方为y轴正方向，上方为z轴正方向
    double Lfoot_x, Lfoot_y, Lfoot_z, Lfoot_R, Lfoot_P, Lfoot_Y;
    double Rfoot_x, Rfoot_y, Rfoot_z, Rfoot_R, Rfoot_P, Rfoot_Y;
    double Torso_x, Torso_y, Torso_z, Torso_R, Torso_P, Torso_Y, TorsoHight, TorsoCoM_z, TorsoCoM_x;
    double LArm_R, LArm_P, LArm_elbow, RArm_R, RArm_P, RArm_elbow;
    double LArm_Rdf, LArm_Pdf, LArm_elbowdf, RArm_Rdf, RArm_Pdf, RArm_elbowdf;


    enum StateOfRobot
    {
        Walk_side,
        Walk_start_side,
        Walk_stop_side,

        Walk_forward,
        Walk_start,
        Walk_stop,

        Walk_forward_stair,
        Walk_start_stair,
        Walk_stop_stair,

        Walk_forward_slope,
        Walk_start_slope,
        Walk_stop_slope,

        Walk_stand,
        Walk_standed,

        Action_Jump,
        Action_UpDown,
        Action_Bezier,
        Action_LiftFoot,
        Action_SwingAround,
        Action_YawAround,
        Action_SlopeStand,
        Pause,
    } RobotState;
    enum StateOfStanceLeg
    {
        Rfoot_stance,
        Lfoot_stance
    } StepState;
    enum StateOfSwingPhase
    {
        Swing_lift,
        Swing_air,
        Swing_touch
    } SwingState;

    int StepRhythm;
    int RhythmCount;
    double DSPRatio;

    void PatternInit();
    void Step_forward();
    void start_right();
    void start_left();
    void stop();
    void stand();

    double stepH , stepL;
    void Step_forward_stair();
    void start_right_stair();
    void stop_stair();

    double Gradient;
    void Step_forward_slope();
    void start_right_slope();
    void stop_slope();

    bool StepPaused;
    bool DSfinish;
    bool JumpFinished;
    int UpDownCount;
    int JumpCount;
    void Jump();
    void SingleStand_UpDown();
    void SwingAround();
    void Bezier();
    void LiftFoot();
    void YawAround();

    double JY901Pitch, JY901Roll;
    void SlopeStand();

    // private:
    enum StateOfPhase
    {
        FirstPhase,
        SecondPhase
    } StartPhase,StopPhase;
    // int mRhythm;
    double TimeStep;
    double Step_TC_x;
    double Step_TC_y;
    double T_circle;
    double T_DSP;
    double Pendulum_H;
    double RobotYaw;
    double RobotYaw_Temp;
    double RobotYaw_Present;
    double RobotYaw_New;
    double RobotYaw_Old;
    double CoM_x;
    double CoM_y;
    double X0;
    double Y0;
    double V0_x;
    double V0_y;
    double X0_New;
    double Y0_New;
    double V0_x_New;
    double V0_y_New;
    double Rfoot_x_RelaToP;
    double Rfoot_y_RelaToP;
    double Lfoot_x_RelaToP;
    double Lfoot_y_RelaToP;
    double Lfoot_x_RelaToP_Old;
    double Rfoot_x_RelaToP_Old;

    double V0;
    double Angle_VxVy;//重心位于中间时的速度方向角，右腿支撑时，质心速度朝向左前方，规定此时Angle_V0为负
    double CoM_x_RelaToR;
    double CoM_y_RelaToR;
    double CoM_x_RelaToP[100];
    double CoM_y_RelaToP[100];
    double V0_x_RelaToP[100];
    double V0_y_RelaToP[100];
    double D_cal;
    double F_cal;
    double D_cal_new;
    double F_cal_new;
    double D_cal_old;
    double F_cal_old;
    double D_cal_end;
    double F_cal_end;
    double L_cal;
    double Foot_Swing_Angle;
    double Foot_Swing_Angle0;
    double Foot_RLDistance;//默认直行时的落脚点距离
    double Foot_Swing_Angle_multiple;//转动冲击规划
    double RobotYaw_multiple;//转动冲击规划
    double Hip_Yaw_Swing;
    double Hip_Yaw_Stance;

    void PID_CoM();
    void Online_CoM(int mRhythm);
    void CoMPIDinit();

    void RobotYaw_Update();
    void X0_Y0_V0Cal();

    void DdoubleSup(int mRhythm);
    void CoMcal(int mRhythm);
    void Foot_xyzCal(int mRhythm);
    void Foot_xyzCal_stair(int mRhythm);
    void Foot_xyzCal_slope(int mRhythm);

    void KinematicsInit();
    void InverseKinematics();


    ////////side walk//////
    double Y_sidewalk;
    double Y0_begin;
    double Y0_end;
    double V0y_begin;
    double V0y_end;
    double swing_y_old,swing_y_new;
    void start_side();
    void Step_side();
    void stop_side();

    void PatternInit_side();
    void X0_Y0_V0Cal_side();
    void Foot_xyzCal_side(int mRhythm);
    /////////////////////////




    // 调试 矩阵下标 Matrix3x3.h
    enum
    {
        m00 = 0,
        m01,
        m02,
        m10,
        m11,
        m12,
        m20,
        m21,
        m22,
        MAXNUM_ELEMENT
    };
};

}
#endif

