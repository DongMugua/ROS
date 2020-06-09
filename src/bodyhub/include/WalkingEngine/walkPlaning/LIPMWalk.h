/*
LIPMWalk.cpp
*/

#ifndef WALK_HPP
#define WALK_HPP

// #include "Kinematics.h"
#include <Eigen/Dense>
#include <type_traits>

#include "CPWalking5.h"
#include "Kalman.h"
#include "SimControll.h"
#include "TalosRobot.h"

// #define SIM_ROBOT
#define CpPlan

#define Pi 3.141592653

using namespace ljnoid;

template <class V>
constexpr V sqr(const V &a) {
  return a * a;
}

typedef Eigen::Matrix<double, 6, 1> Vector6d;

namespace GaitManager {
class LIPMWalk {
 public:
  LIPMWalk();
  ~LIPMWalk();
  CPWalking5 *cpWalk_lipm;
  struct ParameterOfPosture {
    //定义右手系，机器人前方为x轴正方形，左方为y轴正方向，上方为z轴正方向
    double Lfoot_x, Lfoot_y, Lfoot_z, Lfoot_R, Lfoot_P, Lfoot_Y;
    double Rfoot_x, Rfoot_y, Rfoot_z, Rfoot_R, Rfoot_P, Rfoot_Y;
    double Torso_x, Torso_y, Torso_z, Torso_R, Torso_P, Torso_Y, TorsoCoM_z,
        TorsoCoM_x;
    double LArm_R, LArm_P, LArm_elbow, RArm_R, RArm_P, RArm_elbow;
  };
  void simStart();
  void initSquat(std::vector<Eigen::VectorXd> &);
  bool start(double x, int targetstepcount, double *waypoint_Yaw);
  Eigen::Matrix4d LeftLegPosture, RightLegPosture;
  void ForwardKinematics(Eigen::VectorXd jointvalue);
  void SwingAround();
  void YawAround(int YawAmplitude = 0,
                 int RollAmplitude = 80);  //手臂摆动，带默认参数
  void computerJointValue();
  Eigen::Matrix3d Euler_2_rotation(Eigen::Matrix<double, 3, 1> euler);
  double CubicSplineVel(Vector6d *In, double t);  // for debug
  Eigen::Vector2d refComVel = Eigen::Vector2d::Zero();

  Eigen::Vector2d comVel = Eigen::Vector2d::Zero();

  double footSeprate;  // foot seprate at standing for footprint plan

  TalosRobot talosRobot;
  Kalman *leftTorsofilterx;
  Kalman *rightTorsofilterx;
  Kalman *leftTorsofiltery;
  Kalman *rightTorsofiltery;
  void estimateComAndComv();

  Kalman *torsofilterx;
  Kalman *torsofiltery;

  double torsoHeight;      // torso height from forward kinematics
  double torsoHeightWalk;  // torso height at walking

  double timeStep;
  Eigen::VectorXd jointValue;
  Eigen::VectorXd lastJointValue;
  Eigen::VectorXd jointVelocity;
  Eigen::VectorXd measuredJointValue;
  Eigen::VectorXd lastMeasuredJointValue;
  Eigen::VectorXd measuredJointVelocity;
  Eigen::VectorXd armJointValue;
  int armSwingCount = 0;
  int swingMax = 20;
  int squatStep = 50;
  int squatCount = 0;
  enum jointId {
    LLEG_JOINT_START = 0,
    LLEG_JOINT_END = 6,
    LLEG_JOINT_NUM = 6,

    RLEG_JOINT_START = 6,
    RLEG_JOINT_END = 12,
    RLEG_JOINT_NUM = 6,

    LARM_JOINT_START = 12,
    LARM_JOINT_END = 15,
    LARM_JOINT_NUM = 3,

    RARM_JOINT_START = 15,
    RARM_JOINT_END = 18,
    RARM_JOINT_NUM = 3,

    JOINT_NUM = 12,

    FLOATING_FREEDOM_NUM = 6,
    FLOATING_CONFIG_NUM = 7,
    JOINT_FREEDOM_NUM = 18,
    JOINT_CONFIG_NUM = 19,

    LLEG_JOINT_FREEDOM_NUM = 12,  // 6,//
    LLEG_JOINT_CONFIG_NUM = 13,   // 6,//

  };

  Eigen::Vector3d measuredVel = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredVelocityTorsoInLeft = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredVelocityTorsoInRight = Eigen::Vector3d::Zero();
  Eigen::Vector3d estimatedVelocityTorsoInLeft = Eigen::Vector3d::Zero();
  Eigen::Vector3d estimatedVelocityTorsoInRight = Eigen::Vector3d::Zero();
  Eigen::Vector3d estimatedCom = Eigen::Vector3d::Zero();
  Eigen::Vector3d estimatedComVel = Eigen::Vector3d::Zero();

  Eigen::Vector3d measuredTorsoInLeft = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredTorsoInRight = Eigen::Vector3d::Zero();
  Eigen::Vector3d estimatedTorsoInLeft = Eigen::Vector3d::Zero();
  Eigen::Vector3d estimatedTorsoInRight = Eigen::Vector3d::Zero();

  Eigen::Vector3d lastMeasuredTorsoInLeft = Eigen::Vector3d::Zero();
  Eigen::Vector3d lastMeasuredTorsoInRight = Eigen::Vector3d::Zero();
  Eigen::Vector3d lastEstimatedTorsoInLeft = Eigen::Vector3d::Zero();
  Eigen::Vector3d lastEstimatedTorsoInRight = Eigen::Vector3d::Zero();

  Eigen::Vector6d cubicsplineFootPx, cubicsplineFootAx, cubicsplineFootPy,
      cubicsplineFootAy, cubicsplineFootPz1, cubicsplineFootAz1,
      cubicsplineFootPz2, cubicsplineFootAz2;

  Eigen::Vector2d measStanFootInWorld;
  Eigen::Vector2d coordiSInWorld;  // coordinate S in world
  Eigen::Vector2d convertVecInFootToWorld(Eigen::Vector2d vec);
  Eigen::Vector2d convertVecInSToWorld(Eigen::Vector2d vec);
  Eigen::Vector3d convertVecInPToWorld(Eigen::Vector3d vec,
                                       Eigen::Vector3d footInWorld);
  Eigen::Vector3d convertVecInWToP(Eigen::Vector3d vec,
                                   Eigen::Vector3d footInWorld);
  Eigen::Vector2d convertVecInWToPVelocity(Eigen::Vector2d vec,
                                           Eigen::Vector3d footInWorld);
  void estimateComAndComv1();
  Eigen::Vector3d measuredComInWorld = Eigen::Vector3d::Zero();
  Eigen::Vector3d lastMeasuredComInWorld = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredComVelocityInWorld = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredVelocityComInLeft = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredVelocityComInRight = Eigen::Vector3d::Zero();
  Eigen::Vector3d estimatedComInWorld = Eigen::Vector3d::Zero();
  Eigen::Vector3d estimatedComVelocityInWorld = Eigen::Vector3d::Zero();
  // Eigen::Vector3d estimatedCom = Eigen::Vector3d::Zero();
  // Eigen::Vector3d estimatedComVel = Eigen::Vector3d::Zero();

  Eigen::Vector3d measuredComInLeft = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredComInRight = Eigen::Vector3d::Zero();
  Eigen::Vector3d comRefInWorld;
  Eigen::Vector3d lastComRefInWorld;
  Eigen::Vector3d comvRefInWorld;

  int DoubleContactCount = 0;

  void run();

  double RRoffset = 0;
  double LLoffset = 0;
  double JY901Pitch, JY901Roll;
  int FB_GYRO, RL_GYRO, FB_ACCEL, RL_ACCEL;
  double Lfoot_P_imp, Rfoot_P_imp, Lfoot_R_imp, Rfoot_R_imp;
  int FSR_L[6], FSR_R[6];
  double CoM_Vx, CoM_Vy;
  double CoM_x_RelaToW[10], CoM_y_RelaToW[10];
  double CoM_x_ideal[10], CoM_y_ideal[10];
  double CoM_x_measured, CoM_y_measured, CoM_H_measured;
  double error_com_x, error_com_y, error_com_x_last, error_com_y_last;
  int MeasureLagCircle;
  bool JointReadOK;
  bool IsAdjusted;
  bool ActionDone;
  bool IsWalking;

  double WayPoint_Yaw[1000];
  double WayPoint_Y0[1000];
  double WayPoint_X0[1000];
  int StepCount, virtualStepCount;
  int StepCountTarget;
  double Hip_Yaw_L, Hip_Yaw_R;
  double Lfoot_H, Rfoot_H;
  double Swing_H;
  double JointValue[21];  //二十个舵机  0号不用
  double CurrentJointValue[21];

  double CurrentRemainTime;
  double Lfoot_x, Lfoot_y, Lfoot_z, Lfoot_R, Lfoot_P, Lfoot_Y;
  double Rfoot_x, Rfoot_y, Rfoot_z, Rfoot_R, Rfoot_P, Rfoot_Y;
  double Torso_x, Torso_y, Torso_z, Torso_R, Torso_P, Torso_Y;

  double LArm_Rdf = -80, LArm_Pdf = -0, LArm_elbowdf = -0;  //手臂关节角度初始值
  double RArm_Rdf = -80, RArm_Pdf = -0, RArm_elbowdf = -0;
  double LArm_R = LArm_Rdf, LArm_P = LArm_Pdf,
         LArm_elbow = LArm_elbowdf;  //手臂关节角度
  double RArm_R = RArm_Rdf, RArm_P = RArm_Pdf, RArm_elbow = RArm_elbowdf;

  enum StateOfRobot {
    Walk_side,
    Walk_start_side,
    Walk_stop_side,

    Walk_forward,
    Walk_start,
    Walk_stop,
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
  enum StateOfStanceLeg { Rfoot_stance, Lfoot_stance } StepState;
  enum StateOfSwingPhase { Swing_lift, Swing_air, Swing_touch } SwingState;

  //实际为重心到脚底的距离矢量
  Eigen::Vector3d measuredLeftToCom = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredRightToCom = Eigen::Vector3d::Zero();
  Eigen::Vector2d measuredComVelocity = Eigen::Vector2d::Zero();

  Eigen::Vector3d expectedLeftToCom = Eigen::Vector3d::Zero();
  Eigen::Vector3d expectedRightToCom = Eigen::Vector3d::Zero();
  Eigen::Vector2d expectedComVelocity = Eigen::Vector2d::Zero();

  Eigen::Vector3d estimatedLeftToCom = Eigen::Vector3d::Zero();
  Eigen::Vector3d estimatedRightToCom = Eigen::Vector3d::Zero();
  Eigen::Vector2d estimatedComVelocity = Eigen::Vector2d::Zero();

  Eigen::Vector3d lastEstimatedLeftToCom = Eigen::Vector3d::Zero();
  Eigen::Vector3d lastEstimatedRightToCom = Eigen::Vector3d::Zero();

  Eigen::Vector3d lastExpectedLeftToCom = Eigen::Vector3d::Zero();
  Eigen::Vector3d lastExpectedRightToCom = Eigen::Vector3d::Zero();
  Eigen::Vector2d lastExpectedComVelocity = Eigen::Vector2d::Zero();

  // private:
  constexpr const static double Gravity = 9.8;
  double Step_TC_x;
  double Step_TC_y;
  double T_circle, T_dsp;
  double DSPRatio;
  double DSP_x;
  double DSP_y;
  // double Pendulum_H;
  // double CoM_x;
  // double CoM_y;
  double X0;
  double Y0;
  double V0_x;
  double V0_y;
  void PatternInit();

  enum StateOfContact {
    DoubleContact = 0,
    LeftContact = -1,
    RightContact = 1,
    UnstableContact = 2
  } ContactState,
      LastContactState;
  void GroundContactDetect();

  enum PhaseOfStep {
    DoubleSupport = 0,
    LeftStance = -1,
    RightStance = 1
  } StepPhase,
      LastStepPhase, VirtualStepPhase, VirtualLastStepPhase;
  ;
  enum ChangeOfPhase {
    NoChange,
    DtoL,
    DtoR,
    LtoD,
    RtoD
  } PhaseChange,
      VirtualPhaseChange;
  void updateStepPhase();

  void updateStepParameter();
  void computerCurrentPosture();

  Eigen::Vector4d observerProcessDeviation = {
      0.1, 0.1, 3, 3}; /**< The noise of the filtering process that estimates
                          the position of the center of mass */
  Eigen::Vector2d observerMeasurementDeviation = {
      2, 2}; /**< The measurement uncertainty of the computed "measured"
                center of mass position */
  Eigen::Matrix3d covX = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d covY = Eigen::Matrix3d::Identity();
  void computeEstimatedPosture();

  int VirtualPendulumCount, PendulumCount, PendulumCountEnd;
  void updatePendulum();

  // Yaw角为S坐标系下
  double RobotYaw_Torso, RobotYaw_Pendulum, RobotYaw_Present, RobotYaw_New,
      RobotYaw_Old;
  ParameterOfPosture PosPara, LastPosPara, CurrentPos, LastPos, TouchPos;
  double VirtualRobotYaw_Torso, VirtualRobotYaw_Pendulum,
      VirtualRobotYaw_Present, VirtualRobotYaw_New, VirtualRobotYaw_Old;
  int DScount, lastDScount;
  double DSvelo_x, DSvelo_y;
  Eigen::Vector6d cubicsplinePx, cubicsplineAx, cubicsplinePy, cubicsplineAy;
  void CubicSplineInit(Eigen::Vector6d *In, Eigen::Vector6d *Out, double t);
  double CubicSpline(Eigen::Vector6d *In, double t);
  void CubicSplineSixInit(Eigen::Vector6d *In, Matrix<double, 7, 1> *Out,
                          double t, double alpha);
  double CubicSplineSix(Matrix<double, 7, 1> *In, double t);
  void generatePosture();
  void TransformPtoS(double Xin, double Yin, double *Xout, double *Yout);
  // void TransformTtoP(double Xin, double Yin, double *Xout, double *Yout);
  // void LIPMWalk::TransformTtoP(double Xin, double Yin, double *Xout, double
  // *Yout, double YAWin, double *YAWout);
  void TransformTtoP(double Xin, double Yin, double *Xout, double *Yout,
                     double YAWin, double *YAWout);

  static const double Rad2Deg;

  int StepRhythm;
  int RhythmCount;

  double X0_New, Y0_New, V0_x_New, V0_y_New;
  double Rfoot_x_RelaToP, Rfoot_y_RelaToP, Lfoot_x_RelaToP, Lfoot_y_RelaToP,
      Lfoot_x_RelaToP_Old, Rfoot_x_RelaToP_Old;

  double V0,
      Angle_VxVy;  //重心位于中间时的速度方向角，右腿支撑时，质心速度朝向左前方，规定此时Angle_V0为负
  double CoM_x_RelaToR, CoM_y_RelaToR;
  double CoM_x_RelaToP[100];
  double CoM_y_RelaToP[100];
  double V0_x_RelaToP[100];
  double V0_y_RelaToP[100];
  // planned data
  struct robotCoM {
    double CoM_x_RelaToP[100];
    double CoM_y_RelaToP[100];
    double V0_x_RelaToP[100];
    double V0_y_RelaToP[100];
  } plannedCoM[80];  // 20 steps maxium

  // Eigen::Vector2d footPrint[30];
  std::vector<Eigen::Vector3d> footPrint;

  // world frame 定义世界坐标系的原点为起始位置时重心的投影位置，与第一个S系重合
  struct CoM_w_frame {
    double CoMx[200];
    double CoMy[200];
    double V0_x[200];  // not using
    double V0_y[200];  // not using
  };                   // plannedCoM_wF[20]; // 20 steps maxium
  std::vector<CoM_w_frame> plannedCoM_wF;

  ParameterOfPosture PosPara_wF, LastPosPara_wF, CurrentPos_wF, CurrentPosInP,
      LastPos_wF, TouchPos_wF, CommandPos_wF,
      CurrentPosInPtest;  // defined in world frame
  Eigen::Vector6d cubicsplinePyaw_wF, cubicsplineAyaw_wF, cubicsplinePx_wF,
      cubicsplineAx_wF, cubicsplinePy_wF,
      cubicsplineAy_wF;  // world frame spline
  Eigen::Vector6d cubicsplinePx_InP, cubicsplineAx_InP, cubicsplinePy_InP,
      cubicsplineAy_InP;
  Eigen::Vector2d coordiSInWorld_sim;  //用于COM生成时候的坐标系转换

  void SimTransformPtoS(double Xin, double Yin, double *Xout, double *Yout);
  void FootPrintOutput(int CurrentStepCount);
  ParameterOfPosture convertPosStoW(ParameterOfPosture posInS);
  ParameterOfPosture convertPosPtoW(ParameterOfPosture posInP,
                                    Eigen::Vector3d fpInWorld);
  ParameterOfPosture convertPosWtoP(
      ParameterOfPosture posInP);  // auto select footPrint
  ParameterOfPosture convertPosWtoP(ParameterOfPosture posInW,
                                    Eigen::Vector3d fpInWorld);
  Eigen::Vector2d simconvertVecInSToWorld(Eigen::Vector2d vec);
  Eigen::Vector6d cubicsplineFootPx_wf, cubicsplineFootAx_wf,
      cubicsplineFootPy_wf, cubicsplineFootAy_wf, cubicsplineFootPz1_wf,
      cubicsplineFootAz1_wf, cubicsplineFootPz2_wf, cubicsplineFootAz2_wf,
      cubicsplineFootPyaw_wf, cubicsplineFootAyaw_wf;
  // VectorXd FootTraj;
  Matrix<double, 7, 1> cubicsplineFootAxSix_wf;

  //%%%%%%%%%%
  double D_left, D_right;
  double F_left, F_right;
  double D_cal_new;
  double F_cal_new;
  double D_cal_old;
  double F_cal_old;
  double D_cal_end;
  double F_cal_end;
  double L_cal;
  double Foot_Swing_Angle;
  double Foot_Swing_Angle0;
  double Foot_RLDistance;            //默认直行时的落脚点距离
  double Foot_Swing_Angle_multiple;  //转动冲击规划
  double RobotYaw_multiple;          //转动冲击规划
  double Hip_Yaw_Swing;
  double Hip_Yaw_Stance;
  // double Swing_x, Swing_y, Swing_z;
  void PID_CoM();
  void Online_CoM(int mRhythm);
  void CoMPIDinit();

  void X0_Y0_V0Cal();
  void RobotYaw_Update();

  void DdoubleSup(int mRhythm);
  void CoMcal(int mRhythm);
  void Foot_xyzCal(int mRhythm);
  void Foot_xyzCal_stair(int mRhythm);
  void Foot_xyzCal_slope(int mRhythm);

  // virtual plan part
  void offlineCOMplan();
  void offlineStepParaInit(int CurrentStepCount);
  void offlineupdatePendulum(int CurrentStepCount, int currentPendulumCount);
  void LoadPlanDateFromCP(void);
  void HipOffset(void);
  bool lastOnwalkFlag = false;
};
}
#endif
