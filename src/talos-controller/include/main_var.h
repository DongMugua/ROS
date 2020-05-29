#include "cmd_process.h"

#include "LIPMWalk.h"
#include "Kalman.h"
#include "JY901.h"
#include "SimpleSerial.h"

#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace GaitManager ;
using namespace Robot;
using namespace std;

#ifndef MAINVAR
#define MAINVAR
LinuxCM730 linux_cm730("/dev/ttyUSB0");
CM730 cm730(&linux_cm730);
int gID = CM730::ID_CM;

///////////////////////////////IMU//////////////////////////////////////////
Kalman kalmanX;//X方向为Roll
Kalman kalmanY;//Y方向为Pitch

bool kalmanStartFlag = false;
double  KalTimer;
/* IMU Data */
int accX, accY, accZ;
int gyroX, gyroY, gyroZ;
double zeroValue[6] ; // gyroX, gyroY, gyroZ, accX, accY, accZ

/* All the angles start at 180 degrees */
double gyroXangle = 180;
double gyroYangle = 180;
double gyroZangle = 180;
// Complimentary filter
double compAngleX = 180;
double compAngleY = 180;
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double torsoRoll, torsoPitch;
double JY901Roll, JY901Pitch;
///////////////////////////////IMU//////////////////////////////////////////

///////////////////////////////LIPM//////////////////////////////////////////
LIPMWalk mWalk;
int TimeUsed;
struct timeval LIPM_start;
struct timeval LIPM_end;

int  JointOffset[21] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int  ControlOffset[21] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int param[JointData::NUMBER_OF_JOINTS * 5];
int idealvalue[21] = {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048};
int controlvalue[21] = {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048};
int measuredvalue [21] = {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048};
int error[21] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int id;
int speedvalue;

double CoM_x_m, CoM_y_m, CoM_H_m;
double CoM_x_m_old;
double CoM_x_caled, CoM_y_caled, CoM_H_caled;
double MeasuredAngle[21];
double odometer_x=0, odometer_y=0;

JointData mJOINT;

double  BALANCE_KNEE_GAIN = 0.3;
double  BALANCE_ANKLE_PITCH_GAIN = 0.9;
double  BALANCE_HIP_ROLL_GAIN = 0.5;
double  BALANCE_ANKLE_ROLL_GAIN = 1.0;
///////////////////////////////LIPM//////////////////////////////////////////

///////////////////////////////ROS//////////////////////////////////////////
ros::Publisher error_x;
ros::Publisher error_y;
ros::Publisher v_x_c;
ros::Publisher v_x_m;

ros::Publisher com_x_m;
ros::Publisher com_x_c;
ros::Publisher com_x_i;
ros::Publisher com_y_m;
ros::Publisher com_y_c;

ros::Publisher JY901X;
ros::Publisher JY901Y;

ros::Publisher torsoroll;
ros::Publisher torsopitch;
ros::Publisher StepRhythm;

ros::Publisher stepOK;
///////////////////////////////ROS//////////////////////////////////////////
#endif
