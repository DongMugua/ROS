#include <pthread.h>
#include <iostream>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

#include "SimControll.h"

#include "LIPMWalk.h"
// #include "Walking.h"
// #include "CPWalk.h"
// #include "CPWalking.h"
// #include "CPWalking1.h"
// #include "CPWalking2.h"
#include "CPWalking5.h"
#include "Kinematics.h"

#include "JY901.h"
#include "SimpleSerial.h"
#include "serial/serial.h"

#include <std_msgs/Float64MultiArray.h>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"

#include "Kalman.h"

using namespace GaitManager;
using namespace std;

int stepcountTarget;
// ///////////////////////////////LIPM//////////////////////////////////////////
LIPMWalk mWalk;
// CPWalk* cpWalk;
// CPWalking* cpWalk;
// CPWalking1* cpWalk;
// CPWalking2* cpWalk;
CPWalking5* cpWalk;

#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION 0
#define SYNC_READ_HANDLER_FOR_PRESENT_VELOCITY 1
#define BULK_READ_PRESENT_POSITION_ADDR 36
#define BULK_READ_PRESENT_POSITION_LEN 2
#define RIGHT_FSR_ID 112
#define LEFT_FSR_ID 111
#define FSR_ADDR 90
#define FSR_ADDR_LEN 4

DynamixelWorkbench dxl_wb;
std::vector<double> motoDataValuePre = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

uint8_t idArray[30];
uint8_t idCnt = 0;

uint8_t scanRange = 25;
uint8_t scannedId[30];
uint8_t dxlIdCnt = 0;

double measuredJointPos[30];
double measuredJointPWM[30];
double measuredJointCurrent[30];
double measuredJointVel[30];

Kalman* measuredJointFilter[30];

void jointFilter();
void jointFilterInit();

// int assembleOffset[22] = {-31,17,-2,-159,-73,-158,   20,9,-115,3,52,70,
// -60,-793,-148,29,1052,82,150,-25, 17, 7};//3号

// int assembleOffset[22] = {45,-8,-38,-38,-95,-50,   -43,0,57,52,36,74,
// 81,-184,52,32,0,-160,-129,0, 0, 0};//0号

int assembleOffset[22] = {-31,  46,  -34, 54, -101, 17,  20,   -138,
                          -44,  -84, 106, 79, -98,  107, -108, 41,
                          -121, 199, 132, 0,  0,    0};  // 5号

// int assembleOffset[22] = {-22,-96,1,-115,-4,1202,   0,66,-123,-65,-43,41,
// -1293,-52,6,42,20,19,170,-42, 35, 260};//6号

// int assembleOffset[22] = {71,118,128,-92,38,20,   -12,23,38,20,-62,8,
// 0,210,0, -136,0,97,0,0,  0,35};//###### Wed Aug 21 16:44:55 CST 2019

// int assembleOffset[20] = {0,-20,190,20,40,20,
// 						  -20,38,-140,-40,-40,-8, 0,0,0,
// 0,0,0,	0,0};//add mass to 3.38	 fsr  NoDS

// int assembleOffset[20] = {0,20,260,20,40,45,
// 							  -20,0,-220,-40,-40,-45,
// 0,0,0,
// 0,0,0,	0,0};//add mass to 5.38	 squat

int angleDirection[22] = {1,  1, -1, -1, 1, -1, 1, 1, 1, 1, -1,
                          -1, 1, 1,  1,  1, 1,  1, 1, 1, 1, 1};

bool initSDKHandlers(void);
bool initDxlWorkbench(void);

bool sendDataToDxl(void);
// void initSquat();
void squat();
// void squatTest();
void* control_thread(void* ptr);

void* control_thread_robot(void* ptr);

///////JY901/////////////
void* JY901_thread(void* ptr);
// SimpleSerial serial_JY901("/dev/ttyUSB0",115200);//JY901串口处理
// 启用JY901时取消注释

// serial::Serial serial_JY901("/dev/ttyUSB0", 115200,
// serial::Timeout::simpleTimeout(1000),
// 	serial::eightbits,serial::parity_none,serial::stopbits_one,serial::flowcontrol_none);
double JY901Roll, JY901Pitch;

// ///////////////////////////////ROS//////////////////////////////////////////
ros::Publisher jointPosTargetPub;
ros::Publisher jointPosMeasurePub;

std_msgs::Float64MultiArray jointPos;

ros::Publisher jointVelTargetPub;
ros::Publisher jointVelMeasurePub;

std_msgs::Float64MultiArray jointVel;

std_msgs::Float64MultiArray footTraj;

ros::Publisher cpref_pub;
ros::Publisher cpC_pub;

ros::Publisher copm_pub;
ros::Publisher copD_pub;
ros::Publisher copref_pub;

ros::Publisher comm_pub;
ros::Publisher comD_pub;
ros::Publisher comref_pub;
ros::Publisher comEsti_pub;

ros::Publisher comvm_pub;
ros::Publisher comvD_pub;
ros::Publisher comvref_pub;
ros::Publisher comvEsti_pub;

ros::Publisher LFootZ;
ros::Publisher RFootZ;
ros::Publisher footDisRef;
ros::Publisher footDis;

ros::Publisher leftFootTraj_pub;
ros::Publisher rightFootTraj_pub;

ros::Publisher contactState_pub;
ros::Publisher stepPhase_pub;

ros::Publisher JY901X;
ros::Publisher JY901Y;

ros::Publisher PLANfoot_pub;
ros::Publisher Rfoot_pub;
ros::Publisher Lfoot_pub;
ros::Publisher Lyfoot_pub;
ros::Publisher Lxfoot_pub;
ros::Publisher Ryfoot_pub;
ros::Publisher Rxfoot_pub;
ros::Publisher PLANRfoot_pub;
ros::Publisher PLANLfoot_pub;

ros::Publisher Torso_Ppub;
ros::Publisher Torso_Rpub;

ros::Publisher RRoffset_pub;
ros::Publisher LLoffset_pub;

void robotStatePublish();
void advertise();

void RobotStand(uint8_t* idGet, uint8_t idCnt, int32_t velocity);
bool BulkReadFromDxl();
bool SensorBulkRead(uint8_t* bulkReadID, uint8_t readCount,
                    uint16_t* bulkReadAddress, uint16_t* bulkReadLength,
                    int32_t* bulkReadData);
// ///////////////////////////////ROS//////////////////////////////////////////
