#ifndef BODYHUB_H
#define BODYHUB_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <signal.h>
#include <sys/time.h>  //int gettimeofday(struct timeval *, struct timezone *);
#include <termios.h>   //struct termios
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <boost/thread.hpp>
#include <queue>
#include <thread>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

#include <std_msgs/Int32.h>
#include "bodyhub/JointControlPoint.h"
#include "bodyhub/SensorControl.h"
#include "bodyhub/SensorRawData.h"
#include "bodyhub/ServoPositionAngle.h"
#include "bodyhub/SrvInstRead.h"
#include "bodyhub/SrvInstWrite.h"
#include "bodyhub/SrvServoAllRead.h"
#include "bodyhub/SrvServoAllWrite.h"
#include "bodyhub/SrvServoRead.h"
#include "bodyhub/SrvServoWrite.h"
#include "bodyhub/SrvState.h"
#include "bodyhub/SrvString.h"
#include "bodyhub/SrvSyncWrite.h"
#include "bodyhub/SrvTLSstring.h"
#include "bodyhubStatus.h"
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/UInt16.h"

#include "WalkingEngine/walkPlaning/CPWalking5.h"
#include "WalkingEngine/walkPlaning/LIPMWalk.h"

#include "ExtendSensor.h"
#include "SimControll.h"

#define DEBUG
// #define SIM_ROBOT
#define PI acos(-1)
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

#define JOINT_SERVO_NUM 20
#define HEAD_SERVO_NUM 2
#define SERVO_NUM (JOINT_SERVO_NUM + HEAD_SERVO_NUM)

#define DXL_ID_SCAN_RANGE 25               // Dynamixel ID scan range
#define DXL_ID_COUNT_MAX (SERVO_NUM + 20)  // Dynamixel ID count

#define SIZE_LIMIT(value, limit) (((value) > (limit)) ? (limit) : (value))
#define SERVO_DEFAULT_VALUE 2048

struct MotoPoint {
  double position;  // radian
  double velocity;
  double acceleration;
};

// extern
extern std::string sensorNameIDFile;

extern pthread_mutex_t mtxMo;
extern pthread_mutex_t mtxHe;
extern pthread_mutex_t mtxWl;
extern pthread_mutex_t MutexBulkRead;

extern DynamixelWorkbench dxlTls;

extern std::queue<bodyhub::JointControlPoint> motoQueue;
extern std::queue<bodyhub::JointControlPoint> headCtrlQueue;

extern uint8_t bodyhubState;

// function
double Angle2Radian(double angle);

void UpdateState(uint8_t stateNew);
void ClearTimerQueue(void);
void addSensorModule(SensorModule *tempSensor);
void removeSensorModule(SensorModule *tempSensor);
bool ServoBulkRead(uint8_t *bulkReadID, uint8_t readCount, std::string itemName,
                   int32_t *bulkReadData);
bool RawBulkRead(uint8_t *bulkReadID, uint8_t readCount,
                 uint16_t *bulkReadAddress, uint16_t *bulkReadLength,
                 int32_t *bulkReadData);

bool SensorBulkWrite(uint8_t WriteCount, uint8_t *bulkWriteID,
                     uint16_t *bulkWriteAddress, uint16_t *bulkWriteLenght,
                     int32_t *bulkWriteData);

void control_thread_robot(void);

#endif