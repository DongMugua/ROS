#include "bodyhub/bodyhub.h"

uint8_t DxlIdList[DXL_ID_COUNT_MAX];
uint8_t DxlIdCount = 0;
uint8_t timerClosed = 1;  // timer 20ms 关闭
uint8_t masterIDControl = 0;
uint8_t bodyhubState = 0;
std::string stateNewStr;

std::string offsetFile;
std::string InitPoseFile;
std::string sensorNameIDFile;

int stepCount = 5;
bool gaitStart = true;
int countCommand = 20;
int gaitFlagDisplay = 0;
double measuredJointPos[30];
#define BULK_READ_PRESENT_POSITION_ADDR 36
#define BULK_READ_PRESENT_POSITION_LEN 2
#define RIGHT_FSR_ID 112
#define LEFT_FSR_ID 111
#define FSR_ADDR 90
#define FSR_ADDR_LEN 4
#define MediMotoAlpha 12.80
#define SmalMotoAlpha 18.61

bool WalkingSendData(void);
bool WalkingReceiveData(void);
void WalkingStatePublish(void);

int WalkingAngleDirection[SERVO_NUM] = {1,  1, -1, -1, 1, -1, 1, 1, 1, 1, -1,
                                        -1, 1, 1,  1,  1, 1,  1, 1, 1, 1, 1};
float AngleAlpha[SERVO_NUM] = {
    MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha,
    MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha,
    MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, SmalMotoAlpha, SmalMotoAlpha,
    MediMotoAlpha, SmalMotoAlpha, SmalMotoAlpha, SmalMotoAlpha, SmalMotoAlpha,
    SmalMotoAlpha, SmalMotoAlpha};

std::vector<double> motoDataValuePre = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

double StandPositionAngle[SERVO_NUM] = {0, 0, 0,   0,   0, 0,  0,  0, 0, 0, 0,
                                        0, 0, -75, -10, 0, 75, 10, 0, 0, 0, 0};

double ServoOffsetValue[SERVO_NUM] = {-23, 32,  -68, 49, -101, 2,   21,   -158,
                                      -26, -60, 137, 74, 102,  107, -108, 41,
                                      0,   0,   132, 0,  0,    0};

// 舵机下发数据记录存储
std::vector<double> ServoAngleStore = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

std::vector<double> ServoRawValueStore = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

std::queue<bodyhub::JointControlPoint> motoQueue;
std::queue<bodyhub::JointControlPoint> headCtrlQueue;

DynamixelWorkbench dxlTls;
GaitManager::LIPMWalk mWalk;      // walking
GaitManager::CPWalking5 *cpWalk;  // walking

//创建互斥锁
pthread_mutex_t mtxMo;
pthread_mutex_t mtxHe;
pthread_mutex_t mtxWl;
pthread_mutex_t mtxSL;
pthread_mutex_t MutexBulkRead;

std_msgs::UInt16 budyhubStateMsg;

//话题
std_msgs::Float64MultiArray jointPos;
std_msgs::Float64MultiArray jointVel;
std_msgs::Float64MultiArray footTraj;
ros::Publisher jointPosTargetPub;
ros::Publisher jointPosMeasurePub;
ros::Publisher jointVelTargetPub;
ros::Publisher jointVelMeasurePub;
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
// ros::Publisher JY901X;
// ros::Publisher JY901Y;
ros::Publisher WalkingStatusPub;

ros::Publisher StatusPub;
ros::Publisher ServoPositionPub;

//服务
ros::ServiceServer StateService;
ros::ServiceServer MasterIDService;
ros::ServiceServer GetStatusService;
ros::ServiceServer GetJointAngleService;

ros::ServiceServer InstReadValService;
ros::ServiceServer InstWriteValService;
ros::ServiceServer SyncWriteValService;
ros::ServiceServer SetTarPositionValService;
ros::ServiceServer SetTarPositionValAllService;
ros::ServiceServer GetPositionValAllService;

ros::ServiceServer InstReadService;
ros::ServiceServer InstWriteService;
ros::ServiceServer SyncWriteService;
ros::ServiceServer SetLockStateService;
ros::ServiceServer SetLockStateAllService;
ros::ServiceServer GetLockStateAllService;
ros::ServiceServer SetTarPositionService;
ros::ServiceServer SetTarPositionAllService;
ros::ServiceServer GetPositionAllService;

void vectorOut(std::vector<double> &vector_in) {
  /* printf vector double */
  for (unsigned int i = 0; i < vector_in.size(); i++) {
    if (i == vector_in.size() - 1) {
      std::cout << vector_in[i] << std::endl;
    } else
      std::cout << vector_in[i] << ",";
  }
  std::cout << std::endl;
}

void ClearQueue(std::queue<bodyhub::JointControlPoint> &Qtempt) {
  if (!Qtempt.empty()) {
    std::queue<bodyhub::JointControlPoint> empty;
    swap(empty, Qtempt);
  }
}

int getch() {
// 获取按键
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

double Angle2Radian(double angle) { return (angle * PI) / 180; }
double Radian2Angle(double Radian) { return (Radian * 180) / PI; }
double convertValueTOAngle(uint8_t dxlID, int motoValue) {
  return ((motoValue - 2048) / AngleAlpha[dxlID - 1]);
}  // value_to_angle
int convertAngleTOValue(uint8_t dxlID, double motoAngle) {
  return (motoAngle * AngleAlpha[dxlID - 1] + 2048);
}  // angle_to_value

void control_thread_robot() {
  if (pthread_mutex_trylock(&mtxWl) != 0) return;

  int count = 0;

  WalkingReceiveData();  // instead of  receiveDataFromDxl()
  // getchar();
  cpWalk->run();
  mWalk.run();

  WalkingSendData();
  WalkingStatePublish();

  // receiveDataFromDxl();
  // jointFilter();
  // for(int i=0;i<12;i++)
  // {
  // 	cpWalk->measuredJointValue[i]=measuredJointPos[i];
  // 	// cpWalk->measuredJointVelocity[i]=measuredJointVel[i];
  // }
  // cpWalk->measuredJointVelocity=(cpWalk->measuredJointValue-cpWalk->lastMeasuredJointValue)/cpWalk->timeStep;
  // cpWalk->lastMeasuredJointValue.segment(0,12)=cpWalk->measuredJointValue.segment(0,12);
  // cpWalk->talosRobot.measuredmbc.q=sVectorToParam(cpWalk->talosRobot.talos,cpWalk->measuredJointValue.segment(0,12)*Util::TO_RADIAN);
  // cpWalk->talosRobot.measuredmbc.alpha=sVectorToDof(cpWalk->talosRobot.talos,cpWalk->measuredJointVelocity.segment(0,12)*Util::TO_RADIAN);

  pthread_mutex_unlock(&mtxWl);
  for (int i = 0; i < 12; i++)
    mWalk.measuredJointValue[i] = measuredJointPos[i];
  mWalk.measuredJointVelocity =
      (mWalk.measuredJointValue - mWalk.lastMeasuredJointValue) /
      mWalk.timeStep;
  mWalk.lastMeasuredJointValue.segment(0, 12) =
      mWalk.measuredJointValue.segment(0, 12);
  mWalk.talosRobot.measuredmbc.q =
      sVectorToParam(mWalk.talosRobot.talos,
                     mWalk.measuredJointValue.segment(0, 12) * Util::TO_RADIAN);
  mWalk.talosRobot.measuredmbc.alpha = sVectorToDof(
      mWalk.talosRobot.talos,
      mWalk.measuredJointVelocity.segment(0, 12) * Util::TO_RADIAN);

  count++;
  // do some thing
}

void threadTimer() {
  if (SimControll::simEnable) return;
  ROS_INFO("Start 'threadTimer' thrand...");

  static struct timespec nextTime;
  struct timespec realTime, lastrealTime;
  double loopTimeout;
  clock_gettime(CLOCK_MONOTONIC, &nextTime);

  const char *log = NULL;
  bool result = false;
  std::vector<double> ServoRadianStore;

  while (1) {
    // timeset
    nextTime.tv_sec = lastrealTime.tv_sec +
                      (lastrealTime.tv_nsec + 10 * 1000000) / 1000000000;
    nextTime.tv_nsec = (lastrealTime.tv_nsec + 10 * 1000000) % 1000000000;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &nextTime,
                    NULL);  // start run
    clock_gettime(CLOCK_MONOTONIC, &realTime);
    loopTimeout = ((double)realTime.tv_sec * 1000.0 +
                   (double)realTime.tv_nsec * 0.001 * 0.001) -
                  ((double)lastrealTime.tv_sec * 1000.0 +
                   (double)lastrealTime.tv_nsec * 0.001 * 0.001);
    if (loopTimeout > 10.5) ROS_WARN("\ntimer10 timeout %fms\n", loopTimeout);
    lastrealTime = realTime;

    //== StateEnum::walking
    //下发舵机数据
    if (bodyhubState != StateEnum::walking) {
      if ((motoQueue.size() > 0) || (headCtrlQueue.size() > 0)) {
        bodyhub::JointControlPoint jntCtrMsg;
        bodyhub::JointControlPoint headCtrMsg;
        bodyhub::ServoPositionAngle servoPositionsMsg;
        int32_t goalPosition[SERVO_NUM];
        uint8_t idArray[SERVO_NUM] = {0};
        uint8_t idCnt = 0;

        if (motoQueue.size() > 0) {
          pthread_mutex_lock(&mtxMo);
          jntCtrMsg = motoQueue.front();
          for (uint8_t idNum = 0;
               idNum < SIZE_LIMIT(jntCtrMsg.positions.size(), SERVO_NUM);
               idNum++) {
            ServoAngleStore[idNum] = jntCtrMsg.positions.at(idNum);
            idArray[idCnt] = idNum + 1;

            goalPosition[idCnt] =
                convertAngleTOValue(idArray[idCnt],
                                    jntCtrMsg.positions.at(idNum)) +
                ServoOffsetValue[idArray[idCnt] - 1];
            ServoRawValueStore[idNum] = goalPosition[idCnt];
            idCnt++;
          }
          motoQueue.pop();
          pthread_mutex_unlock(&mtxMo);
        }
        if (headCtrlQueue.size() > 0) {
          pthread_mutex_lock(&mtxHe);
          if (idCnt > JOINT_SERVO_NUM) idCnt = JOINT_SERVO_NUM;
          headCtrMsg = headCtrlQueue.front();
          for (uint8_t idNum = 0;
               idNum < SIZE_LIMIT(headCtrMsg.positions.size(), HEAD_SERVO_NUM);
               idNum++) {
            ServoAngleStore[idNum + JOINT_SERVO_NUM] =
                headCtrMsg.positions.at(idNum);
            idArray[idCnt] = idCnt + JOINT_SERVO_NUM + 1;  // ID21 ID22

            goalPosition[idCnt] =
                convertAngleTOValue(idArray[idCnt],
                                    headCtrMsg.positions.at(idNum)) +
                ServoOffsetValue[idArray[idCnt] - 1];
            ServoRawValueStore[idNum + JOINT_SERVO_NUM] = goalPosition[idCnt];
            idCnt++;
          }  // ID19 ID20
          headCtrlQueue.pop();
          pthread_mutex_unlock(&mtxHe);
        }

        // #ifdef DEBUG
        //   ROS_INFO("舵机下发：");
        //   vectorOut(ServoAngleStore);
        //   vectorOut(ServoRawValueStore);
        // #endif

        result = dxlTls.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, idArray,
                                  idCnt, goalPosition, 1, &log);  //同步写指令
        if (result == false) {
          ROS_ERROR("SigHandler()---%s", log);
        }
        ServoRadianStore.clear();
        servoPositionsMsg.angle = ServoAngleStore;
        ServoPositionPub.publish(servoPositionsMsg);
      }
    } else {
      control_thread_robot();
    }
  }
}

bool ServoBulkRead(uint8_t *bulkReadID, uint8_t readCount, std::string itemName,
                   int32_t *bulkReadData) {
  /* Dynamixels bulkread */
  const char *log = NULL;
  bool result = false;

  dxlTls.clearBulkReadParam();
  for (uint8_t idNum = 0; idNum < readCount; idNum++) {
    result = dxlTls.addBulkReadParam(bulkReadID[idNum], itemName.c_str(), &log);
    if (result == false) ROS_ERROR("ServoBulkRead# %s", log);
  }
  result = dxlTls.bulkRead(&log);
  if (result == false) {
    ROS_ERROR("ServoBulkRead$ %s\n", log);
    return false;
  }
  result = dxlTls.getBulkReadData(&bulkReadData[0], &log);
  if (result == false) {
    ROS_ERROR("ServoBulkRead# %s\n", log);
    return false;
  }
  return true;
}

bool RawBulkRead(uint8_t *bulkReadID, uint8_t readCount,
                 uint16_t *bulkReadAddress, uint16_t *bulkReadLength,
                 int32_t *bulkReadData) {
  const char *log = NULL;
  bool result = false;

  dxlTls.clearBulkReadParam();
  for (uint8_t idNum = 0; idNum < readCount; idNum++) {
    result = dxlTls.addBulkReadParam(bulkReadID[idNum], bulkReadAddress[idNum],
                                     bulkReadLength[idNum], &log);
    if (result == false) ROS_ERROR("RawBulkRead#1 %s", log);
  }
  result = dxlTls.bulkRead(&log);
  if (result == false) {
    ROS_ERROR("RawBulkRead#2 %s\n", log);
    return false;
  }
  result =
      dxlTls.getRawBulkReadData(&bulkReadID[0], readCount, &bulkReadAddress[0],
                                &bulkReadLength[0], &bulkReadData[0], &log);
  if (result == false) {
    ROS_ERROR("RawBulkRead#3 %s\n", log);
    return false;
  }

  return true;
}

bool SensorBulkWrite(uint8_t WriteCount, uint8_t *bulkWriteID,
                     uint16_t *bulkWriteAddress, uint16_t *bulkWriteLenght,
                     int32_t *bulkWriteData) {
  const char *log = NULL;
  bool result = false;

  for (uint8_t index = 0; index < WriteCount; index++) {
    result = dxlTls.addBulkWriteParam(
        bulkWriteID[index], bulkWriteAddress[index], bulkWriteLenght[index],
        bulkWriteData[index], &log);
    if (result == false) {
      ROS_ERROR("addBulkWriteParam() %s", log);
      return false;
    }
  }
  result = dxlTls.bulkWrite(&log);
  if (result == false) {
    ROS_ERROR("bulkWrite() %s\n", log);
    return false;
  }
  return true;
}

// bool ServoMovingGet() {
//   /* at least 1 servo is moving return true ,else return false */
//   int32_t movingGet[20] = {0};

//   ServoBulkRead(DxlIdList, DxlIdCount, "Moving", &movingGet[0]);
//   for (uint8_t idNum = 0; idNum < DxlIdCount; idNum++) {
//     if (movingGet[idNum]) {
// #ifdef DEBUG
//       ROS_INFO("Moving !!");
// #endif
//       return true;
//     }
//   }
//   return false;
// }

std::vector<double> linspace(double start, double end, int num) {
  std::vector<double> linspaced;

  if (num == 0) {
    return linspaced;
  }
  if (num == 1) {
    linspaced.push_back(start);
    return linspaced;
  }

  double delta = (end - start) / (num - 1);

  for (int i = 0; i < num - 1; ++i) {
    linspaced.push_back(start + delta * i);
  }
  linspaced.push_back(end);

  return linspaced;
}

void RobotStand(uint8_t *idGet, uint8_t idCnt, int32_t velocity) {
  bodyhub::ServoPositionAngle servoPositionsMsg;

  if (!SimControll::simEnable) {
    const char *log = NULL;
    uint8_t idArray[30] = {0};
    int32_t standPositionGoal[30];
    uint8_t intervalCount = 1500 / 20 + 1;
    std::vector<std::vector<double>> motoMoveSequence(30);

    for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
      idArray[idNum] = idGet[idNum];
      standPositionGoal[idNum] =
          convertAngleTOValue(idArray[idNum],
                              StandPositionAngle[idArray[idNum] - 1]) +
          ServoOffsetValue[idArray[idNum] - 1];
      motoMoveSequence[idNum] = linspace(
          ServoRawValueStore[idArray[idNum] - 1], standPositionGoal[idNum], intervalCount);
      ServoAngleStore[idArray[idNum] - 1] = StandPositionAngle[idArray[idNum] - 1];
      ServoRawValueStore[idArray[idNum] - 1] = standPositionGoal[idNum];
    }

    for (uint8_t standCycleT = 0; standCycleT < intervalCount; standCycleT++) {
      for (uint8_t idNum = 0; idNum < idCnt; idNum++)
        standPositionGoal[idNum] = motoMoveSequence[idNum][standCycleT];

      dxlTls.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, idArray, idCnt,
                       standPositionGoal, 1, &log);  //同步写指令
                                                     // TODO: fail return
      usleep(22000);
    }
  } else  // simulation joint command update
  {
    std::vector<double> ServoRadianStore;

    for (uint8_t i = 0; i < SERVO_NUM; i++) {
      ServoRadianStore.push_back(StandPositionAngle[i] * Util::TO_RADIAN);
      ServoAngleStore[i] = StandPositionAngle[i];
    }
    SimControll::updateJointCmdQueue(ServoRadianStore);  // radian
    SimControll::updateJointCmdQueue(ServoRadianStore);  // radian ?
  }
  servoPositionsMsg.angle = ServoAngleStore;
  ServoPositionPub.publish(servoPositionsMsg);
}

void LoadOffset(const std::string path) {
  std::map<std::string, double> offsetMap;
  std::string stringID = "ID";
  std::string IDNameStr;

  YAML::Node offsetDoc;
  try {
    offsetDoc = YAML::LoadFile(path.c_str());
  } catch (const std::exception &e) {
    ROS_WARN("Fail to load offset yaml.");
    return;
  }

  YAML::Node itemData = offsetDoc["offset"];
  if (itemData.size() == 0) return;

  for (YAML::const_iterator itItemNum = itemData.begin();
       itItemNum != itemData.end(); itItemNum++) {
    std::string IDName = itItemNum->first.as<std::string>();
    double offsetValue = itItemNum->second.as<double>();

    offsetMap[IDName] = offsetValue;
  }
  for (uint8_t idNum = 0; idNum < SERVO_NUM; idNum++) {
    IDNameStr = stringID + std::to_string(idNum + 1);
    if (offsetMap.find(IDNameStr) == offsetMap.end())
      ROS_WARN("Without find offset of %s ", IDNameStr.c_str());
    else
      ServoOffsetValue[idNum] = offsetMap[IDNameStr];
    printf("ServoOffsetValue: %f \n", ServoOffsetValue[idNum - 1]);
  }
}

void LoadDxlInitPose(const std::string path) {
  std::map<std::string, double> initPoseMap;
  std::string stringID = "ID";
  std::string IDNameStr;

  YAML::Node initPoseDoc;
  try {
    initPoseDoc = YAML::LoadFile(path.c_str());
  } catch (const std::exception &e) {
    ROS_WARN("Fail to load dxlinitPose yaml.");
    return;
  }

  YAML::Node itemData = initPoseDoc["InitPose"];
  if (itemData.size() == 0) return;

  for (YAML::const_iterator itItemNum = itemData.begin();
       itItemNum != itemData.end(); itItemNum++) {
    std::string IDName = itItemNum->first.as<std::string>();
    double initPoseValue = itItemNum->second.as<double>();

    initPoseMap[IDName] = initPoseValue;
  }
  for (uint8_t idNum = 0; idNum < SERVO_NUM; idNum++) {
    IDNameStr = stringID + std::to_string(idNum + 1);
    if (initPoseMap.find(IDNameStr) == initPoseMap.end())
      ROS_WARN("Without find dxlinitPose of %s ", IDNameStr.c_str());
    else
      StandPositionAngle[idNum] = initPoseMap[IDNameStr];
    printf("StandPositionAngle:%d %f \n", idNum, StandPositionAngle[idNum - 1]);
  }
}

void MotoPositionCallback(const bodyhub::JointControlPoint::ConstPtr &msg) {
  if (masterIDControl == msg->mainControlID) 
  {
    bodyhub::JointControlPoint jointControlMsg;
    jointControlMsg.mainControlID = msg->mainControlID;
    jointControlMsg.positions = msg->positions;
    for(uint8_t i=0;i<msg->positions.size();i++)
    {
      if(i < msg->velocities.size())
        jointControlMsg.velocities.push_back(msg->velocities.at(i));
      else
        jointControlMsg.velocities.push_back(0.0f);

      if(i < msg->accelerations.size())
        jointControlMsg.accelerations.push_back(msg->accelerations.at(i));
      else
        jointControlMsg.accelerations.push_back(0.0f);
    }

    pthread_mutex_lock(&mtxMo);
    motoQueue.push(jointControlMsg);
    pthread_mutex_unlock(&mtxMo);

    //数据到达
    if ((bodyhubState == StateEnum::ready) ||
        (bodyhubState == StateEnum::pause))
      UpdateState(StateEnum::running);
  }
}

void HeadPositionCallback(const bodyhub::JointControlPoint::ConstPtr &msg) {
  /* 仅头部舵机数据接收 0_ID19 1_ID20 */
  if (masterIDControl == msg->mainControlID) 
  {
    bodyhub::JointControlPoint jointControlMsg;
    jointControlMsg.mainControlID = msg->mainControlID;
    jointControlMsg.positions = msg->positions;
    for(uint8_t i=0;i<msg->positions.size();i++)
    {
      if(i < msg->velocities.size())
        jointControlMsg.velocities.push_back(msg->velocities.at(i));
      else
        jointControlMsg.velocities.push_back(0.0f);

      if(i < msg->accelerations.size())
        jointControlMsg.accelerations.push_back(msg->accelerations.at(i));
      else
        jointControlMsg.accelerations.push_back(0.0f);
    }

    pthread_mutex_lock(&mtxMo);
    headCtrlQueue.push(jointControlMsg);
    pthread_mutex_unlock(&mtxMo);

    //数据到达
    if ((bodyhubState == StateEnum::ready) ||
        (bodyhubState == StateEnum::pause))
      UpdateState(StateEnum::running);
  }
}

bool InstReadValSrvCallback(bodyhub::SrvInstRead::Request &req,
                            bodyhub::SrvInstRead::Response &res) {
  bool result = false;
  const char *log = NULL;
  std::string itemName;
  uint8_t dxlID = 0;
  int32_t readGetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    dxlID = req.dxlID;
    itemName = req.itemName;
    result = dxlTls.readRegister(dxlID, itemName.c_str(), &readGetData, &log);
    if (result == false) {
      ROS_WARN("%s\n", log);
      res.getData = SERVO_DEFAULT_VALUE;
    } else
      res.getData = readGetData;
  } else {
    res.getData = SERVO_DEFAULT_VALUE;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}
bool InstWriteValSrvCallback(bodyhub::SrvInstWrite::Request &req,
                             bodyhub::SrvInstWrite::Response &res) {
  const char *log = NULL;
  std::string itemName;
  uint8_t dxlID = 0;
  int32_t writeSetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    dxlID = req.dxlID;
    itemName = req.itemName;
    writeSetData = req.setData;
    dxlTls.writeRegister(dxlID, itemName.c_str(), writeSetData, &log);
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}
bool SyncWriteValSrvCallback(bodyhub::SrvSyncWrite::Request &req,
                             bodyhub::SrvSyncWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t idArray[30] = {0};
  int32_t writeSetData[30];
  uint8_t idCnt = 0;
  uint8_t handleIndex = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    idCnt = req.idCnt;

    if (req.itemName == "Goal_Position") {
      handleIndex = SYNC_WRITE_HANDLER_FOR_GOAL_POSITION;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
        writeSetData[idNum] = req.setData[idNum];
      }
    } else if (req.itemName == "Moving_Speed") {
      handleIndex = SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
        writeSetData[idNum] = req.setData[idNum];
      }
    } else {
      res.complete = result;
      return true;
    }

    result = dxlTls.syncWrite(handleIndex, idArray, idCnt, writeSetData, 1,
                              &log);  //同步写指令
  } else
    ROS_WARN("YOU ARE NOT IN directOperate");

  res.complete = result;
  return true;
}

bool SetServoTarPositionValCallback(bodyhub::SrvServoWrite::Request &req,
                                    bodyhub::SrvServoWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t dxlID = 0;
  int32_t writeSetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    if (SimControll::simEnable) {
      std_msgs::Float64MultiArray jointPosition =
          SimControll::SimRobotData.getJointPostion();
      jointPosition.data[req.dxlID] =
          Angle2Radian(convertValueTOAngle(req.dxlID, req.setData));
      SimControll::updateJointCmdQueue(jointPosition.data);
    } else {
      dxlID = req.dxlID;
      writeSetData = req.setData;
      dxlTls.writeRegister(dxlID, "Goal_Position", writeSetData, &log);
    }
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}

bool SetServoTarPositionValAllCallback(
    bodyhub::SrvServoAllWrite::Request &req,
    bodyhub::SrvServoAllWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t idArray[30] = {0};
  int32_t writeSetData[30];
  uint8_t idCnt = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    if (SimControll::simEnable) {
      std_msgs::Float64MultiArray jointPosition =
          SimControll::SimRobotData.getJointPostion();
      for (uint8_t i = 0; i < req.idArray.size(); i++)
        jointPosition.data[req.idArray[i] - 1] =
            Angle2Radian(convertValueTOAngle(req.idArray[i], req.setData[i]));
      SimControll::updateJointCmdQueue(jointPosition.data);
    } else {
      idCnt = req.idCnt;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
        writeSetData[idNum] = req.setData[idNum];
      }
      result = dxlTls.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, idArray,
                                idCnt, writeSetData, 1, &log);  //同步写指令
    }
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}

bool GetServoPositionValAllCallback(bodyhub::SrvServoAllRead::Request &req,
                                    bodyhub::SrvServoAllRead::Response &res) {
  int32_t defaultValue = 2048;
  uint8_t idArray[30] = {0};
  uint8_t idCnt = 0;
  int32_t readGetData[30] = {
      defaultValue, defaultValue, defaultValue, defaultValue, defaultValue,
      defaultValue, defaultValue, defaultValue, defaultValue, defaultValue,
      defaultValue, defaultValue, defaultValue, defaultValue, defaultValue,
      defaultValue, defaultValue, defaultValue, defaultValue, defaultValue,
      defaultValue, defaultValue};

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    if (SimControll::simEnable) {
      std_msgs::Float64MultiArray jointPosition =
          SimControll::SimRobotData.getJointPostion();
      for (uint8_t i = 0; i < req.idArray.size(); i++)
        res.getData.push_back(convertAngleTOValue(
            req.idArray[i],
            Radian2Angle(jointPosition.data[req.idArray[i] - 1])));
    } else {
      idCnt = req.idCnt;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
      }
      ServoBulkRead(idArray, idCnt, "Present_Position", &readGetData[0]);
      for (uint8_t idNum = 0; idNum < idCnt; idNum++)
        res.getData.push_back(readGetData[idNum]);
    }
  } else {
    res.getData.push_back(0);
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}

bool InstReadSrvCallback(bodyhub::SrvInstRead::Request &req,
                         bodyhub::SrvInstRead::Response &res) {
  bool result = false;
  const char *log = NULL;
  std::string itemName;
  uint8_t dxlID = 0;
  int32_t readGetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    dxlID = req.dxlID;
    itemName = req.itemName;
    result = dxlTls.readRegister(dxlID, itemName.c_str(), &readGetData, &log);
    if (result == false) {
      ROS_WARN("%s\n", log);
      res.getData = 998;
    } else if (itemName == "Present_Position")
      res.getData = convertValueTOAngle(dxlID, readGetData);  // value_to_angle
    else
      res.getData = readGetData;
  } else {
    res.getData = 999;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}

bool InstWriteSrvCallback(bodyhub::SrvInstWrite::Request &req,
                          bodyhub::SrvInstWrite::Response &res) {
  const char *log = NULL;
  std::string itemName;
  uint8_t dxlID = 0;
  int32_t writeSetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    dxlID = req.dxlID;
    itemName = req.itemName;
    if (itemName == "Goal_Position")
      writeSetData = convertAngleTOValue(dxlID, req.setData);
    else
      writeSetData = req.setData;
    dxlTls.writeRegister(dxlID, itemName.c_str(), writeSetData, &log);
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}

bool SyncWriteSrvCallback(bodyhub::SrvSyncWrite::Request &req,
                          bodyhub::SrvSyncWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t idArray[30] = {0};
  int32_t writeSetData[30];
  uint8_t idCnt = 0;
  uint8_t handleIndex = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    idCnt = req.idCnt;

    if (req.itemName == "Goal_Position") {
      handleIndex = SYNC_WRITE_HANDLER_FOR_GOAL_POSITION;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
        writeSetData[idNum] =
            convertAngleTOValue(idArray[idNum], req.setData[idNum]);
      }
    } else if (req.itemName == "Moving_Speed") {
      handleIndex = SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
        writeSetData[idNum] = req.setData[idNum];
      }
    } else {
      res.complete = result;
      return true;
    }

    result = dxlTls.syncWrite(handleIndex, idArray, idCnt, writeSetData, 1,
                              &log);  //同步写指令
  } else
    ROS_WARN("YOU ARE NOT IN directOperate");

  res.complete = result;
  return true;
}

bool SetServoLockStateCallback(bodyhub::SrvServoWrite::Request &req,
                               bodyhub::SrvServoWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t dxlID = 0;
  int32_t writeSetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    dxlID = req.dxlID;
    writeSetData = (int32_t)req.setData;
    if (writeSetData == 0)
      dxlTls.torqueOff(dxlID, &log);
    else
      dxlTls.torqueOn(dxlID, &log);
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}
bool SetServoLockStateAllCallback(bodyhub::SrvServoAllWrite::Request &req,
                                  bodyhub::SrvServoAllWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t idArray[30] = {0};
  int32_t writeSetData[30];
  uint8_t idCnt = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    idCnt = req.idCnt;
    for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
      idArray[idNum] = req.idArray[idNum];       // test
      writeSetData[idNum] = req.setData[idNum];  // test
      if (writeSetData[idNum] == 0)
        dxlTls.writeRegister(idArray[idNum], "Torque_Enable", 0, &log);
      else if (writeSetData[idNum] == 1)
        dxlTls.writeRegister(idArray[idNum], "Torque_Enable", 1, &log);
    }
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}
bool GetServoLockStateAllCallback(bodyhub::SrvServoAllRead::Request &req,
                                  bodyhub::SrvServoAllRead::Response &res) {
  uint8_t idArray[30] = {0};
  uint8_t idCnt = 0;
  int32_t readGetData[30] = {0};

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    idCnt = req.idCnt;
    for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
      idArray[idNum] = req.idArray[idNum];
    }
    ServoBulkRead(idArray, idCnt, "Torque_Enable", &readGetData[0]);
    for (uint8_t idNum = 0; idNum < idCnt; idNum++)
      res.getData.push_back(readGetData[idNum]);
  } else {
    res.getData.push_back(0);
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}
bool SetServoTarPositionCallback(bodyhub::SrvServoWrite::Request &req,
                                 bodyhub::SrvServoWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t dxlID = 0;
  int32_t writeSetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    if (SimControll::simEnable) {
      std_msgs::Float64MultiArray jointPosition =
          SimControll::SimRobotData.getJointPostion();
      jointPosition.data[req.dxlID] = Angle2Radian(req.setData);
      SimControll::updateJointCmdQueue(jointPosition.data);
    } else {
      dxlID = req.dxlID;
      writeSetData = convertAngleTOValue(dxlID, req.setData);
      dxlTls.writeRegister(dxlID, "Goal_Position", writeSetData, &log);
    }
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}
bool SetServoTarPositionAllCallback(bodyhub::SrvServoAllWrite::Request &req,
                                    bodyhub::SrvServoAllWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t idArray[30] = {0};
  int32_t writeSetData[30];
  uint8_t idCnt = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    if (SimControll::simEnable) {
      std_msgs::Float64MultiArray jointPosition =
          SimControll::SimRobotData.getJointPostion();
      for (uint8_t i = 0; i < req.idArray.size(); i++)
        jointPosition.data[req.idArray[i] - 1] = Angle2Radian(req.setData[i]);
      SimControll::updateJointCmdQueue(jointPosition.data);
    } else {
      idCnt = req.idCnt;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
        writeSetData[idNum] =
            convertAngleTOValue(req.idArray[idNum], req.setData[idNum]);
      }
      result = dxlTls.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, idArray,
                                idCnt, writeSetData, 1, &log);  //同步写指令
    }
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}
bool GetServoPositionAllCallback(bodyhub::SrvServoAllRead::Request &req,
                                 bodyhub::SrvServoAllRead::Response &res) {
  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    if (SimControll::simEnable) {
      std_msgs::Float64MultiArray jointPosition =
          SimControll::SimRobotData.getJointPostion();
      for (uint8_t i = 0; i < req.idArray.size(); i++)
        res.getData.push_back(
            Radian2Angle(jointPosition.data[req.idArray[i] - 1]));
    } else {
      uint8_t idArray[30] = {0};
      uint8_t idCnt = 0;
      int32_t readGetData[30] = {0};

      idCnt = req.idCnt;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
      }
      ServoBulkRead(idArray, idCnt, "Present_Position", &readGetData[0]);
      for (uint8_t idNum = 0; idNum < idCnt; idNum++)
        res.getData.push_back(convertValueTOAngle(
            req.idArray[idNum], readGetData[idNum]));  // value_to_angle
    }
  } else {
    res.getData.push_back(0);
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}

bool StateSrvCallback(bodyhub::SrvState::Request &req,
                      bodyhub::SrvState::Response &res) {
  if ((req.masterID == masterIDControl) || (masterIDControl == 0)) {
    if (req.stateReq == "setStatus") {
      masterIDControl = req.masterID;
      if (bodyhubState == StateEnum::preReady) UpdateState(StateEnum::ready);
    } else if (req.stateReq == "resetStatus") {
    } else if (req.stateReq == "break") {
    } else if (req.stateReq == "stop") {
      if ((bodyhubState == StateEnum::running) ||
          (bodyhubState == StateEnum::pause) ||
          (bodyhubState == StateEnum::walking)) {
        pthread_mutex_lock(&mtxWl);
        ClearTimerQueue();
        UpdateState(StateEnum::stoping);
        pthread_mutex_unlock(&mtxWl);
      }
    } else if (req.stateReq == "reset") {
      if (bodyhubState != StateEnum::stoping) {
        masterIDControl = 0;
        UpdateState(StateEnum::preReady);
      }
    } else if (req.stateReq == "walking") {
      if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::walking);
    }
    res.stateRes = bodyhubState;
  } else
    res.stateRes = masterIDControl;
  return true;
}

bool GetStatusCallback(bodyhub::SrvString::Request &req,
                       bodyhub::SrvString::Response &res) {
  if (req.str != "")
  {
    res.data = stateNewStr;
    res.poseQueueSize = motoQueue.size();
  }
  return true;
}

bool GetJointAngleCallback(bodyhub::SrvServoAllRead::Request &req,
                           bodyhub::SrvServoAllRead::Response &res) {
  res.getData = ServoAngleStore;
  return true;
}

bool MasterIDSrvCallback(bodyhub::SrvTLSstring::Request &req,
                         bodyhub::SrvTLSstringResponse &res) {
#ifdef DEBUG
// ROS_INFO("MasterIDSrvCallback get masterID %s", req.str.c_str());
#endif
  res.data = masterIDControl;
  return true;
}

void QueueThread() {
  ros::NodeHandle n;
  ros::CallbackQueue topicQueue;

  n.setCallbackQueue(&topicQueue);

  ros::Subscriber MotoPositionSub = n.subscribe(
      "MediumSize/BodyHub/MotoPosition", 1000, MotoPositionCallback);
  ros::Subscriber HeadPositionSub = n.subscribe(
      "MediumSize/BodyHub/HeadPosition", 1000, HeadPositionCallback);

  while (n.ok()) {
    topicQueue.callAvailable(ros::WallDuration(0.01));
  }
}

bool initSDKHandlers(void) {
  bool result = false;
  const char *log = NULL;

  result = dxlTls.addSyncWriteHandler(DxlIdList[0], "Goal_Position", &log);
  if (result == false) {
    ROS_ERROR("%s", log);
    return result;
  }

  result = dxlTls.addSyncWriteHandler(DxlIdList[0], "Moving_Speed",
                                      &log);  //第一个舵机号要为存在舵机
  if (result == false) {
    ROS_ERROR("%s", log);
    return result;
  }

  result = dxlTls.initBulkRead(&log);
  if (result == false) {
    ROS_ERROR("%s", log);
    return result;
  }

  result = dxlTls.initBulkWrite(&log);
  if (result == false) {
    ROS_ERROR("%s", log);
    return result;
  }

  return result;
}

void UpdateState(uint8_t stateNew) {
  int32_t readGetData = 0;
  bodyhub::ServoPositionAngle servoPositionsMsg;
  const char *log = NULL;
  bool result = false;

  if ((stateNew == StateEnum::preReady) || (stateNew == StateEnum::stoping)) {
    if (!SimControll::simEnable) {
      for (uint8_t idNum = 0; idNum < SERVO_NUM; idNum++) {
        result = dxlTls.readRegister(idNum + 1, "Present_Position",
                                     &readGetData, &log);
        if (result == false) {
          ROS_WARN("%sUpdateState():missingID%d %s\n", stateNewStr.c_str(),
                   idNum + 1, log);
          ServoRawValueStore[idNum] = 2048;  // dataerror
          ServoAngleStore[idNum] = 0;        // dataerror
        } else {
          ServoAngleStore[idNum] = convertValueTOAngle(idNum, readGetData);
          ServoRawValueStore[idNum] = readGetData;
        }
      }
      servoPositionsMsg.angle = ServoAngleStore;
      ServoPositionPub.publish(servoPositionsMsg);
      if (bodyhubState == StateEnum::directOperate) {
        // reload offset
        if (offsetFile != "") LoadOffset(offsetFile);
#ifdef DEBUG
        for (uint8_t idNum = 0; idNum < SERVO_NUM; idNum++)
          ROS_INFO("directOperate exit Servo Present_Position ID%d: %f", idNum,
                   ServoRawValueStore[idNum]);
#endif
      }
      result = dxlTls.itemWrite(254, "P_gain", 6, &log);  // init
    } else {
      std_msgs::Float64MultiArray jointPosition =
          SimControll::SimRobotData.getJointPostion();
      for (uint8_t i = 0; i < jointPosition.data.size(); i++)
        servoPositionsMsg.angle.push_back(Radian2Angle(jointPosition.data[i]));
      ServoPositionPub.publish(servoPositionsMsg);
    }
    RobotStand(DxlIdList, DxlIdCount, 1000);
    ClearTimerQueue();
  } else if (stateNew == StateEnum::walking) {
    // RobotSquat
    std::vector<Eigen::VectorXd> squatSequence;
    mWalk.initSquat(squatSequence);
    for (uint8_t cycleCount = 0; cycleCount < squatSequence.size();
         cycleCount++) {
      mWalk.squatCount++;
      mWalk.jointValue.segment(0, 12) =
          squatSequence[cycleCount];  // FIXME: (0,22)
      WalkingSendData();
      usleep(22000);  // FIXME:
    }

    if (!SimControll::simEnable) {
      // P P P P P P_gain
      result = dxlTls.itemWrite(1, "P_gain", 15, &log);
      if (result == false) {
        printf("%s\n", log);
        printf("Failed to P_gain \n");
      }
      result = dxlTls.itemWrite(7, "P_gain", 15, &log);
      if (result == false) {
        printf("%s\n", log);
        printf("Failed to P_gain \n");
      }

      result = dxlTls.itemWrite(2, "P_gain", 40, &log);
      if (result == false) {
        printf("%s\n", log);
        printf("Failed to P_gain \n");
      }
      result = dxlTls.itemWrite(8, "P_gain", 40, &log);
      if (result == false) {
        printf("%s\n", log);
        printf("Failed to P_gain \n");
      }

      result = dxlTls.itemWrite(3, "P_gain", 22, &log);
      if (result == false) {
        printf("%s\n", log);
        printf("Failed to P_gain \n");
      }
      result = dxlTls.itemWrite(9, "P_gain", 22, &log);
      if (result == false) {
        printf("%s\n", log);
        printf("Failed to P_gain \n");
      }

      result = dxlTls.itemWrite(4, "P_gain", 35, &log);
      if (result == false) {
        printf("%s\n", log);
        printf("Failed to P_gain \n");
      }
      result = dxlTls.itemWrite(10, "P_gain", 35, &log);
      if (result == false) {
        printf("%s\n", log);
        printf("Failed to P_gain \n");
      }

      result = dxlTls.itemWrite(5, "P_gain", 25, &log);
      if (result == false) {
        printf("%s\n", log);
        printf("Failed to P_gain \n");
      }
      result = dxlTls.itemWrite(11, "P_gain", 25, &log);
      if (result == false) {
        printf("%s\n", log);
        printf("Failed to P_gain \n");
      }

      result = dxlTls.itemWrite(6, "P_gain", 40, &log);
      if (result == false) {
        printf("%s\n", log);
        printf("Failed to P_gain \n");
      }
      result = dxlTls.itemWrite(12, "P_gain", 40, &log);
      if (result == false) {
        printf("%s\n", log);
        printf("Failed to P_gain \n");
      }

      // hand
      result = dxlTls.itemWrite(13, "P_gain", 3, &log);
      if (result == false) {
        printf("%s\n", log);
        printf("Failed to P_gain \n");
      }
      result = dxlTls.itemWrite(16, "P_gain", 3, &log);
      if (result == false) {
        printf("%s\n", log);
        printf("Failed to P_gain \n");
      }
    }
  } else if (stateNew == StateEnum::ready) {
    if (SimControll::simEnable && SimControll::simState != 1)
      SimControll::simStart();
  } else if (stateNew == StateEnum::error) {
    if (SimControll::simEnable) SimControll::simPause();
  }

  bodyhubState = stateNew;
  budyhubStateMsg.data = stateNew;
  StatusPub.publish(budyhubStateMsg);

  switch (stateNew) {
    case StateEnum::init:
      stateNewStr = "init";
      break;
    case StateEnum::preReady:
      stateNewStr = "preReady";
      break;
    case StateEnum::ready:
      stateNewStr = "ready";
      break;
    case StateEnum::running:
      stateNewStr = "running";
      break;
    case StateEnum::pause:
      stateNewStr = "pause";
      break;
    case StateEnum::stoping:
      stateNewStr = "stoping";
      break;
    case StateEnum::error:
      stateNewStr = "error";
      break;
    case StateEnum::directOperate:
      stateNewStr = "directOperate";
      break;
    case StateEnum::walking:
      stateNewStr = "walking";
      break;
    default:
      break;
  }
  ROS_INFO("The new bodyhubState: %s--%d ", stateNewStr.c_str(), stateNew);
}

void ClearTimerQueue() {
  pthread_mutex_lock(&mtxHe);
  pthread_mutex_lock(&mtxMo);
  if ((!motoQueue.empty()) || (!headCtrlQueue.empty())) {
    ClearQueue(motoQueue);
    ClearQueue(headCtrlQueue);
  }
  pthread_mutex_unlock(&mtxHe);
  pthread_mutex_unlock(&mtxMo);
}

#if 0
void ServoPrarmInit(uint8_t *idList, uint8_t idNum) {
  bool result = false;
  const char *log = NULL;
  int32_t param = 40;

  result = dxlTls.addSyncWriteHandler(idList[0], "P_gain", &log);
  if (result == false) {
    ROS_WARN("ServoPrarmInit() --> %s", log);
    return;
  }

  result = dxlTls.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, idList, idNum,
                            &param, 1, &log);
  if (result == false) {
    ROS_WARN("ServoPrarmInit() --> %s", log);
    return;
  }
}
#else
void ServoPrarmInit(uint8_t *idList, uint8_t idNum) {
  bool result = false;
  const char *log = NULL;
  int32_t param = 30;

  for (uint8_t i = 0; i < idNum; i++) {
    result = dxlTls.writeRegister(idList[i], "P_gain", param, &log);
    if (result == false) {
      ROS_WARN("ServoPrarmInit() --> %d, %s", idList[i], log);
    }
  }
}
#endif

bool ProtocolInit() {
  const char *portName = "/dev/ttyUSB0";
  int baudrate = 1000000;

  const char *log = NULL;
  bool result = false;

  result = dxlTls.init(portName, baudrate, &log);  // initWorkbench
  if (result == false) {
    ROS_ERROR("dxlTls.init() log: %s", log);
    return false;
  } else
    ROS_INFO("dxlTls.init() succeed, baudrate: %d", baudrate);
  for(uint8_t scanCount = 0; scanCount < 3; scanCount++){
    dxlTls.scan(DxlIdList, &DxlIdCount, DXL_ID_SCAN_RANGE, &log);
    if(DxlIdCount == 22)
      break;
  }
  ROS_INFO("Number of Dynamixel devices: %d", DxlIdCount);

  if (DxlIdCount > 0) {
    for (uint8_t idNum = 0; idNum < DxlIdCount; idNum++) {
      ROS_INFO("DxlIdList[%d]: %d", idNum, DxlIdList[idNum]);
    }
    for (uint8_t idNum = 0; idNum < DxlIdCount; ++idNum)
      dxlTls.torqueOn(DxlIdList[idNum], &log);
    result = initSDKHandlers();
    ServoPrarmInit(DxlIdList, DxlIdCount);
  } else {
    ROS_ERROR("No Dynamixel device found!");
  }
  return true;
}

void ShutDownSignHandler(int sig) {
  if (SimControll::simEnable) SimControll::simStop();
  ros::shutdown();
}

void STATEinit(ros::NodeHandle nh) {
  StatusPub = nh.advertise<std_msgs::UInt16>("MediumSize/BodyHub/Status", 0);
  ServoPositionPub = nh.advertise<bodyhub::ServoPositionAngle>(
      "MediumSize/BodyHub/ServoPositions", 0, true);

  StateService =
      nh.advertiseService("MediumSize/BodyHub/StateJump", StateSrvCallback);
  GetStatusService =
      nh.advertiseService("MediumSize/BodyHub/GetStatus", GetStatusCallback);
  MasterIDService = nh.advertiseService("MediumSize/BodyHub/GetMasterID",
                                        MasterIDSrvCallback);
  GetJointAngleService = nh.advertiseService("MediumSize/BodyHub/GetJointAngle",
                                             GetJointAngleCallback);

  // VALUE
  InstReadValService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/InstReadVal", InstReadValSrvCallback);
  InstWriteValService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/InstWriteVal", InstWriteValSrvCallback);
  SyncWriteValService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/SyncWriteVal", SyncWriteValSrvCallback);

  SetTarPositionValService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/SetServoTarPositionVal",
      SetServoTarPositionValCallback);
  SetTarPositionValAllService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/SetServoTarPositionValAll",
      SetServoTarPositionValAllCallback);
  GetPositionValAllService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/GetServoPositionValAll",
      GetServoPositionValAllCallback);

  // ANGLE
  InstReadService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/InstRead", InstReadSrvCallback);
  InstWriteService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/InstWrite", InstWriteSrvCallback);
  SyncWriteService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/SyncWrite", SyncWriteSrvCallback);

  SetLockStateService =
      nh.advertiseService("MediumSize/BodyHub/DirectMethod/SetServoLockState",
                          SetServoLockStateCallback);
  SetLockStateAllService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/SetServoLockStateAll",
      SetServoLockStateAllCallback);
  GetLockStateAllService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/GetServoLockStateAll",
      GetServoLockStateAllCallback);

  SetTarPositionService =
      nh.advertiseService("MediumSize/BodyHub/DirectMethod/SetServoTarPosition",
                          SetServoTarPositionCallback);
  SetTarPositionAllService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/SetServoTarPositionAll",
      SetServoTarPositionAllCallback);
  GetPositionAllService =
      nh.advertiseService("MediumSize/BodyHub/DirectMethod/GetServoPositionAll",
                          GetServoPositionAllCallback);

  /***walking***walking***walking***/
  jointPos.data.resize(12);
  footTraj.data.resize(3);

  jointPosTargetPub =
      nh.advertise<std_msgs::Float64MultiArray>("jointPosTarget", 1000);
  jointPosMeasurePub =
      nh.advertise<std_msgs::Float64MultiArray>("jointPosMeasure", 1000);
  jointVelTargetPub =
      nh.advertise<std_msgs::Float64MultiArray>("jointVelTarget", 1000);
  jointVelMeasurePub =
      nh.advertise<std_msgs::Float64MultiArray>("jointVelMeasure", 1000);
  cpref_pub = nh.advertise<std_msgs::Float64>("cpref", 1000);
  cpC_pub = nh.advertise<std_msgs::Float64>("cpC", 1000);
  copm_pub = nh.advertise<std_msgs::Float64>("copm", 1000);
  copD_pub = nh.advertise<std_msgs::Float64>("copD", 1000);
  copref_pub = nh.advertise<std_msgs::Float64>("copref", 1000);
  comm_pub = nh.advertise<std_msgs::Float64>("comm", 1000);
  comD_pub = nh.advertise<std_msgs::Float64>("comD", 1000);
  comref_pub = nh.advertise<std_msgs::Float64>("comref", 1000);
  comEsti_pub = nh.advertise<std_msgs::Float64>("comEsti", 1000);
  comvm_pub = nh.advertise<std_msgs::Float64>("comvm", 1000);
  comvD_pub = nh.advertise<std_msgs::Float64>("comvD", 1000);
  comvref_pub = nh.advertise<std_msgs::Float64>("comvref", 1000);
  comvEsti_pub = nh.advertise<std_msgs::Float64>("comvEsti", 1000);
  LFootZ = nh.advertise<std_msgs::Float64>("LFootZ", 1000);
  RFootZ = nh.advertise<std_msgs::Float64>("RFootZ", 1000);
  footDisRef = nh.advertise<std_msgs::Float64>("footDisRef", 1000);
  footDis = nh.advertise<std_msgs::Float64>("footDis", 1000);
  contactState_pub = nh.advertise<std_msgs::Float64>("contactState", 1000);
  stepPhase_pub = nh.advertise<std_msgs::Float64>("stepPhase", 1000);
  // JY901X = nh.advertise<std_msgs::Float64>("JY901X",1000);
  // JY901Y = nh.advertise<std_msgs::Float64>("JY901Y",1000);

  WalkingStatusPub = nh.advertise<std_msgs::Float64>(
      "/MediumSize/BodyHub/WalkingStatus", 1000);
  /***walking***walking***walking***/

  UpdateState(StateEnum::init);

  // load offset (value)
  if (offsetFile != "") LoadOffset(offsetFile);
  // load dxlinitpose (angle)
  if (InitPoseFile != "") LoadDxlInitPose(InitPoseFile);

  pthread_mutex_init(&mtxWl, NULL);
  pthread_mutex_init(&mtxMo, NULL);
  pthread_mutex_init(&mtxHe, NULL);
  pthread_mutex_init(&mtxSL, NULL);
  pthread_mutex_init(&MutexBulkRead, NULL);

  if (SimControll::simEnable)  // Vrep Simulation
  {
    SimControll::simInit(nh);
    signal(SIGINT, ShutDownSignHandler);  //关闭节点时停止仿真
  } else {
    if (ProtocolInit() == false) {
#if 1
      ROS_ERROR("ProtocolInit() error!");
      exit(0);
#else
      SimControll::simEnable = true;
      SimControll::simInit(nh);
      signal(SIGINT, ShutDownSignHandler);  //关闭节点时停止仿真
#endif
    }
  }

  //更新下一个状态
  UpdateState(StateEnum::preReady);
}

void STATEpreReady() { const char *log = NULL; }

void STATEready() { const char *log = NULL; }
void STATErunning() {
  // check empty & Moving
  if ((motoQueue.empty()) && (headCtrlQueue.empty()))
    UpdateState(StateEnum::pause);  //更新下一个状态
}
void STATEpause() { const char *log = NULL; }
void STATEstoping() {
  //更新下一个状态
  UpdateState(StateEnum::ready);
}
void STATEerror() { const char *log = NULL; }

void STATEdirectOperate() { const char *log = NULL; }

void STATEwalking() { const char *log = NULL; }

int main(int argc, char **argv) {
  //初始化节点
  ros::init(argc, argv, "BodyHubNode");
  ros::NodeHandle nodeHandle;

  /* Load ROS Parameter */
  nodeHandle.getParam("simenable", SimControll::simEnable);
  nodeHandle.param<std::string>("poseOffsetPath", offsetFile, "");
  nodeHandle.param<std::string>("poseInitPath", InitPoseFile, "");
  nodeHandle.param<std::string>("sensorNameIDPath", sensorNameIDFile, "");

  STATEinit(nodeHandle);
  cpWalk = GaitManager::CPWalking5::GetInstance();
  cpWalk->commandRosInit();

  boost::thread communicateThread(QueueThread);
  boost::thread timerThread(threadTimer);
  // 创建外接传感器相关线程
  std::thread ThreadExtendSensor(ExtendSensorThread);
  std::thread ThreadExtendSensorPoll(ExtendSensorTimerThread);

  std::thread ThreadSimulate(SimControll::simulateThread);

  ros::Rate loop_rate(100);
  while (ros::ok()) {
    if (bodyhubState == StateEnum::preReady)
      STATEpreReady();
    else if (bodyhubState == StateEnum::ready)
      STATEready();
    else if (bodyhubState == StateEnum::running)
      STATErunning();
    else if (bodyhubState == StateEnum::pause)
      STATEpause();
    else if (bodyhubState == StateEnum::stoping)
      STATEstoping();
    else if (bodyhubState == StateEnum::error)
      STATEerror();
    else if (bodyhubState == StateEnum::directOperate)
      STATEdirectOperate();
    else if (bodyhubState == StateEnum::walking)
      STATEwalking();

    ros::spinOnce();
    loop_rate.sleep();
  }

  communicateThread.join();
  timerThread.join();
  ThreadExtendSensor.join();
  ThreadExtendSensorPoll.join();
  ThreadSimulate.join();

  return 0;
}

bool WalkingSendData(void) {
  if (!SimControll::simEnable) {
    const char *log = NULL;
    bool result = false;
    double hand_swing = 1.8;
    int32_t goalPosition[22] = {0};
    float value;
    for (uint8_t idNum = 0; idNum < DxlIdCount; idNum++)  // ANGLE_TO_VALUE
    {
      if (idNum < 12)
        goalPosition[idNum] = WalkingAngleDirection[idNum] *
                                  mWalk.jointValue[idNum] * AngleAlpha[idNum] +
                              2048 + ServoOffsetValue[idNum];
      else if (idNum == 12)
        goalPosition[idNum] =
            -hand_swing * mWalk.armSwingCount * AngleAlpha[idNum] + 2048 +
            ServoOffsetValue[idNum];
      else if (idNum == 15)
        goalPosition[idNum] =
            -hand_swing * mWalk.armSwingCount * AngleAlpha[idNum] + 2048 +
            ServoOffsetValue[idNum];
      else if (idNum == 13)
        goalPosition[idNum] = ((StandPositionAngle[13] - ServoAngleStore[13]) *
                                   mWalk.squatCount / mWalk.squatStep +
                               ServoAngleStore[13]) *
                                  AngleAlpha[idNum] +
                              2048 + ServoOffsetValue[idNum];
      else if (idNum == 14)
        goalPosition[idNum] = ((StandPositionAngle[14] - ServoAngleStore[14]) *
                                   mWalk.squatCount / mWalk.squatStep +
                               ServoAngleStore[14]) *
                                  AngleAlpha[idNum] +
                              2048 + ServoOffsetValue[idNum];
      else if (idNum == 16)
        goalPosition[idNum] = ((StandPositionAngle[16] - ServoAngleStore[16]) *
                                   mWalk.squatCount / mWalk.squatStep +
                               ServoAngleStore[16]) *
                                  AngleAlpha[idNum] +
                              2048 + ServoOffsetValue[idNum];
      else if (idNum == 17)
        goalPosition[idNum] = ((StandPositionAngle[17] - ServoAngleStore[17]) *
                                   mWalk.squatCount / mWalk.squatStep +
                               ServoAngleStore[17]) *
                                  AngleAlpha[idNum] +
                              2048 + ServoOffsetValue[idNum];
      else
        goalPosition[idNum] =
            0 * AngleAlpha[idNum] + 2048 + ServoOffsetValue[idNum];
    }
    if (mWalk.RobotState == mWalk.Action_YawAround) {
      goalPosition[12] =
          WalkingAngleDirection[12] * mWalk.LArm_P * AngleAlpha[12] + 2048 +
          ServoOffsetValue[12];
      goalPosition[13] =
          WalkingAngleDirection[13] * -mWalk.LArm_R * AngleAlpha[13] + 2048 +
          ServoOffsetValue[13];
      goalPosition[14] =
          WalkingAngleDirection[14] * -mWalk.LArm_elbow * AngleAlpha[14] +
          2048 + ServoOffsetValue[14];
      goalPosition[15] =
          WalkingAngleDirection[15] * -mWalk.RArm_P * AngleAlpha[15] + 2048 +
          ServoOffsetValue[15];
      goalPosition[16] =
          WalkingAngleDirection[16] * mWalk.RArm_R * AngleAlpha[16] + 2048 +
          ServoOffsetValue[16];
      goalPosition[17] =
          WalkingAngleDirection[17] * mWalk.RArm_elbow * AngleAlpha[17] + 2048 +
          ServoOffsetValue[17];

      // ROS_WARN("13:%f, 14:%f, 15:%f, 16:%f, 17:%f,
      // 18:%f",mWalk.LArm_P,-mWalk.LArm_R,-mWalk.LArm_elbow,-mWalk.RArm_P,mWalk.RArm_R,mWalk.RArm_elbow);
    }

    dxlTls.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, goalPosition, &log);
  } else {
    uint8_t i = 0;
    std::vector<double> jointCmd;
    std_msgs::Float64MultiArray jointPosition =
        SimControll::SimRobotData.getJointPostion();
    for (i = 0; i < 12; i++) {
      jointCmd.push_back(
          Angle2Radian(WalkingAngleDirection[i] * mWalk.jointValue[i]));
    }
    for (; i < 18; i++) 
    {
      jointCmd.push_back(0.0);
      if((i == 12) || (i == 15))
      {
         jointCmd[i] = Angle2Radian(-1.8 * mWalk.armSwingCount)*WalkingAngleDirection[i];
      }
      else
      {
        jointCmd[i] = Angle2Radian(StandPositionAngle[i])*WalkingAngleDirection[i];
      }
    }
    SimControll::updateJointCmdQueue(jointCmd);
  }
}

bool WalkingReceiveData(void) {
  /* BulkRead servo Position  FSR*/

  if (!SimControll::simEnable) {
    int32_t present_position[30];
    int32_t present_current[30];
    int32_t present_velocity[30];  // FIXME:
    int32_t left_fsr[30];
    int32_t right_fsr[30];
    uint8_t *present_left_fsr;
    uint8_t *present_right_fsr;

    uint8_t right_fsr_id = RIGHT_FSR_ID;
    uint8_t left_fsr_id = LEFT_FSR_ID;
    uint16_t addr = FSR_ADDR;
    uint16_t len = FSR_ADDR_LEN;

    const char *log = NULL;
    bool result = false;

    uint8_t readCount = 12;
    uint8_t bulkReadID[readCount];
    uint16_t bulkReadAddress[readCount];
    uint16_t bulkReadLength[readCount];
    uint8_t readDataLength = 0;
    int32_t bulkReadData[readCount * 2];
    int32_t bulkReadWord = 0;

    // BulkRead servo Position
    for (uint8_t idNum = 0; idNum < readCount; idNum++) {
      bulkReadID[idNum] = DxlIdList[idNum];
      bulkReadAddress[idNum] = BULK_READ_PRESENT_POSITION_ADDR;
      bulkReadLength[idNum] = BULK_READ_PRESENT_POSITION_LEN;
      readDataLength += BULK_READ_PRESENT_POSITION_LEN;
    }

    pthread_mutex_lock(&MutexBulkRead);
    RawBulkRead(bulkReadID, readCount, bulkReadAddress, bulkReadLength,
                bulkReadData);
    pthread_mutex_unlock(&MutexBulkRead);
    for (uint8_t idNum = 0; idNum < readCount; idNum++) {
      bulkReadWord =
          DXL_MAKEWORD(bulkReadData[idNum * 2], bulkReadData[idNum * 2 + 1]);
      motoDataValuePre[idNum] = present_position[idNum] = bulkReadWord;
      // std::cout << unsigned(DxlIdList[idNum]) << "-" << bulkReadWord << "
      // ";//行打印 value
    }

    for (uint8_t idNum = 0; idNum < readCount; idNum++)  // VALUE_TO_DEGREE
    {
      present_position[idNum] =
          present_position[idNum] - ServoOffsetValue[idNum];
      measuredJointPos[idNum] =
          (present_position[idNum] - 2048) /
          (AngleAlpha[idNum] * WalkingAngleDirection[idNum]);

      // std::cout << unsigned(DxlIdList[idNum]) << "#" <<
      // measuredJointPos[idNum]
      // << "  ";//行打印 角度

      // if(present_position[id] <= 65535 && present_position[id] >= 10000)
      // 	measuredJointPWM[id] = present_position[id]-65535;
      // else
      // 	measuredJointPWM[id] = present_position[id];

      // if(present_position[id] <= 65535 && present_position[id] >= 10000)
      // 	measuredJointCurrent[id] = present_position[id]-65535;
      // else
      // 	measuredJointCurrent[id] = present_position[id];
    }
    // std::cout << "\r";

    usleep(10);
    // BulkRead footprint
    readCount = 2;
    bulkReadID[0] = LEFT_FSR_ID;
    bulkReadID[1] = RIGHT_FSR_ID;
    bulkReadAddress[0] = bulkReadAddress[1] = FSR_ADDR;
    bulkReadLength[0] = bulkReadLength[1] = FSR_ADDR_LEN;
    readDataLength = 2 * FSR_ADDR_LEN;

    pthread_mutex_lock(&MutexBulkRead);
    RawBulkRead(bulkReadID, readCount, bulkReadAddress, bulkReadLength,
                bulkReadData);
    pthread_mutex_unlock(&MutexBulkRead);
    // std::cout << std::setw(5) << unsigned(bulkReadData[0]) << ":" <<
    // std::setw(5)
    //           << unsigned(bulkReadData[1]) << ":" << std::setw(5)
    //           << unsigned(bulkReadData[2]) << ":" << std::setw(5)
    //           << unsigned(bulkReadData[3]) << "  R----L" << std::setw(5)
    //           << unsigned(bulkReadData[4]) << ":" << std::setw(5)
    //           << unsigned(bulkReadData[5]) << ":" << std::setw(5)
    //           << unsigned(bulkReadData[6]) << ":" << std::setw(5)
    //           << unsigned(bulkReadData[7]) << "#\r";  // \r  //行打印

    for (uint8_t i = 0; i < 4; i++) mWalk.FSR_L[i] = bulkReadData[i + 4];

    for (uint8_t i = 0; i < 4; i++) mWalk.FSR_R[i] = bulkReadData[i];
  } else {
    std_msgs::Float64MultiArray jointPosition =
        SimControll::SimRobotData.getJointPostion();

    for (uint8_t i = 0; i < jointPosition.data.size(); i++) {
      measuredJointPos[i] = WalkingAngleDirection[i] * Radian2Angle(jointPosition.data.at(i));
    }

    std_msgs::Float64MultiArray leftFT = SimControll::SimRobotData.getLeftFT();
    std_msgs::Float64MultiArray rightFT =
        SimControll::SimRobotData.getRightFT();
    // ROS_INFO("leftFT: %f\trightFT: %f", leftFT.data.at(5),
    // rightFT.data.at(5));

    if (leftFT.data.at(5) > 0.5)
      leftFT.data[5] = 220;
    else
      leftFT.data[5] = 0;

    if (rightFT.data.at(5) > 0.5)
      rightFT.data[5] = 220;
    else
      rightFT.data[5] = 0;

    for (uint8_t i = 0; i < 4; i++) {
      mWalk.FSR_L[i] = leftFT.data[5];
      mWalk.FSR_R[i] = rightFT.data[5];
    }
  }
}

void WalkingStatePublish(void) {
  std_msgs::Float64 msg_val;
  static bool walking_flag = 0;

  for (int i = 0; i < 12; ++i) {
    jointPos.data[i] = mWalk.measuredJointValue[i];
  }
  jointPosMeasurePub.publish(jointPos);
  for (int i = 0; i < 12; ++i) {
    jointPos.data[i] = mWalk.jointValue[i];
  }
  jointPosTargetPub.publish(jointPos);

  if (walking_flag != cpWalk->onWalking) {
    walking_flag = cpWalk->onWalking;
    msg_val.data = walking_flag;
    WalkingStatusPub.publish(msg_val);
  }

  // for (int i = 0; i < 12; ++i) {
  //   jointPos.data[i]=cpWalk->measuredJointVelocity[i];
  // }
  // jointVelMeasurePub.publish(jointPos);
  // for (int i = 0; i < 12; ++i) {
  //   jointPos.data[i]=cpWalk->jointVelocity[i];
  // }
  // jointVelTargetPub.publish(jointPos);

  // msg_val.data = JY901Roll;
  // JY901X.publish(msg_val);

  // msg_val.data = JY901Pitch;
  // JY901Y.publish(msg_val);

  // Eigen::Vector2d a;
  // int XorY = 1;

  // if(cpWalk->countTraj <= cpWalk->stepTrajNum && cpWalk->onWalking == true)
  // //
  // {

  // 	for(int i = 0; i < 3; ++i)
  // 	{
  // 		footTraj.data[i]=cpWalk->lFootRefTrajCmd[i];
  // 	}
  // 	// msg_val.data= cpWalk->jointValue[5-1];
  // 	leftFootTraj_pub.publish(footTraj);

  // 	for(int i = 0; i < 3; ++i)
  // 	{
  // 		footTraj.data[i]=cpWalk->rFootRefTrajCmd[i];
  // 	}
  // 	// msg_val.data= cpWalk->jointValue[5-1];
  // 	rightFootTraj_pub.publish(footTraj);

  // 	a =
  // cpWalk->cpRefInWorld;//cpWalk->cpref.block(cpWalk->countTraj,0,1,2).transpose();
  // 	msg_val.data = a(XorY);
  // 	cpref_pub.publish(msg_val);

  // 	a = cpWalk->cpMeasureInWorld;
  // 	msg_val.data = a(XorY);
  // 	cpC_pub.publish(msg_val);

  // 	a = SimControll::cop.segment(0,2);
  // 	msg_val.data = a(XorY);
  // 	copm_pub.publish(msg_val);

  // 	a = cpWalk->copDesir;
  // 	msg_val.data = a(XorY);
  // 	copD_pub.publish(msg_val);

  // 	a = cpWalk->copRefInWorld;
  // 	msg_val.data = a(XorY);
  // 	copref_pub.publish(msg_val);

  // 	a = cpWalk->measuredComInWorld.segment(0,2);
  // 	msg_val.data = a(XorY);
  // 	comm_pub.publish(msg_val);

  // 	a = cpWalk->comDesirInWorld.segment(0,2);
  // 	msg_val.data = a(XorY);
  // 	comD_pub.publish(msg_val);

  // 	a = cpWalk->comRefInWorld.segment(0,2);
  // 	msg_val.data = a(XorY);
  // 	comref_pub.publish(msg_val);

  // 	a = cpWalk->estimatedComInWorld.segment(0,2);
  // 	msg_val.data = a(XorY);
  // 	comEsti_pub.publish(msg_val);

  // 	a = cpWalk->measuredComVelocityInWorld.segment(0,2);
  // 	msg_val.data = a(XorY);
  // 	comvm_pub.publish(msg_val);

  // 	a = cpWalk->comvDesirInWorld.segment(0,2);
  // 	msg_val.data = a(XorY);
  // 	comvD_pub.publish(msg_val);

  // 	a = cpWalk->comvRefInWorld.segment(0,2);
  // 	msg_val.data = a(XorY);
  // 	comvref_pub.publish(msg_val);

  // 	a = cpWalk->estimatedComVelocityInWorld.segment(0,2);
  // 	// a = cpWalk->measuredComVelocity.segment(0,2);
  // 	// a = cpWalk->expectedComVelocity.segment(0,2);
  // 	msg_val.data = a(XorY);
  // 	comvEsti_pub.publish(msg_val);

  // 	if(cpWalk->currentStanceStatus == CPWalking6::LEFT_STANCE)
  // 		msg_val.data = 50;
  // 	else if(cpWalk->currentStanceStatus == CPWalking6::RIGHT_STANCE)
  // 		msg_val.data = -50;
  // 	contactState_pub.publish(msg_val);

  // }

  Eigen::Vector2d a;
  int XorY = 1;

  // if(mWalk.StepCountTarget != 0)				//
  // {
  // 	// msg_val.data = mWalk.CurrentPos.Rfoot_z;
  // 	// leftFootTraj_pub.publish(msg_val);

  // 	// msg_val.data = mWalk.CurrentPos.Lfoot_z;
  // 	// rightFootTraj_pub.publish(msg_val);

  // 	a = mWalk.measuredTorsoInLeft.segment(0,2);
  // 	msg_val.data = a(XorY);
  // 	comm_pub.publish(msg_val);

  // 	a = mWalk.expectedLeftToCom.segment(0,2);
  // 	msg_val.data = a(XorY);
  // 	comD_pub.publish(msg_val);

  // 	a = mWalk.estimatedCom.segment(0,2);
  // 	msg_val.data = a(XorY);
  // 	comEsti_pub.publish(msg_val);

  // 	a = mWalk.measuredVel.segment(0,2);
  // 	msg_val.data = a(XorY);
  // 	comvm_pub.publish(msg_val);

  // 	a = mWalk.expectedComVelocity;
  // 	msg_val.data = a(XorY);
  // 	comvD_pub.publish(msg_val);

  // 	// a = mWalk.estimatedComVelocity;
  // 	// msg_val.data = a(XorY);
  // 	// comvEsti_pub.publish(msg_val);

  // 	a = mWalk.estimatedComVel.segment(0,2);
  // 	msg_val.data = a(XorY);
  // 	comvEsti_pub.publish(msg_val);

  // 	a = mWalk.refComVel;
  // 	msg_val.data = a(XorY);
  // 	comvref_pub.publish(msg_val);

  // 	if(mWalk.StepPhase == LIPMWalk::DoubleSupport)
  // 		msg_val.data = 0;
  // 	else if(mWalk.StepPhase == LIPMWalk::LeftStance)
  // 		msg_val.data = 0.3;
  // 	else if(mWalk.StepPhase == LIPMWalk::RightStance)
  // 		msg_val.data = -0.3;
  // 	contactState_pub.publish(msg_val);
  // }

  if (mWalk.StepCountTarget != 0) {
    // msg_val.data = mWalk.CurrentPos.Rfoot_z;
    // leftFootTraj_pub.publish(msg_val);

    // msg_val.data = mWalk.CurrentPos.Lfoot_z;
    // rightFootTraj_pub.publish(msg_val);

    a = mWalk.measuredComInWorld.segment(0, 2);
    msg_val.data = a(XorY);
    comm_pub.publish(msg_val);

    a = mWalk.comRefInWorld.segment(0, 2);
    msg_val.data = a(XorY);
    comref_pub.publish(msg_val);

    a = mWalk.estimatedComInWorld.segment(0, 2);
    msg_val.data = a(XorY);
    comEsti_pub.publish(msg_val);

    a = mWalk.measuredComVelocityInWorld.segment(0, 2);
    msg_val.data = a(XorY);
    comvm_pub.publish(msg_val);

    a = mWalk.comvRefInWorld.segment(0, 2);
    msg_val.data = a(XorY);
    comvref_pub.publish(msg_val);

    a = mWalk.estimatedComVelocityInWorld.segment(0, 2);
    msg_val.data = a(XorY);
    comvEsti_pub.publish(msg_val);

    msg_val.data = mWalk.PosPara_wF.Lfoot_z;
    LFootZ.publish(msg_val);
    msg_val.data = mWalk.PosPara_wF.Rfoot_z;
    RFootZ.publish(msg_val);

    msg_val.data = mWalk.PosPara_wF.Lfoot_y - mWalk.PosPara_wF.Rfoot_y;
    footDisRef.publish(msg_val);
    msg_val.data = mWalk.CurrentPos_wF.Lfoot_y - mWalk.CurrentPos_wF.Rfoot_y;
    footDis.publish(msg_val);

    if (mWalk.StepPhase == GaitManager::LIPMWalk::DoubleSupport)
      msg_val.data = 0;
    else if (mWalk.StepPhase == GaitManager::LIPMWalk::LeftStance)
      msg_val.data = 0.3;
    else if (mWalk.StepPhase == GaitManager::LIPMWalk::RightStance)
      msg_val.data = -0.3;
    stepPhase_pub.publish(msg_val);

    if (mWalk.ContactState == GaitManager::LIPMWalk::DoubleContact)
      msg_val.data = 0;
    else if (mWalk.ContactState == GaitManager::LIPMWalk::LeftContact)
      msg_val.data = 0.3;
    else if (mWalk.ContactState == GaitManager::LIPMWalk::RightContact)
      msg_val.data = -0.3;
    contactState_pub.publish(msg_val);
  }
}
