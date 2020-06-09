#include "bodyhub/bodyhub.h"

#define DEGUG_EN 0

pthread_mutex_t MtuexSensorList;
pthread_mutex_t MtuexSensorControl;

std::map<std::string, uint8_t> SensorNameIDMap;  // <sensor, ID>
std::list<SensorModule *> SensorsList;
std::queue<SensorControl_t> SensorControlQueue;
std::map<uint8_t, uint8_t *> SensorDataMap;  // <sensorID, data>

ros::ServiceServer RegistSensorService;
ros::ServiceServer DeleteSensorService;

ros::Publisher SensorRawDataPub;
ros::Subscriber SensorControlSub;

bool SensorWrite(uint8_t WriteCount, uint8_t *WriteID, uint16_t *WriteAddress,
                 uint16_t *WriteLenght,
                 uint8_t WriteData[][CONTROL_DATA_SIZE_MAX]) {
  const char *log = NULL;
  bool result = false;

  for (uint8_t index = 0; index < WriteCount; index++) {
    result = dxlTls.writeRegister(WriteID[index], WriteAddress[index],
                                  WriteLenght[index], WriteData[index], &log);
    if (result == false) {
      ROS_ERROR("SensorWrite(): %s", log);
      return false;
    }
    usleep(500);
  }
  return true;
}

void SensorbulkReadPoll() {
  pthread_mutex_lock(&MtuexSensorList);
  if (SensorsList.size()) {
    bodyhub::SensorRawData sensorRawDataMsg;

    uint8_t readCount = SensorsList.size();
    uint8_t bulkReadID[readCount];
    uint16_t bulkReadAddress[readCount];
    uint16_t bulkReadLength[readCount];
    uint8_t readDataLength = 0;
    uint8_t idNum = 0;
    for (auto s_it = SensorsList.begin(); s_it != SensorsList.end(); s_it++) {
      bulkReadID[idNum] = (*s_it)->sensorID;
      bulkReadAddress[idNum] = (*s_it)->startAddr;
      bulkReadLength[idNum] = (*s_it)->length;
      readDataLength += (*s_it)->length;
      idNum++;
    }

    int32_t bulkReadData[readDataLength];
    uint8_t dataIndex = 0;
    pthread_mutex_lock(&MutexBulkRead);
    RawBulkRead(bulkReadID, readCount, bulkReadAddress, bulkReadLength,
                bulkReadData);
    pthread_mutex_unlock(&MutexBulkRead);
    // bulkReadData[] -> SensorDataMap
    for (idNum = 0; idNum < readCount; idNum++) {
      for (uint8_t i = 0; i < bulkReadLength[idNum]; i++) {
        SensorDataMap[bulkReadID[idNum]][i] = bulkReadData[dataIndex];
        dataIndex++;
      }
    }

    // publish sensorData
    for (idNum = 0; idNum < readCount; idNum++) {
      sensorRawDataMsg.sensorReadID.push_back(bulkReadID[idNum]);
      sensorRawDataMsg.sensorStartAddress.push_back(bulkReadAddress[idNum]);
      sensorRawDataMsg.sensorReadLength.push_back(bulkReadLength[idNum]);
    }
    for (idNum = 0; idNum < readDataLength; idNum++)
      sensorRawDataMsg.sensorData.push_back(bulkReadData[idNum]);
    sensorRawDataMsg.sensorCount = readCount;
    sensorRawDataMsg.dataLength = readDataLength;
    SensorRawDataPub.publish(sensorRawDataMsg);
  }
  pthread_mutex_unlock(&MtuexSensorList);
}

void SensorControlPoll() {
  pthread_mutex_lock(&MtuexSensorControl);
  if (!SensorControlQueue.empty()) {
    uint8_t size = SensorControlQueue.size();
    uint8_t WriteID[size];
    uint16_t WriteAddress[size];
    uint16_t WriteLenght[size];
    uint8_t WriteData[size][CONTROL_DATA_SIZE_MAX];
    uint8_t index = 0;
    SensorControl_t SensorControl;
    while (!SensorControlQueue.empty()) {
      SensorControl = SensorControlQueue.front();
      WriteID[index] = SensorControl.id;
      WriteAddress[index] = SensorControl.addr;
      WriteLenght[index] = SensorControl.lenght;
      memcpy(WriteData[index], SensorControl.data, SensorControl.lenght);
      SensorControlQueue.pop();
      index++;
    }
    SensorWrite(size, WriteID, WriteAddress, WriteLenght, WriteData);
  }
  pthread_mutex_unlock(&MtuexSensorControl);
}

bool SensorPing(std::string name) {
  bool result = false;
  const char *log = NULL;
  result = dxlTls.ping(SensorNameIDMap.at(name), &log);
  if (result == false) {
    ROS_ERROR("SensorPing(): %s", log);
    return false;
  }
  return true;
}

void LoadSensorNameID(const std::string path) {
  uint8_t mmmm = 138;
  YAML::Node sensorNameIDDoc;
  try {
    sensorNameIDDoc = YAML::LoadFile(path.c_str());
  } catch (const std::exception &e) {
    ROS_WARN("Fail to load sensorNameID yaml.");
    return;
  }
  YAML::Node itemData = sensorNameIDDoc["sensorNameID"];
  if (itemData.size() == 0) return;

  for (YAML::const_iterator itItemNum = itemData.begin();
       itItemNum != itemData.end(); itItemNum++) {
    std::string sensorName = itItemNum->first.as<std::string>();
    int sensorID = itItemNum->second.as<int>();

    SensorNameIDMap[sensorName] = sensorID;
  }
}

void addSensorModule(SensorModule *tempSensor) {
  pthread_mutex_lock(&MtuexSensorList);
  // check whether the nodule length is zero
  if (tempSensor->length == 0) {
    ROS_ERROR("sensorBulkReadLength is illegal.");
    delete tempSensor;
    return;
  }
  // check whether the module name already exists
  for (auto s_it = SensorsList.begin(); s_it != SensorsList.end(); s_it++) {
    if ((*s_it)->sensorName == tempSensor->sensorName) {
      (*s_it)->length = tempSensor->length;
      (*s_it)->startAddr = tempSensor->startAddr;

      SensorDataMap.erase(tempSensor->sensorID);
      SensorDataMap[tempSensor->sensorID] = new uint8_t[tempSensor->length];
      delete tempSensor;
      return;
    }
  }
  try {
    tempSensor->sensorID = SensorNameIDMap.at(tempSensor->sensorName);
  } catch (const std::out_of_range &e) {
    ROS_ERROR("%s -- [%s] sensorName is illegal.", e.what(),
              tempSensor->sensorName.c_str());
    return;
  }

  SensorsList.push_back(tempSensor);
  SensorsList.unique();

  SensorDataMap[tempSensor->sensorID] = new uint8_t[tempSensor->length];
#if DEGUG_EN
  for (const auto &sensorN : SensorsList)
    std::cout << sensorN->sensorName << std::endl;
#endif
  pthread_mutex_unlock(&MtuexSensorList);
}

bool removeSensorModlue(std::string name) {
  pthread_mutex_lock(&MtuexSensorList);
  for (auto s_it = SensorsList.begin(); s_it != SensorsList.end(); s_it++) {
    if ((*s_it)->sensorName == name) {
      delete[] SensorDataMap[(*s_it)->sensorID];
      SensorDataMap.erase((*s_it)->sensorID);
      SensorsList.remove(*s_it);
      delete *s_it;
#if DEGUG_EN
      for (const auto &sensorN : SensorsList)
        std::cout << sensorN->sensorName << std::endl;
#endif
      pthread_mutex_unlock(&MtuexSensorList);
      return true;
    }
  }
  pthread_mutex_unlock(&MtuexSensorList);
  return false;
}

bool RegistSensorCallback(bodyhub::SrvInstWrite::Request &req,
                          bodyhub::SrvInstWrite::Response &res) {
  if (SensorNameIDMap.empty()) {
    ROS_ERROR("RegistSensorCallback(): The map of names is empty!");
    res.complete = false;
    return false;
  }
  if (SensorNameIDMap.count(req.itemName) == 0) {
    ROS_ERROR("RegistSensorCallback(): Name of not found!");
    res.complete = false;
    return false;
  }
  if (SensorPing(req.itemName) == false) {
    ROS_ERROR("RegistSensorCallback(): No Ping to the specified sensor!");
    res.complete = false;
    return false;
  }
  SensorModule *newSensor = new SensorModule;
  newSensor->sensorName = req.itemName;
  newSensor->startAddr = req.dxlID;
  newSensor->length = (uint8_t)req.setData;
  addSensorModule(newSensor);

  res.complete = true;
  return true;
}

bool DeleteSensorCallback(bodyhub::SrvTLSstring::Request &req,
                          bodyhub::SrvTLSstring::Response &res) {
  if (removeSensorModlue(req.str) == true) {
    res.data = 1;
  } else {
    res.data = 0;
  }

  return true;
}

void SensorControlCallback(const bodyhub::SensorControl::ConstPtr &msg) {
  if (SensorNameIDMap.empty()) {
    ROS_ERROR("SensorControlCallback(): The map of names is empty!");
    return;
  }
  if (SensorNameIDMap.count(msg->SensorName) == 0) {
    ROS_ERROR("SensorControlCallback(): Name of not found!");
    return;
  }
  SensorControl_t ControlParamete;
  ControlParamete.id = SensorNameIDMap.at(msg->SensorName);
  ControlParamete.addr = msg->SetAddr;
  ControlParamete.lenght = msg->ParamList.size();
  // ROS_INFO("name: %s, id: %d, addr: %d, paramLen: %d",
  // msg->SensorName.c_str(), ControlParamete.id, ControlParamete.addr,
  // ControlParamete.lenght);
  if (ControlParamete.lenght > (CONTROL_DATA_SIZE_MAX - 1)) {
    ROS_ERROR("SensorControlCallback(): Too many parameters!");
    return;
  }
  for (uint8_t i = 0; i < ControlParamete.lenght; i++)
    ControlParamete.data[i] = msg->ParamList[i];

  pthread_mutex_lock(&MtuexSensorControl);
  SensorControlQueue.push(ControlParamete);
  pthread_mutex_unlock(&MtuexSensorControl);
}

void SensorControlInit() {
  ros::NodeHandle nodeHandle;
  SensorControlSub =
      nodeHandle.subscribe("/BodyHub/SensorControl", 2, SensorControlCallback);
}

void ExtendSensorInit() {
  pthread_mutex_init(&MtuexSensorList, NULL);
  pthread_mutex_init(&MtuexSensorControl, NULL);

  if (sensorNameIDFile != "") LoadSensorNameID(sensorNameIDFile);

  ros::NodeHandle nh;
  RegistSensorService = nh.advertiseService("MediumSize/BodyHub/RegistSensor",
                                            RegistSensorCallback);
  DeleteSensorService = nh.advertiseService("MediumSize/BodyHub/DeleteSensor",
                                            DeleteSensorCallback);
  SensorControlInit();

  SensorRawDataPub =
      nh.advertise<bodyhub::SensorRawData>("MediumSize/BodyHub/SensorRaw", 10);
}

void ExtendSensorThread() {
  if (SimControll::simEnable) return;
  ROS_INFO("Start 'ExtendSensorThread' thread...");
  ExtendSensorInit();

  ros::Rate loop_rate(1000);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return;
}

void ExtendSensorTimerThread() {
  if (SimControll::simEnable) return;
  ROS_INFO("Start 'ExtendSensorTimerThread' thread...");
  uint32_t timeCycle = 50;  // ms
  static struct timespec nextTime;
  clock_gettime(CLOCK_MONOTONIC, &nextTime);

  while (ros::ok()) {
    nextTime.tv_sec += (nextTime.tv_nsec + timeCycle * 1000000) / 1000000000;
    nextTime.tv_nsec = (nextTime.tv_nsec + timeCycle * 1000000) % 1000000000;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &nextTime,
                    NULL);  // start run

    SensorbulkReadPoll();  // SensorsList bulkRead
    SensorControlPoll();
  }
  return;
}