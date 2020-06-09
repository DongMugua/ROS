#include "bodyhub/bodyhub.h"
// #include "Util.h"

namespace SimControll {

ros::Publisher JointCmdPub;
ros::Publisher SimStartPub;
ros::Publisher SimStopPub;
ros::Publisher SimPausePub;
ros::Publisher SimEnSyncPub;
ros::Publisher SimTrigNextPub;
ros::Subscriber SimStepDoneSub;
ros::Subscriber SimStateSub;

ros::Subscriber SimJointPositionSub;
ros::Subscriber SimJointVelocitySub;
ros::Subscriber SimLeftFTSub;
ros::Subscriber SimRightFTSub;

pthread_mutex_t mtxJV;

bool stepDone = false;   //一步仿真完成标志
bool simEnable = false;  //仿真启动标志
int8_t simState = 0;
std_msgs::Bool simCtrMsg;
std::queue<std::vector<double>> jointCmdQueue;

ClassRobotData SimRobotData;

void SimJointPositionCallback(
    const std_msgs::Float64MultiArray::ConstPtr &msg) {
  SimRobotData.setJointPosition(*msg);
}

void SimJointVelocityCallback(
    const std_msgs::Float64MultiArray::ConstPtr &msg) {
  SimRobotData.setJointVelocity(*msg);
}

void SimLeftFootFTCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
  SimRobotData.setLeftFT(*msg);
}

void SimRightFootFTCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
  SimRobotData.setRightFT(*msg);
}

void SimulateRobotTopicInit(ros::NodeHandle nh) {
  SimJointPositionSub = nh.subscribe<std_msgs::Float64MultiArray>(
      "/sim/joint/angle", 1, &SimJointPositionCallback);
  SimJointVelocitySub = nh.subscribe<std_msgs::Float64MultiArray>(
      "/sim/joint/velocity", 1, &SimJointVelocityCallback);
  SimLeftFTSub = nh.subscribe<std_msgs::Float64MultiArray>(
      "/sim/force/leftFoot", 1, &SimLeftFootFTCallback);
  SimRightFTSub = nh.subscribe<std_msgs::Float64MultiArray>(
      "/sim/force/rightFoot", 1, &SimRightFootFTCallback);

  JointCmdPub =
      nh.advertise<std_msgs::Float64MultiArray>("/sim/joint/command", 1);
}

void simInit(ros::NodeHandle nh)
{
  pthread_mutex_init(&mtxJV, NULL);

  SimStepDoneSub = nh.subscribe<std_msgs::Bool>("simulationStepDone", 10, &SimStepDoneCallback);
  SimStateSub = nh.subscribe<std_msgs::Int32>("simulationState", 1, &SimStateCallback);

  SimStartPub = nh.advertise<std_msgs::Bool>("startSimulation", 1);
  SimStopPub = nh.advertise<std_msgs::Bool>("stopSimulation", 1);
  SimPausePub = nh.advertise<std_msgs::Bool>("pauseSimulation", 1);
  SimEnSyncPub = nh.advertise<std_msgs::Bool>("enableSyncMode", 1);
  SimTrigNextPub = nh.advertise<std_msgs::Bool>("triggerNextStep", 1);

  SimulateRobotTopicInit(nh);

  //等待发布者与接收者建立连接
  ROS_INFO("Waiting for connection with Vrep......");
  while (ros::ok() && (SimStartPub.getNumSubscribers() <= 0 ||
                       SimEnSyncPub.getNumSubscribers() <= 0 ||
                       SimTrigNextPub.getNumSubscribers() <= 0))
    ;
  ROS_INFO("Connection with Vrep completed!!!");
  simCtrMsg.data = 1;               //仿真控制变量
  SimEnSyncPub.publish(simCtrMsg);  //开启vrep同步模式
  SimStartPub.publish(simCtrMsg);   //开始vrep仿真
}

void SimStepDoneCallback(const std_msgs::Bool::ConstPtr &msg) {
  if (msg->data) stepDone = true;
}

void SimStateCallback(const std_msgs::Int32::ConstPtr &msg) {
  // pause--2 start--1
  simState = msg->data;
  // ROS_INFO("Simulation state update: %d", msg->data);
}

void updateJointCmdQueue(const std::vector<double> &jointPositions) {
  pthread_mutex_lock(&mtxJV);
  jointCmdQueue.push(jointPositions);  // radian vector
  pthread_mutex_unlock(&mtxJV);
}

void simStart() {
  simCtrMsg.data = 1;              //仿真控制变量
  SimStartPub.publish(simCtrMsg);  //开始vrep仿真
  ROS_INFO("Simulation in Vrep started!");
}

void simStop() {
  simCtrMsg.data = 1;             //仿真控制变量
  SimStopPub.publish(simCtrMsg);  //停止vrep仿真
  ROS_INFO("Simulation in Vrep stopped!");
}

void simPause() {
  simCtrMsg.data = 1;              //仿真控制变量
  SimPausePub.publish(simCtrMsg);  //暂停vrep仿真
  ROS_INFO("Simulation in Vrep paused!");
}

void simJoinPositionPublish(std_msgs::Float64MultiArray jointValue) {
  jointValue.data.resize(22);
  JointCmdPub.publish(jointValue);  // 转发数据
}

void JointControlMessagePoll() {
  if ((motoQueue.size() > 0) || (headCtrlQueue.size() > 0)) {
    if (motoQueue.size() > 0) {
      bodyhub::JointControlPoint jointControlMsg;
      pthread_mutex_lock(&mtxMo);
      jointControlMsg = motoQueue.front();
      motoQueue.pop();
      pthread_mutex_unlock(&mtxMo);
      for(uint8_t i=0;i<jointControlMsg.positions.size();i++)
        jointControlMsg.positions[i] = Angle2Radian(jointControlMsg.positions[i]);
      updateJointCmdQueue(jointControlMsg.positions);
    }
    if (headCtrlQueue.size() > 0) {
      bodyhub::JointControlPoint headControlMsg;
      pthread_mutex_lock(&mtxHe);
      headControlMsg = headCtrlQueue.front();
      headCtrlQueue.pop();
      pthread_mutex_unlock(&mtxHe);
      for(uint8_t i=0;i<headControlMsg.positions.size();i++)
        headControlMsg.positions[i] = Angle2Radian(headControlMsg.positions[i]);
      updateJointCmdQueue(headControlMsg.positions);
    }
  }
}

void simulateThread() {
  if (!SimControll::simEnable) return;

  ROS_INFO("Start 'simulateThread' thrand...");

  std_msgs::Float64MultiArray jointCmd;
  stepDone = true;
  ros::Rate loop_rate(1000);

  while (ros::ok()) {
    if (stepDone == true) {
      if (jointCmdQueue.empty()) 
      {
        if (bodyhubState == StateEnum::walking)
          control_thread_robot();
        else
          JointControlMessagePoll();
      }
      if (!jointCmdQueue.empty()) 
      {
        pthread_mutex_lock(&mtxJV);
        jointCmd.data.assign(jointCmdQueue.front().begin(),
                             jointCmdQueue.front().end());
        jointCmdQueue.pop();
        pthread_mutex_unlock(&mtxJV);
        simJoinPositionPublish(jointCmd);
      }

      stepDone = false;
      SimTrigNextPub.publish(simCtrMsg);  // Vrep仿真一步,大约要150ms才能收到simulationStepDone消息
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("'simulateThread' thrand exit");
}

}  // namespace SimControll