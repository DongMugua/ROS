#ifndef SIMCONTROLL_H
#define SIMCONTROLL_H

#include <ros/ros.h>
#include <iostream>
#include <queue>
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"

namespace SimControll {

class ClassRobotData {
 private:
  std_msgs::Float64MultiArray jointPosition;
  std_msgs::Float64MultiArray jointVelocity;
  std_msgs::Float64MultiArray leftFT;
  std_msgs::Float64MultiArray rightFT;

 public:
  ClassRobotData() {
    jointPosition.data.resize(22);
    jointVelocity.data.resize(22);
    leftFT.data.resize(6);
    rightFT.data.resize(6);
  }

  void setJointPosition(std_msgs::Float64MultiArray parameter) {
    jointPosition.layout = parameter.layout;
    jointPosition.data = parameter.data;
  }

  void setJointVelocity(std_msgs::Float64MultiArray parameter) {
    jointVelocity.layout = parameter.layout;
    jointVelocity.data = parameter.data;
  }

  void setLeftFT(std_msgs::Float64MultiArray parameter) {
    leftFT.layout = parameter.layout;
    leftFT.data = parameter.data;
  }

  void setRightFT(std_msgs::Float64MultiArray parameter) {
    rightFT.layout = parameter.layout;
    rightFT.data = parameter.data;
  }

  std_msgs::Float64MultiArray getJointPostion() { return jointPosition; }

  std_msgs::Float64MultiArray getJointVelocity() { return jointVelocity; }

  std_msgs::Float64MultiArray getLeftFT() { return leftFT; }

  std_msgs::Float64MultiArray getRightFT() { return rightFT; }
};

extern bool simEnable;
extern int8_t simState;
extern ClassRobotData SimRobotData;

void simInit(ros::NodeHandle nh);
void simStart();
void simStop();
void simPause();
void updateJointCmdQueue(const std::vector<double> &jointPositions);
void SimStepDoneCallback(const std_msgs::Bool::ConstPtr &msg);
void SimStateCallback(const std_msgs::Int32::ConstPtr &msg);

void simControll(Eigen::Matrix<double, 20, 1> jointValue);
void simJoinPositionPublish(std_msgs::Float64MultiArray jointValue);

void simulateThread();
}  // namespace SimControll

#endif