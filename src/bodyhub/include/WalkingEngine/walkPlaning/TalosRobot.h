
#ifndef TALOS_ROBOT_H
#define TALOS_ROBOT_H

#include "MultiBody.h"
#include "MultiBodyConfig.h"
#include "MultiBodyGraph.h"

namespace ljnoid {

class TalosRobot {
 public:
  TalosRobot();
  virtual ~TalosRobot();

  void constructRobot();
  void constructTorso();
  void constructLeftLeg();
  void constructRightLeg();
  void constructLeftArm();
  void constructRightArm();

  void modelJointTrajControll();
  void jointParaInit();
  static TalosRobot* GetInstance() { return m_UniqueInstance; }

  rbd::MultiBodyGraph mbg;
  rbd::MultiBody talos;
  rbd::MultiBodyConfig talosmbc;
  rbd::MultiBodyConfig measuredmbc;
  rbd::MultiBodyConfig expectedmbc;

  rbd::MultiBody talosFloat;
  rbd::MultiBodyConfig talosFloatmbc;
  rbd::MultiBodyConfig measuredFloatmbc;

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

    LLEG_JOINT_FREEDOM_NUM = 12,
    LLEG_JOINT_CONFIG_NUM = 13,

  };

  double timeStep;
  Eigen::Matrix<double, JOINT_NUM, 1> jointCommand;

  enum linkId {
    TORSO_LINK = 0,

    LLEG_LINK_1 = 1,
    LLEG_LINK_2 = 2,
    LLEG_LINK_3 = 3,
    LLEG_LINK_4 = 4,
    LLEG_LINK_5 = 5,
    LLEG_LINK_6 = 6,
    LLEG_LINK_SOLE = 7,
    LLEG_LINK_NUM = 7,

    RLEG_LINK_1 = 8,
    RLEG_LINK_2 = 9,
    RLEG_LINK_3 = 10,
    RLEG_LINK_4 = 11,
    RLEG_LINK_5 = 12,
    RLEG_LINK_6 = 13,
    RLEG_LINK_SOLE = 14,
    RLEG_LINK_NUM = 7,

    LARM_LINK_1 = 15,
    LARM_LINK_2 = 16,
    LARM_LINK_3 = 17,
    LARM_LINK_HAND = 18,
    LARM_LINK_NUM = 4,

    RARM_LINK_1 = 19,
    RARM_LINK_2 = 20,
    RARM_LINK_3 = 21,
    RARM_LINK_HAND = 22,
    RARM_LINK_NUM = 4,

    LINK_NUM = 23,
  };

 private:
  /// fixed base
  Eigen::Matrix<double, JOINT_NUM, 1> uPD;
  Eigen::Matrix<double, JOINT_NUM, 1> err;
  Eigen::Matrix<double, JOINT_NUM, 1> errV;

  Eigen::Matrix<double, JOINT_NUM, 1> qref;
  Eigen::Matrix<double, JOINT_NUM, 1> qrefOld;
  Eigen::Matrix<double, JOINT_NUM, 1> q;
  Eigen::Matrix<double, JOINT_NUM, 1> qold;
  Eigen::Matrix<double, JOINT_NUM, 1> dqref;
  Eigen::Matrix<double, JOINT_NUM, 1> dqrefOld;
  Eigen::Matrix<double, JOINT_NUM, 1> dq;

  static TalosRobot* m_UniqueInstance;
};
}

#endif
