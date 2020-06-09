#ifndef INERTIAL_DATA_FILTER_H
#define INERTIAL_DATA_FILTER_H

#include "SimControll.h"
#include "UnscentedKalmanFilter.h"
#include "Util.h"

// #include "Kalman.h"

struct InertialData {
  Eigen::Vector3d gyro =
      Eigen::Vector3d::Zero(); /*< The change in orientation around the
                                  x-, y-, and z-axis (in radian/s). */
  Eigen::Vector3d acc = Eigen::Vector3d::
      Zero(); /**< The acceleration along the x-, y- and z-axis (in m/s^2). */
  Eigen::Vector3d angle = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation2D =
      Eigen::Quaterniond::Identity(); /** The orientation of the torso
                                         represented as a quaternion
                                         without the z-Rotation. */
  Eigen::Quaterniond orientation3D =
      Eigen::Quaterniond::Identity(); /** The orientation of the torso
                                         represented as a quaternion including
                                         the z-Rotation. */
};

class InertialDataFilter {
 public:
  InertialData inertialData;

  void update(InertialData& inertialData);
  void kalmanUpdate(InertialData& inertialData);

 private:
  Eigen::Vector3d gyroDeviation = Eigen::Vector3d(
      0.003 * Util::TO_RADIAN, 0.003 * Util::TO_RADIAN,
      0.003 * Util::TO_RADIAN);  // Eigen::Vector3d(0.03*Util::TO_DEGREE,
                                 // 0.03*Util::TO_DEGREE, 0.03*Util::TO_DEGREE);
                                 // // Noise of the gyro in °/s / sqrt(Hz).
  Eigen::Vector3d accDeviation = Eigen::Vector3d(10., 10., 10.);              //
  Eigen::Vector3d accDeviationWhileWalking = Eigen::Vector3d(50., 50., 50.);  //

  struct State : public Manifold<3> {
    Eigen::Quaterniond orientation;

    State(
        const Eigen::Quaterniond& orientation = Eigen::Quaterniond::Identity())
        : orientation(orientation) {
      // orientation = Quaternionf::Identity(); // Workaround, usual
      // initialization crashes in Debug
    }

    State operator+(const Vectord& angleAxis) const;
    State& operator+=(const Vectord& angleAxis);
    Vectord operator-(const State& other) const;
  };

  UKFM<State> ukf = UKFM<State>(State());

  Eigen::Vector3d lastRawAngle = Eigen::Vector3d::Zero();

  void estimate(const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc,
                double timePassed, const Eigen::Vector3d& gyroDeviation,
                const Eigen::Vector3d& accDeviation);

  ///////////////////////////////IMU//////////////////////////////////////////
  // Kalman kalmanX;//X方向为Roll
  // Kalman kalmanY;//Y方向为Pitch

  /* All the angles start at 180 degrees */
  double gyroXangle = 0;
  double gyroYangle = 0;
  // Complimentary filter
  double compAngleX = 0;
  double compAngleY = 0;
  double kalAngleX = 0;
  double kalAngleY = 0;  // Calculated angle using a Kalman filter
};

namespace AngleAxisOpera {
// Do not missinterpret the vectors as Euler angles!
Eigen::Vector3d pack(const Eigen::AngleAxisd& angleAxis);
Eigen::AngleAxisd unpack(const Eigen::Vector3d& angleAxisVec);

Eigen::Quaterniond removeZRotation(const Eigen::Quaterniond& rotation);
};

#endif