// #pragma once

// #include "Representations/Sensing/InertialData.h"
// #include "Representations/Sensing/RobotModel.h"
// #include "Tools/Math/Eigen.h"
// #include "Tools/Math/UnscentedKalmanFilter.h"
// #include "Tools/Motion/LIP3D.h"

// STREAMABLE(LIPStateEstimatorParameters,
// {
//   void onRead(),

//   (Vector2f)(20, 20) positionProcessDeviation, // Standard deviation of the
//   position in the prediction step (in mm per second).
//   (Vector2f)(20, 20) velocityProcessDeviation, // Standard deviation of the
//   velocity in the prediction step (in mm/s per second).
//   (Vector2f)(20, 20) zmpProcessDeviation, // Standard deviation of the zmp in
//   the prediction step (in mm per second).
//   (Vector2f)(20, 20) positionMeasurementDeviation, // Standard deviation of
//   the zmp in the measurement step (in mm per second).
//   (Vector2f)(20, 20) zmpMeasurementDeviation, // Standard deviation of the
//   zmp in the measurement step (in mm per second).
// });

// class LIPStateEstimator
// #ifdef TARGET_ROBOT
//   : public AlignedMemory
// #endif
// {
// public:
//   enum class SupportFoot
//   {
//     left = 1,
//     right = -1
//   };

//   struct EstimatedState
//   {
//     LIP3D com; // relative to origin
//     Vector2f zmp = Vector2f::Zero(); // relative to origin
//     SupportFoot supportFoot = SupportFoot::right;
//     Vector2f origin = Vector2f::Zero();

//     EstimatedState(const Array2f& comHeights) : com(comHeights) {};

//     EstimatedState& update(float time) { com.update(time, zmp); return *this;
//     }
//     EstimatedState predict(float time) const { return
//     EstimatedState(*this).update(time); }
//   };

//   LIPStateEstimator(const InertialData& theInertialData, const RobotModel&
//   theRobotModel);

//   void init(const Array2f& LIPHeights, const Vector2f& leftOrigin,
//   LIPStateEstimatorParameters params);
//   void update(float timePassed, const Array2f& LIPHeights, const Vector2f&
//   leftOrigin, const SupportFoot newSupportFoot);
//   void update(float timePassed, const Array2f& LIPHeights, const Vector2f&
//   leftOrigin);

//   EstimatedState getEstimate() const;
//   EstimatedState convertToOtherFoot(const EstimatedState& state) const;
//   SupportFoot guessSupportFoot(const Vector2f& leftOrigin) const;
//   void draw(float footOffsetY, float forwardingTime) const;
//   void plot() const;

// private:
//   Array2f LIPHeights = Array2f::Zero();
//   SupportFoot supportFoot = SupportFoot::right;
//   Vector2f origin = Vector2f::Zero();

//   UKF<6> ukf = UKF<6>(Vector6f::Zero()); // The statevector of the ukf is
//   composed of: position, velocity, zmp;

//   const InertialData& theInertialData;
//   const RobotModel& theRobotModel;
//   LIPStateEstimatorParameters params;

//   Vector4f measure(SupportFoot supportFoot, const Vector2f& LIPOrigin) const;
// };

#ifndef LIP_STATE_ESTIMATOR_H
#define LIP_STATE_ESTIMATOR_H

#include "InertialDataFilter.h"
#include "SimControll.h"

using Vector8d = Eigen::Matrix<double, 8, 1>;

struct LIPStateEstimatorParameters {
  Eigen::Vector2d positionProcessDeviation =
      Eigen::Vector2d(0.05, 0.05);  // Standard deviation of the position in the
                                    // prediction step (in m per second).
  Eigen::Vector2d velocityProcessDeviation =
      Eigen::Vector2d(0.1, 0.1);  // Standard deviation of the velocity in the
                                  // prediction step (in m/s per second).
  Eigen::Vector2d cpProcessDeviation =
      Eigen::Vector2d(0.05, 0.05);  // Standard deviation of the zmp in the
                                    // prediction step (in m per second).
  Eigen::Vector2d zmpProcessDeviation =
      Eigen::Vector2d(0.05, 0.05);  // Standard deviation of the zmp in the
                                    // prediction step (in m per second).
  Eigen::Vector2d positionMeasurementDeviation =
      Eigen::Vector2d(0.01, 0.01);  // Standard deviation of the zmp in the
                                    // measurement step (in m per second).
  Eigen::Vector2d velocityMeasurementDeviation = Eigen::Vector2d(0.01, 0.01);
  Eigen::Vector2d cpMeasurementDeviation =
      Eigen::Vector2d(0.01, 0.01);  // Standard deviation of the zmp in the
                                    // measurement step (in m per second).
  Eigen::Vector2d zmpMeasurementDeviation =
      Eigen::Vector2d(0.01, 0.01);  // Standard deviation of the zmp in the
                                    // measurement step (in m per second).
};

class LIPStateEstimator {
 public:
  enum class SupportFoot { left = 1, right = -1 };

  struct EstimatedState {
    Eigen::Vector2d com;     // relative to current stance foot
    Eigen::Vector2d comVel;  // relative to current stance foot
    Eigen::Vector2d cp;      // relative to current stance foot
    Eigen::Vector2d zmp =
        Eigen::Vector2d::Zero();  // relative to current stance foot
    SupportFoot supportFoot = SupportFoot::right;

    EstimatedState(){};

    EstimatedState& update(double time, Eigen::Vector2d cpcmd) {
      double TimeCon_x = std::sqrt(9.81 / 0.442);  // sqrt(g/H)
      double TimeCon_y = std::sqrt(9.81 / 0.442);  // sqrt(g/H)
      Eigen::MatrixXd k(2, 2);
      k << TimeCon_x, 0., 0., TimeCon_y;

      Eigen::Vector2d position = com;
      Eigen::Vector2d velocity = comVel;
      // std::cout<< "comVel: " <<  comVel.transpose() <<std::endl;
      Eigen::Vector2d newPosition = position - k * (position - cp) * time;
      comVel = -k * (newPosition - cp);
      Eigen::Vector2d newcp = cp + k * (cp - zmp) * time;  // cpcmd;//

      com = newPosition;
      cp = newcp;
      return *this;
    }
    // EstimatedState predict(double time) const { return
    // EstimatedState(*this).update(time); }
  };

  LIPStateEstimator();
  void init(InertialData& inertialData, Eigen::Vector2d cpinit,
            Eigen::Vector2d copinit);

  void update(double timePassed, const SupportFoot newSupportFoot,
              InertialData& inertialData, Eigen::Vector2d cpcmd);

  EstimatedState getEstimate() const;
  EstimatedState convertToOtherFoot(const EstimatedState& state);

 private:
  SupportFoot supportFoot = SupportFoot::right;

  UKF<8> ukf = UKF<8>(Vector8d::Zero());  // The statevector of the ukf is
                                          // composed of: position, velocity,
                                          // zmp;

  InertialData theInertialData;
  LIPStateEstimatorParameters params;

  Vector8d measure(SupportFoot supportFoot);  // const;

  Eigen::Vector2d comMeasure;
  Eigen::Vector2d lastcomMeasure;
  Eigen::Vector2d comvMeasure;
  Eigen::Vector2d lastcomvMeasure;
  Eigen::Vector2d cpMeasure;
};

#endif
