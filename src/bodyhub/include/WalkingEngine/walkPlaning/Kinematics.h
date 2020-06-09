#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Eigen/Dense>
#include "Util.h"

namespace Chains {
enum Chain {
  head,
  leftArm,
  rightArm,
  leftLeg,
  rightLeg,
  numOfChains,
};
}

namespace Kinematics {

#define pi Util::PI

extern Eigen::Matrix4d _RLeg_T[8];
extern Eigen::Matrix4d _RLeg_ABase0, _RLeg_Ryp2, _RLeg_A7RFoot;
extern Eigen::Matrix4d _LLeg_T[8];
extern Eigen::Matrix4d _LLeg_ABase0, _LLeg_Ryp2, _LLeg_A7LFoot;

template <typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(
    const _Matrix_Type_& a,
    double epsilon = std::numeric_limits<double>::epsilon());

Eigen::Matrix4d init_transform(double a, double al, double d, double t);

void get_LLeg_transform(Eigen::Matrix<double, 6, 1> t);

void get_RLeg_transform(Eigen::Matrix<double, 6, 1> t);

Eigen::Matrix4d forward_kinematics(Eigen::Matrix<double, 6, 1> t,
                                   Chains::Chain chain);

// Modified D-H
Eigen::Matrix<double, 6, 6> calc_jacobi(Eigen::Matrix<double, 6, 1> t,
                                        Chains::Chain chain);

Eigen::Matrix<double, 6, 1> calc_del_theta_DLS(
    Eigen::Matrix<double, 6, 1> theta, Eigen::Matrix<double, 6, 1> del_p,
    Chains::Chain chain);

Eigen::Matrix<double, 6, 1> cal_del_pos(Eigen::Matrix4d c_pos,
                                        Eigen::Matrix4d d_pos);

bool inverse_kinematics1(Eigen::Matrix4d p, Chains::Chain chain,
                         Eigen::Matrix<double, 6, 1>& theta);
bool inverse_kinematics(Eigen::Matrix4d p, Chains::Chain chain,
                        Eigen::Matrix<double, 6, 1>& theta);

Eigen::Matrix<double, 6, 1> RLeg_inverse_kinematics(Eigen::Matrix4d p);
Eigen::Matrix<double, 6, 1> LLeg_inverse_kinematics(Eigen::Matrix4d p);
}

#endif