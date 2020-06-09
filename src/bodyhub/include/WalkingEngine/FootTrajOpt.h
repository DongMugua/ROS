#ifndef _FootTrajOpt_h_
#define _FootTrajOpt_h_

#include <sys/time.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <iostream>

#include "TaskSolver.h"
#include "Util.h"
#include "limits.h"

using namespace std;
using namespace Eigen;

VectorXd FootTrajOpt(Vector6d bound, double tswing, double timestep,
                     double accratio, double omega) {
  struct timeval start, start2;
  struct timeval end;
  gettimeofday(&start, NULL);

  double AccRatio = accratio;  // 0.3;
  double Omega = omega;        // 800;

  double Tswing = tswing;  // 0.8;
  double dt = timestep;    // 0.01;
  Vector3d beginP = Vector3d(bound(0), bound(2), bound(4));
  Vector3d endP = Vector3d(bound(1), bound(3), bound(5));

  int N = Tswing / dt;
  int nx = 3;
  int nu = 1;

  MatrixXd Adt(3, 3);
  Adt << 1., dt, pow(dt, 2) / 2, 0., 1., dt, 0., 0., 1.;
  MatrixXd bdt(3, 1);
  bdt << pow(dt, 3) / 6, pow(dt, 2) / 2, dt;

  // cost
  SparseMatrix<double> Ajerk(nu * N, nx * (N + 1) + nu * N);
  std::vector<Triplet<double>> Ajerktriplet;
  for (int i = 0; i < nu * N; i++) {
    Ajerktriplet.emplace_back(i, nx * (N + 1) + i, 1);
  }
  Ajerk.setFromTriplets(Ajerktriplet.begin(), Ajerktriplet.end());
  MatrixXd bjerk(nu * N, 1);
  bjerk.setZero();

  // SparseMatrix<double> Aacc(nu*N, nx*(N+1)+nu*N);
  // std::vector< Triplet<double>> Aacctriplet;
  // for (int i=0; i<nu*N; i++){
  //     Aacctriplet.emplace_back(i,nx*i+2,1);
  // }
  // Aacc.setFromTriplets(Aacctriplet.begin(),Aacctriplet.end());
  // MatrixXd bacc(nu*N,1);
  // bacc.setZero();

  // double wjerk = 1;
  // double wacc = 0;

  // SparseMatrix<double,RowMajor>
  // AcostSparse(Ajerk.rows()+Aacc.rows(),Ajerk.cols());
  // AcostSparse.topRows(Ajerk.rows()) = wjerk*Ajerk;
  // AcostSparse.middleRows(Ajerk.rows(),Aacc.rows()) = wacc*Aacc;

  // SparseMatrix<double> Psparse = AcostSparse.transpose()*AcostSparse;

  // MatrixXd bcost(Ajerk.rows()+Aacc.rows(),1);
  // bcost<<wjerk*bjerk, wacc*bacc;

  SparseMatrix<double> Psparse = Ajerk.transpose() * Ajerk;

  MatrixXd q = -Ajerk.transpose() * bjerk;
  for (int i = 0; i < (N + 1) * AccRatio; i++) {
    q(3 * i + 1, 0) += 1;
    q(3 * (N + 1) * AccRatio + 1, 0) += -1;
  }
  q = Omega * q;

  std::vector<c_float> qvec;
  for (int i = 0; i < q.size(); i++) {
    qvec.push_back(q(i));
  }

  // constrains
  SparseMatrix<double> Ax(nx * (N + 1), nx * (N + 1));
  std::vector<Triplet<double>> Axtriplet;
  for (int k = 0; k < N; k++) {
    for (int i = 0; i < nx; i++) {
      for (int j = 0; j < nx; j++) {
        Axtriplet.emplace_back(nx + k * nx + i, k * nx + j, Adt(i, j));
      }
    }
  }
  for (int k = nx * 1; k < nx * (N + 1); k++) {
    Axtriplet.emplace_back(k, k, -1);
  }
  Ax.setFromTriplets(Axtriplet.begin(), Axtriplet.end());

  SparseMatrix<double> Bu(nx * (N + 1), nu * N);
  std::vector<Triplet<double>> Butriplet;
  for (int k = 0; k < N; k++) {
    for (int i = 0; i < nx; i++) {
      for (int j = 0; j < nu; j++) {
        Butriplet.emplace_back(nx + k * nx + i, k * nu + j, bdt(i, j));
      }
    }
  }
  Bu.setFromTriplets(Butriplet.begin(), Butriplet.end());

  SparseMatrix<double, ColMajor> Aeq(Ax.rows(), Ax.cols() + Bu.cols());
  Aeq.leftCols(Ax.cols()) = Ax;
  Aeq.rightCols(Bu.cols()) = Bu;

  SparseMatrix<double> Aeq2(6, nx * (N + 1) + nu * N);
  std::vector<Triplet<double>> Aeq2triplet;
  Aeq2triplet.emplace_back(0, 0, 1);
  Aeq2triplet.emplace_back(1, 1, 1);
  Aeq2triplet.emplace_back(2, 2, 1);
  Aeq2triplet.emplace_back(3, nx * (N + 1) - 3, 1);
  Aeq2triplet.emplace_back(4, nx * (N + 1) - 2, 1);
  Aeq2triplet.emplace_back(5, nx * (N + 1) - 1, 1);
  Aeq2.setFromTriplets(Aeq2triplet.begin(), Aeq2triplet.end());

  SparseMatrix<double> Aineq(N + 1, nx * (N + 1) + nu * N);
  std::vector<Triplet<double>> Aineqtriplet;
  for (int i = (N + 1) * AccRatio; i < (N + 1); i++) {
    Aineqtriplet.emplace_back(i, i * 3 + 2, 1);
  }
  Aineq.setFromTriplets(Aineqtriplet.begin(), Aineqtriplet.end());

  SparseMatrix<double, RowMajor> AsparseAll(Aeq.rows() + Aeq2.rows(),
                                            Aeq.cols());
  AsparseAll.topRows(Aeq.rows()) = Aeq;
  AsparseAll.middleRows(Aeq.rows(), Aeq2.rows()) = Aeq2;
  // AsparseAll.bottomRows(Aineq.rows()) = Aineq;

  std::vector<c_float> lvec, uvec;
  uvec.insert(uvec.begin(), Aeq.rows(), 0);
  uvec.insert(uvec.end(), 1, beginP(0));
  uvec.insert(uvec.end(), 1, beginP(1));
  uvec.insert(uvec.end(), 1, beginP(2));
  uvec.insert(uvec.end(), 1, endP(0));
  uvec.insert(uvec.end(), 1, endP(1));
  uvec.insert(uvec.end(), 1, endP(2));
  // uvec.insert(uvec.end(),Aineq.rows(),1000);

  lvec.insert(lvec.begin(), Aeq.rows(), 0);
  lvec.insert(lvec.end(), 1, beginP(0));
  lvec.insert(lvec.end(), 1, beginP(1));
  lvec.insert(lvec.end(), 1, beginP(2));
  lvec.insert(lvec.end(), 1, endP(0));
  lvec.insert(lvec.end(), 1, endP(1));
  lvec.insert(lvec.end(), 1, endP(2));
  // lvec.insert(lvec.end(),Aineq.rows(),-1000);

  // solution
  OSQPTasks::TaskSolver TrajSolver;
  TrajSolver.update(Psparse, qvec, AsparseAll, lvec, uvec);
  VectorXd solution = TrajSolver.solver();

  double time =
      1000000 * (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec);
  std::cout << "time(ms):" << time / 1000 << std::endl;

  VectorXd posTraj(N + 1);
  for (int i = 0; i < N + 1; ++i) {
    posTraj(i) = solution(3 * i);
  }
  return posTraj;
}

#endif