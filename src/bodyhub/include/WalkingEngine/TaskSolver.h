#ifndef _TaskSolver_h_
#define _TaskSolver_h_

#include <limits.h>
#include <Eigen/../unsupported/Eigen/KroneckerProduct>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "osqp.h"

#include <MultiBodyConfig.h>

namespace OSQPTasks {

class TaskSolver {
 public:
  TaskSolver();
  virtual ~TaskSolver();

  csc* EigenSparseToCSC(const Eigen::SparseMatrix<c_float>& mat);

  void update(const Eigen::MatrixXd& P, const Eigen::VectorXd& q,
              const Eigen::MatrixXd& A, const Eigen::VectorXd& l,
              const Eigen::VectorXd& u);

  void update(const Eigen::SparseMatrix<double>& P, std::vector<c_float>& q,
              const Eigen::SparseMatrix<double>& A, std::vector<c_float>& l,
              std::vector<c_float>& u);

  void updateqlu(std::vector<c_float>& q, std::vector<c_float>& l,
                 std::vector<c_float>& u);

  void updateq(std::vector<c_float>& q);

  Eigen::VectorXd solver();

 private:
  // Problem settings
  OSQPSettings* settings_;

  // Structures
  OSQPWorkspace* work_;
  OSQPData* data_;
};

}  // namespace

#endif
