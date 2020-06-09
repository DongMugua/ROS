#ifndef CORIOLIS_H_KED6CAHK
#define CORIOLIS_H_KED6CAHK

#include <array>

#include <RBDyn/CoM.h>
#include <RBDyn/MultiBodyConfig.h>

namespace rbd {

using Block = std::array<int, 3>;
using Blocks = std::vector<Block>;

/** Expand a symmetric product of a jacobian by its transpose onto every DoF.
 * @param jac Jacobian whose product will be expanded
 * @param mb Multibody system
 * @param jacMat The product of J^T*J that will be expanded
 */
Eigen::MatrixXd expand(const rbd::Jacobian& jac, const rbd::MultiBody& mb,
                       const Eigen::MatrixXd& jacMat);

/** Expand a symmetric product of a jacobian by its transpose onto every DoF
 * and accumulate the result
 * @param res Accumulator matrix
 * @see expand
 */
void expandAdd(const rbd::Jacobian& jac, const rbd::MultiBody& mb,
               const Eigen::MatrixXd& jacMat, Eigen::MatrixXd& res);

/** Compute a compact kinematic path, i.e. the sequence of consecutive blocks
 * of DoF contained in the original path.
 * @param jac Jacobian to be compacted
 * @param mb Multibody system
 */
Blocks compactPath(const rbd::Jacobian& jac, const rbd::MultiBody& mb);

/** Expand a symmetric product of a jacobian by its transpose onto every DoF
 * and accumulate the result using a compact representation of the DoF.
 * @param compacPath Blocks representing the compact kinematic path of the
 *Jacobian
 * @param jacMat The matrix containing the product J^T*J to be expanded
 * @param res The accumulator matrix
 **/
void compactExpandAdd(const Blocks& compactPath, const Eigen::MatrixXd& jacMat,
                      Eigen::MatrixXd& res);

/**
 * Computation of the Coriolis effects matrix on a multibody. The Coriolis
 * factorization is not unique, we use the formulation of M. Bjerkend and
 * K. Pettersen in "A new Coriolis matrix factorization", 2012
 * NB: ForwardDynamics::C() directly computes the product of this matrix with
 * qd.
 * This C*qd is unique, but C itself is not.
 */
class Coriolis {
 public:
  /** Initialize the required structures
   * @param mb Multibody system
   */
  Coriolis(const rbd::MultiBody& mb);

  /** Compute the matrix C of Coriolis effects.
   * @param mb Multibody system
   * @param mbc Multibody configuration associated to mb
   */
  Eigen::MatrixXd coriolis(const rbd::MultiBody& mb,
                           const rbd::MultiBodyConfig& mbc);

 private:
  std::vector<rbd::Jacobian> jacs_;
  std::vector<Eigen::MatrixXd> jacMats_;
  std::vector<Eigen::MatrixXd> jacDotMats_;
  std::vector<Blocks> compactPaths_;
};

}  // ns rbd

#endif /* end of include guard: CORIOLIS_H_KED6CAHK */
