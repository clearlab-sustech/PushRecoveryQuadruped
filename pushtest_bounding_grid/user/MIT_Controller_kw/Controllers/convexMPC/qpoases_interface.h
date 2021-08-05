#pragma once

#include <memory>
#include <vector>
#include <qpOASES.hpp>

namespace convexmpc
{
/**
 * qpOASESModel uses the LGPL solver qpOASES to solve a linearly constrained QP.
 * qpOASES solves a problem in the form:
 * ```
 * min   1/2*x'Hx + x'g
 * s.t.  lb  <=  x <= ub
 * lbA <= Ax <= ubA
 * ```
 *
 * More informations about the solver are available at:
 * https://projects.coin-or.org/qpOASES
 */

enum CvxOptStatus
{
  CVX_SOLVED,
  CVX_INFEASIBLE,
  CVX_FAILED
};

struct qpOASESData
{
  std::vector<qpOASES::real_t> H;
  std::vector<qpOASES::real_t> g;
  std::vector<qpOASES::real_t> A;
  std::vector<qpOASES::real_t> lb;
  std::vector<qpOASES::real_t> ub;
};

class qpOASESModel
{
  std::shared_ptr<qpOASES::QProblem> qpoases_problem_;  /**< pointer to a qpOASES Quadratic Problem */
  qpOASES::Options qpoases_options_;                    /**< qpOASES solver options */
  std::vector<qpOASES::real_t> H_;                      /**< Quadratic cost matrix */
  std::vector<qpOASES::real_t> g_;                      /**< Gradient of the optimization problem? */
  std::vector<qpOASES::real_t> A_;                      /**< Constraints matrix */
  std::vector<qpOASES::real_t> lb_;                     /**< Variables lower bounds */
  std::vector<qpOASES::real_t> ub_;                     /**< Variables upper bounds */
  
  int num_vars_;
  int num_cnts_;

  /**
   * Instantiates a new qpOASES problem if it has not been instantiated yet
   * or if the size of the problem has changed.
   *
   * @returns true if a new qpOASES problem has been instantiated
   */
  bool updateSolver();

  /**
   * Instantiates a new qpOASES problem
   */
  void createSolver();

  std::vector<double> solution_;  /**< optimizer's solution for current model */

public:
  qpOASESModel();
  ~qpOASESModel() = default;

  void updateData(const qpOASESData& qpOASES_data);

  void updateNumVars(int num_vars);

  void updateNumCnts(int num_cnts);

  CvxOptStatus optimize();

  std::vector<double> getSolution();
};

} // namespace convexmpc
