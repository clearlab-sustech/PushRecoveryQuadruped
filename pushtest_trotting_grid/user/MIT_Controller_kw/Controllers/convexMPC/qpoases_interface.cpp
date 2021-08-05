#include "qpoases_interface.h"

using namespace qpOASES;

namespace convexmpc
{
double QPOASES_INFTY = qpOASES::INFTY;

qpOASESModel::qpOASESModel()
{
  // set to be fast. More details at:
  // https://www.coin-or.org/qpOASES/doc/3.2/doxygen/classOptions.html
  // https://projects.coin-or.org/qpOASES/browser/stable/3.2/src/Options.cpp#L191
  qpoases_options_.setToMPC();
  qpoases_options_.printLevel = qpOASES::PL_NONE;
  // enable regularisation to deal with degenerate Hessians
  qpoases_options_.enableRegularisation = qpOASES::BT_TRUE;
  qpoases_options_.ensureConsistency();
}

bool qpOASESModel::updateSolver()
{
  bool solver_updated = false;

  if (!qpoases_problem_ || num_vars_ != qpoases_problem_->getNV() || num_cnts_ != qpoases_problem_->getNC())
  {
    // Create Problem - this should be called only once
    qpoases_problem_.reset(new QProblem(num_vars_, num_cnts_));
    qpoases_problem_->setOptions(qpoases_options_);
    solver_updated = true;
  }
  
  return solver_updated;
}

void qpOASESModel::createSolver()
{
  qpoases_problem_.reset();
  updateSolver();
}

void qpOASESModel::updateData(const qpOASESData& qpOASES_data)
{
  H_ = qpOASES_data.H;
  g_ = qpOASES_data.g;
  A_ = qpOASES_data.A;
  lb_ = qpOASES_data.lb;
  ub_ = qpOASES_data.ub;
}

void qpOASESModel::updateNumVars(int num_vars)
{
  num_vars_ = num_vars;
}

void qpOASESModel::updateNumCnts(int num_cnts)
{
  num_cnts_ = num_cnts;
}

CvxOptStatus qpOASESModel::optimize()
{
  updateSolver();
  qpOASES::returnValue val = qpOASES::RET_QP_SOLUTION_STARTED;

  // Solve Problem
  int nWSR = 255;
  if (qpoases_problem_->isInitialised())
  {
    val = qpoases_problem_->hotstart(
        g_.data(), nullptr, nullptr, lb_.data(), ub_.data(), nWSR);
  }

  if (val != qpOASES::SUCCESSFUL_RETURN)
  {
    // TODO ATM this means we are creating a new solver even if updateSolver
    //      returned true and the problem is not initialized. Still, it makes
    //      tests pass.
    createSolver();

    val = qpoases_problem_->init(H_.data(), g_.data(), A_.data(), nullptr, nullptr, lb_.data(), ub_.data(), nWSR);
  }

  if (val == qpOASES::SUCCESSFUL_RETURN)
  {
    // opt += m_objective.affexpr.constant;
    solution_.resize(num_vars_, 0.);
    val = qpoases_problem_->getPrimalSolution(solution_.data());
    return CVX_SOLVED;
  }
  else if (val == qpOASES::RET_INIT_FAILED_INFEASIBILITY)
  {
    return CVX_INFEASIBLE;
  }
  else
  {
    return CVX_FAILED;
  }
}

std::vector<double> qpOASESModel::getSolution()
{
  return solution_;
}

} // namespace sco
