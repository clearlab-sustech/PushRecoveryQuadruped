#ifndef CONVEXMPC_PROBLEM_H
#define CONVEXMPC_PROBLEM_H

#include <eigen3/Eigen/Dense>
#include <memory>
#include <qpOASES.hpp>
#include "FSM_States/ControlFSMData.h"
#include "common_types.h"
#include "RobotState.h"
#include "utils.h"
#include "qpoases_interface.h"

namespace convexmpc
{
const double kBigNumber = 5e10;

struct ProbConfig
{
  ProbConfig() {}
  ~ProbConfig() = default;
  ProbConfig(double dt_, int horizon_, double mu_, double f_max_, double alpha_)
    : dt(dt_), horizon(horizon_), mu(mu_), f_max(f_max_), alpha(alpha_)
  {
  }

  double dt;
  int horizon;
  double mu;
  double f_max;
  double alpha;

  void printProbConfig();
};

struct QPProbData
{
  Eigen::Matrix<fpt, Eigen::Dynamic, 13> A_qp;
  Eigen::Matrix<fpt, Eigen::Dynamic, Eigen::Dynamic> B_qp;
  Eigen::Matrix<fpt, Eigen::Dynamic, Eigen::Dynamic> S;
  Eigen::Matrix<fpt, Eigen::Dynamic, 1> X_d;
  Eigen::Matrix<fpt, Eigen::Dynamic, 1> U_b;
  Eigen::Matrix<fpt, Eigen::Dynamic, Eigen::Dynamic> fmat;
  Eigen::Matrix<fpt, Eigen::Dynamic, Eigen::Dynamic> qH;
  Eigen::Matrix<fpt, Eigen::Dynamic, 1> qg;
  Eigen::Matrix<fpt, Eigen::Dynamic, Eigen::Dynamic> eye_12h;

  void resize(s16 horizon);

  void continuous2QP(const Eigen::Matrix<fpt, 13, 13> &Ac, const Eigen::Matrix<fpt, 13, 12> &Bc, ProbConfig prob_config);
};

class Prob
{
public:
  Prob();

  ~Prob() = default;

  /** Link the ControlFSMData object to the Prob object */
  void updateControlFSMData(const ControlFSMData<float>& control_FSM_data);

  /** Update reference trajectory */
  void updateRefTraj(const float* ref_traj);

  /** 
   * Update MPC table, which is determined by the gait and 
   * specifies whether the ground reaction force should be zero (in air)
   */
  void updateMPCTable(const int* mpc_table);

  /** Update the problem configuration */
  void updateProbConfig(const ProbConfig &prob_config);

  /** Update the weights of the waypoints of a reference trajectory within a horizon */
  void updateWeights(const std::vector<Eigen::Matrix<fpt, Eigen::Dynamic, 1>>& weights);

  /** Update the variable x_drag. Don't know what it is yet */
  void updateXDrag(double x_drag);

  /** Formulate the QP problem and call the solver */
  int solve(std::vector<double>& solution);

private:
  const ControlFSMData<float>* control_FSM_data_;
  const float* ref_traj_;
  const int* mpc_table_;
  ProbConfig prob_config_;
  std::vector<Eigen::Matrix<fpt, Eigen::Dynamic, 1>> weights_;
  double x_drag_;
  std::unique_ptr<qpOASESModel> qpOASES_model_;

  std::shared_ptr<RobotState> calcRobotState();

  void calcContinuousSSMats(Eigen::Matrix<fpt, 13, 13> &A_ct, Eigen::Matrix<fpt, 13, 12> &B_ct_r);

  QPProbData calcQPProbData();

  std::vector<qpOASES::real_t> qp2qpOASESHelper(const Eigen::Matrix<fpt, Eigen::Dynamic, Eigen::Dynamic> &src);

  qpOASESData qp2qpOASES(const QPProbData &qp_prob_data);

  // qpOASESData calcqpOASESDataReduced(const qpOASESData &qpOASES_data);

  bool updateModel();
};

} // namespace convexmpc
#endif // CONVEXMPC_PROBLEM_H