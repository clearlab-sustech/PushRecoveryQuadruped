#include "problem.h"
#include <eigen3/unsupported/Eigen/MatrixFunctions>

namespace convexmpc
{
void QPProbData::resize(s16 horizon)
{
  A_qp.resize(13 * horizon, Eigen::NoChange);

  B_qp.resize(13 * horizon, 12 * horizon);

  S.resize(13 * horizon, 13 * horizon);

  X_d.resize(13 * horizon, Eigen::NoChange);

  U_b.resize(20 * horizon, Eigen::NoChange);

  fmat.resize(20 * horizon, 12 * horizon);

  qH.resize(12 * horizon, 12 * horizon);

  qg.resize(12 * horizon, Eigen::NoChange);

  eye_12h.resize(12 * horizon, 12 * horizon);

  A_qp.setZero();
  B_qp.setZero();
  S.setZero();
  X_d.setZero();
  U_b.setZero();
  fmat.setZero();
  qH.setZero();
  eye_12h.setIdentity();

#ifdef K_DEBUG
  printf("RESIZED MATRICES FOR HORIZON: %d\n",horizon);
#endif
}

Prob::Prob()
{
  qpOASES_model_ = std::make_unique<qpOASESModel>();
}

void Prob::updateControlFSMData(const ControlFSMData<float>& control_FSM_data)
{
  control_FSM_data_ = &control_FSM_data;
}

void Prob::updateRefTraj(const float* ref_traj)
{
  ref_traj_ = ref_traj;
}

void Prob::updateMPCTable(const int* mpc_table)
{
  mpc_table_ = mpc_table;
}

void Prob::updateProbConfig(const ProbConfig& prob_config)
{
  prob_config_ = prob_config;
}

void Prob::updateWeights(const std::vector<Eigen::Matrix<fpt, Eigen::Dynamic, 1>>& weights)
{
  weights_ = weights;
}

void Prob::updateXDrag(double x_drag)
{
  x_drag_ = x_drag;
}

int Prob::solve(std::vector<double>& solution)
{
  if (updateModel())
  {
    CvxOptStatus opt_status = qpOASES_model_->optimize();
    if (opt_status != CVX_SOLVED)
    {
      std::cout << "MPC solver failed!" << std::endl;
      return 1;
    }
    else
    {
      solution = qpOASES_model_->getSolution();
      return 0;
    }
  }
  return 1;
}

std::shared_ptr<RobotState> Prob::calcRobotState()
{
  auto& state_estimate = control_FSM_data_->_stateEstimator->getResult();

  std::array<Eigen::Vector3f, 4> p_foot;
  for (unsigned int i = 0; i < 4; ++i)
  {
    p_foot[i] = state_estimate.position + state_estimate.rBody.transpose() * 
                (control_FSM_data_->_quadruped->getHipLocation(i) + 
                control_FSM_data_->_legController->datas[i].p);
  }

  float r[12];
  for (unsigned int i = 0; i < 12; ++i)
    r[i] = p_foot[i%4][i/4] - state_estimate.position[i/4];

  std::shared_ptr<RobotState> robot_state = std::make_shared<RobotState>();

  robot_state->set(state_estimate.position.data(), 
                    state_estimate.vWorld.data(), 
                    state_estimate.orientation.data(),
                    state_estimate.omegaWorld.data(),
                    r, 
                    state_estimate.rpy[2]);

  return robot_state;
}

void Prob::calcContinuousSSMats(Eigen::Matrix<fpt, 13, 13>& A_ct, Eigen::Matrix<fpt, 13, 12>& B_ct_r)
{
  std::shared_ptr<RobotState> robot_state = calcRobotState();

  Eigen::Matrix<fpt, 3, 3> I_world = robot_state->R_yaw * robot_state->I_body * robot_state->R_yaw.transpose();

  A_ct.setZero();
  A_ct(3, 9) = 1.f;
  A_ct(11, 9) = x_drag_;
  A_ct(4, 10) = 1.f;
  A_ct(5, 11) = 1.f;

  A_ct(11, 12) = 1.f;
  A_ct.block(0, 6, 3, 3) = robot_state->R_yaw.transpose();

  B_ct_r.setZero();
  Eigen::Matrix<fpt, 3, 3> I_inv = I_world.inverse();

  for (s16 b = 0; b < 4; ++b)
  {
    B_ct_r.block(6, b*3, 3, 3) = cross_mat(I_inv,robot_state->r_feet.col(b));
    B_ct_r.block(9, b*3, 3, 3) = Eigen::Matrix<fpt, 3, 3>::Identity() / robot_state->m;
  }
}

void QPProbData::continuous2QP(const Eigen::Matrix<fpt, 13, 13>& Ac, const Eigen::Matrix<fpt, 13, 12>& Bc, ProbConfig prob_config)
{
  Eigen::Matrix<fpt,25,25> ABc, expmm;
  Eigen::Matrix<fpt,13,13> Adt;
  Eigen::Matrix<fpt,13,12> Bdt;

  ABc.setZero();
  ABc.block(0, 0, 13, 13) = Ac;
  ABc.block(0, 13, 13, 12) = Bc;
  ABc = prob_config.dt * ABc;
  expmm = ABc.exp();
  Adt = expmm.block(0, 0, 13, 13);
  Bdt = expmm.block(0, 13, 13, 12);

#ifdef K_PRINT_EVERYTHING
  cout << "Adt: \n" << Adt << "\nBdt:\n" << Bdt << endl;
#endif

  if(prob_config.horizon > 20) { // wenchun: 19
    throw std::runtime_error("horizon is too long!");
  }

  Eigen::Matrix<fpt, 13, 13> powerMats[20];
  powerMats[0].setIdentity();
  for (int i = 1; i < prob_config.horizon + 1; ++i) {
    powerMats[i] = Adt * powerMats[i-1];
  }

  for (s16 r = 0; r < prob_config.horizon; ++r)
  {
    A_qp.block(13*r, 0, 13, 13) = powerMats[r+1];//Adt.pow(r+1);
    for (s16 c = 0; c < prob_config.horizon; ++c)
    {
      if(r >= c)
      {
        s16 a_num = r - c;
        B_qp.block(13*r, 12*c, 13, 12) = powerMats[a_num] /*Adt.pow(a_num)*/ * Bdt;
      }
    }
  }

#ifdef K_PRINT_EVERYTHING
  cout << "Aq:\n" << A_qp << "\nBQP:\n" << B_qp << endl;
#endif
}

QPProbData Prob::calcQPProbData()
{
  QPProbData qp_prob_data;
  qp_prob_data.resize(prob_config_.horizon);

  // A_qp & B_qp
  Eigen::Matrix<fpt, 13, 13> A_ct;
  Eigen::Matrix<fpt, 13, 12> B_ct_r;
  calcContinuousSSMats(A_ct, B_ct_r);
  qp_prob_data.continuous2QP(A_ct, B_ct_r, prob_config_);

  // S
  Eigen::Matrix<fpt, Eigen::Dynamic, 1> full_weight;
  full_weight.resize(13 * prob_config_.horizon, Eigen::NoChange);
  for (int i = 0; i < prob_config_.horizon; ++i)
  {
    full_weight.segment(13*i, 12) = weights_.at(i);
    full_weight(13*i + 12) = 0.f;
  }
  qp_prob_data.S = full_weight.asDiagonal();

  // X_d
  for (int i = 0; i < prob_config_.horizon; ++i)
    for (unsigned int j = 0; j < 12; ++j)
      qp_prob_data.X_d(13*i+j, 0) = ref_traj_[12*i+j];

  // U_b
  unsigned int k = 0;
  for (int i = 0; i < prob_config_.horizon; ++i)
  {
    for (unsigned int j = 0; j < 4; ++j)
    {
      qp_prob_data.U_b(5*k + 0) = kBigNumber;
      qp_prob_data.U_b(5*k + 1) = kBigNumber;
      qp_prob_data.U_b(5*k + 2) = kBigNumber;
      qp_prob_data.U_b(5*k + 3) = kBigNumber;
      qp_prob_data.U_b(5*k + 4) = mpc_table_[i*4 + j] * prob_config_.f_max;
      ++k;
    }
  }

  // fmat
  fpt mu = 1.f / prob_config_.mu;
  Eigen::Matrix<fpt,5,3> f_block;

  f_block << mu,   0, 1.f,
            -mu,   0, 1.f,
              0,  mu, 1.f,
              0, -mu, 1.f,
              0,   0, 1.f;

  for (int i = 0; i < prob_config_.horizon * 4; ++i)
  {
    qp_prob_data.fmat.block(i*5, i*3, 5, 3) = f_block;
  }

  // qH
  qp_prob_data.qH = 2 * (qp_prob_data.B_qp.transpose() 
                          * qp_prob_data.S * qp_prob_data.B_qp 
                          + prob_config_.alpha * qp_prob_data.eye_12h);

  // qg
  std::shared_ptr<RobotState> robot_state = calcRobotState();
  Eigen::Matrix<fpt,3,1> rpy;
  quat2rpy(robot_state->q, rpy);
  Eigen::Matrix<fpt,13,1> x_0;
  x_0 << rpy(2), rpy(1), rpy(0), 
          robot_state->p, robot_state->w, robot_state->v, -9.8f;
  qp_prob_data.qg = 2 * qp_prob_data.B_qp.transpose() * qp_prob_data.S 
                      * (qp_prob_data.A_qp * x_0 - qp_prob_data.X_d);
  
  return qp_prob_data;
}

std::vector<qpOASES::real_t> Prob::qp2qpOASESHelper(const Eigen::Matrix<fpt, Eigen::Dynamic, Eigen::Dynamic>& in)
{
  std::vector<qpOASES::real_t> out;
  s32 a = 0;
  for (s16 r = 0; r < in.rows(); ++r)
    for (s16 c = 0; c < in.cols(); ++c)
    {
      out.push_back(in(r, c));
      ++a;
    }

  return out;
}

qpOASESData Prob::qp2qpOASES(const QPProbData& qp_prob_data)
{
  qpOASESData qpOASES_data;

  qpOASES_data.H = qp2qpOASESHelper(qp_prob_data.qH);
  qpOASES_data.g = qp2qpOASESHelper(qp_prob_data.qg);
  qpOASES_data.A = qp2qpOASESHelper(qp_prob_data.fmat);
  qpOASES_data.ub = qp2qpOASESHelper(qp_prob_data.U_b);
  for (int i = 0; i < 20 * prob_config_.horizon; ++i)
    qpOASES_data.lb.push_back(0.0f);

  return qpOASES_data;
}

// qpOASESData Prob::calcqpOASESDataReduced(const qpOASESData& qpOASES_data)
// {
//   qpOASESData qpOASES_data_reduced;

//   return qpOASES_data_reduced;
// }

bool Prob::updateModel()
{
  bool model_updated = false;

  QPProbData qp_prob_data = calcQPProbData();
  qpOASESData qpOASES_data = qp2qpOASES(qp_prob_data);
  qpOASES_model_->updateData(qpOASES_data);
  qpOASES_model_->updateNumVars(12 * prob_config_.horizon);
  qpOASES_model_->updateNumCnts(20 * prob_config_.horizon);
  model_updated = true;

  return model_updated;
}

} // namespace convexmpc