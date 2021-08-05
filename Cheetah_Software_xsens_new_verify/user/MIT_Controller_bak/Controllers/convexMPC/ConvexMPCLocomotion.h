#ifndef CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H
#define CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H

#include <Controllers/FootSwingTrajectory.h>
#include <FSM_States/ControlFSMData.h>
#include <SparseCMPC/SparseCMPC.h>
#include "cppTypes.h"
#include "Gait.h"
#include "FootstepOpt.h"
#include <cstdio>
#include "qpOASES2Eigen_interface/QProblemEigen.h"

#include "saveFiles.h"

using Eigen::Array4f;
using Eigen::Array4i;
using Eigen::Dynamic;
using namespace std;

template <typename T>
struct CMPC_Result
{
  LegControllerCommand<T> commands[4];
  Vec4<T> contactPhase;
};

struct CMPC_Jump
{
  static constexpr int START_SEG = 6;
  static constexpr int END_SEG = 0;
  static constexpr int END_COUNT = 2;
  bool jump_pending = false;
  bool jump_in_progress = false;
  bool pressed = false;
  int seen_end_count = 0;
  int last_seg_seen = 0;
  int jump_wait_counter = 0;

  void debug(int seg)
  {
    (void)seg;
    //printf("[%d] pending %d running %d\n", seg, jump_pending, jump_in_progress);
  }

  void trigger_pressed(int seg, bool trigger)
  {
    (void)seg;
    if (!pressed && trigger)
    {
      if (!jump_pending && !jump_in_progress)
      {
        jump_pending = true;
        //printf("jump pending @ %d\n", seg);
      }
    }
    pressed = trigger;
  }

  bool should_jump(int seg)
  {
    debug(seg);

    if (jump_pending && seg == START_SEG)
    {
      jump_pending = false;
      jump_in_progress = true;
      //printf("jump begin @ %d\n", seg);
      seen_end_count = 0;
      last_seg_seen = seg;
      return true;
    }

    if (jump_in_progress)
    {
      if (seg == END_SEG && seg != last_seg_seen)
      {
        seen_end_count++;
        if (seen_end_count == END_COUNT)
        {
          seen_end_count = 0;
          jump_in_progress = false;
          //printf("jump end @ %d\n", seg);
          last_seg_seen = seg;
          return false;
        }
      }
      last_seg_seen = seg;
      return true;
    }

    last_seg_seen = seg;
    return false;
  }
};

class ConvexMPCLocomotion
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ConvexMPCLocomotion(float _dt, int _iterations_between_mpc, MIT_UserParameters *parameters);

  ~ConvexMPCLocomotion()
  {
    csvLog.close();
  }

  void initialize();

  template <typename T>
  void run(ControlFSMData<T> &data);
  bool currently_jumping = false;

  Vec3<float> pBody_des;
  Vec3<float> vBody_des;
  Vec3<float> aBody_des;

  Vec3<float> pBody_RPY_des;
  Vec3<float> vBody_Ori_des;

  Vec3<float> pFoot_des[4];
  Vec3<float> vFoot_des[4];
  Vec3<float> aFoot_des[4];

  Vec3<float> Fr_des[4];

  Vec4<float> contact_state;

private:
  void _SetupCommand(ControlFSMData<float> &data);

  float _yaw_turn_rate;
  float _yaw_des;

  float _roll_des;
  float _pitch_des;

  float _x_vel_des = 0.;
  float _y_vel_des = 0.;

  float x_vel_cmd = 0.;
  float y_vel_cmd = 0.;

  // High speed running
  //float _body_height = 0.34;
  // float _body_height = 0.29;
  float _body_height = 0.3;

  float _body_height_running = 0.29;
  float _body_height_jumping = 0.36;

  float side_sign[4] = {-1, 1, -1, 1};
  float interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
  float interleave_gain = -0.2;
  float v_abs = 0.f;

  void recompute_timing(int iterations_per_mpc);
  void updateMPCIfNeeded(int *mpcTable, ControlFSMData<float> &data, bool omniMode);
  void solveDenseMPC(int *mpcTable, ControlFSMData<float> &data);
  void solveSparseMPC(int *mpcTable, ControlFSMData<float> &data);
  void initSparseMPC();
  void computex_des_bounding(ControlFSMData<float> &data);
  void computex_des_trotting(ControlFSMData<float> &data);
  void changeSupportIfNeeded_bounding(ControlFSMData<float> &data);
  void changeSupportIfNeeded_trotting(ControlFSMData<float> &data);
  void proposedCoMAndFeetPlan(ControlFSMData<float> &data);

  void matrix_to_real_t(qpOASES::real_t *dst, DMat<float> src, s16 rows, s16 cols);
  void computeAiBi(int ith_horizon);
  void computeSxSu();
  void computeConstrsMat(int ith_horizon);
  void trajOpt(ControlFSMData<float> &data);
  void CoMinit(ControlFSMData<float> &data, float t);
  int iterationsBetweenMPC;
  int horizonLength;
  int default_iterations_between_mpc;
  float dt;
  float dtMPC;
  int iterationCounter = 0;
  Vec3<float> f_ff[4];
  Vec4<float> swingTimes;
  FootSwingTrajectory<float> footSwingTrajectories[4];
  OffsetDurationGait crawling1, crawling2, crawling3, crawling4, trotting, bounding, pronking, jumping, galloping, standing, trotRunning, walking, walking2, pacing;
  MixedFrequncyGait random, random2;
  Mat3<float> Kp, Kd, Kp_stance, Kd_stance;
  bool firstRun = true;
  bool firstSwing[4];
  float swingTimeRemaining[4];
  float stand_traj[6];
  int current_gait;
  int gaitNumber;

  Vec3<float> world_position_desired;
  Vec3<float> rpy_int;
  Vec3<float> rpy_comp;
  float x_comp_integral = 0;
  Vec3<float> pFoot[4];
  Vec2<float> pFoot_u[2];
  DVec<float> pos_foot_pred[4];
  Vec3<float> ad_foot; //zejun
  Vec3<float> md_foot;
  Vec3<float> poff;
  Vec2<float> p_com;
  int ze;
  CMPC_Result<float> result;
  float trajAll[12 * 36];

  MIT_UserParameters *_parameters = nullptr;
  CMPC_Jump jump_state;

  vectorAligned<Vec12<double>> _sparseTrajectory;

  SparseCMPC _sparseCMPC;

  //wenchun:
  FootstepOpt<float> footstepOpt[4];
  FootstepOpt<float> footstepopt[3];
  double objvalue[3];
  double objmin[4];
  Vec3<float> cur_foothold[4];
  Vec2<float> x0[4];
  Vec2<float> x_des[4];
  Vec2<float> p_com_des;
  s16 nStep[4];
  s16 nopt[4];
  Vec2<float> v_com1;
  Vec2<float> x_des1, x_des2, x_des3, x_des4;

  qpOASES::QProblem CoMOpt;
  qpOASES::real_t *sln;
  qpOASES::real_t *H_qp;
  qpOASES::real_t *g_qp;
  qpOASES::real_t *A_qp;
  qpOASES::real_t *Aub_qp;
  qpOASES::int_t nWSR;

  qpOASES::QProblem CoM_init;
  qpOASES::real_t *sln1;
  qpOASES::real_t *H_qp1;
  qpOASES::real_t *g_qp1;
  qpOASES::real_t *A_qp1;
  qpOASES::real_t *Aub_qp1;

  Vec2<float> x_qp;
  Vec4<float> stat;
  // Vec8<float> A_pls;
  // Vec8<float> b_pl;
  Vec254<float> TrotAx;
  Vec48<float> CrawlAx1;
  Vec21<float> CrawlAx2;
  Vec47<float> CrawlAx3;

  int horizon = 10;
  Eigen::Matrix<float, 3, 12> ones;
  Mat3<float> eye3;
  float timeInterval = 0.03;
  DMat<float> Aa, Ba, Sx, Su, f_opt_traj, H_traj, f_traj, Q_w, R_w, x_predict;
  DVec<float> x_next;
  double v_opt_traj;
  DMat<int> contact_table;
  Mat4<float> Q_i;
  Mat2<float> R_i;
  Vec4<float> x_init;
  float BIGNUM = 1.0e26;
  float robotMass = 9.0;
  Gait *gait;
  QProblemEigen *trajSolver;
  int nrest;
  Vec12<float> coeff;
  double optv;
  double optval[3];
  double opttime;
  int optstep;

  ofstream csvLog;
  bool first_run = false;
  int shirt_iter = 0;
};

#endif //CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H
