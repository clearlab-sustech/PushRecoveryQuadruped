#include <iostream>
#include <Utilities/Timer.h>
#include <Utilities/Utilities_print.h>
#include <fstream>

#include "ConvexMPCLocomotion.h"
#include "convexMPC_interface.h"
#include "../../../../common/FootstepPlanner/GraphSearch.h"

#include "Gait.h"
#include <eigen3/unsupported/Eigen/MatrixFunctions>

//#define DRAW_DEBUG_SWINGS
//#define DRAW_DEBUG_PATH

////////////////////
// Controller
////////////////////

ConvexMPCLocomotion::ConvexMPCLocomotion(float _dt,
                                         int _iterations_between_mpc,
                                         MIT_UserParameters *parameters) : iterationsBetweenMPC(_iterations_between_mpc),
                                                                           horizonLength(10), //wenchun: 10
                                                                           dt(_dt),
                                                                           // if(seResult.vWorld[0]>0.2)
                                                                           // {horizonLength = 5;
                                                                           //   }
                                                                           crawling1(16, Vec4<int>(0, 8, 12, 4), Vec4<int>(12, 12, 12, 12), "Crawling1"),
                                                                           crawling2(16, Vec4<int>(12, 4, 8, 0), Vec4<int>(12, 12, 12, 12), "Crawling2"),
                                                                           crawling3(16, Vec4<int>(8, 0, 4, 12), Vec4<int>(12, 12, 12, 12), "Crawling3"),
                                                                           crawling4(16, Vec4<int>(4, 12, 0, 8), Vec4<int>(12, 12, 12, 12), "Crawling4"),
                                                                           trotting(horizonLength, Vec4<int>(0, 5, 5, 0), Vec4<int>(5, 5, 5, 5), "Trotting"),
                                                                           // trotting(horizonLength, Vec4<int>(0,4,4,0), Vec4<int>(4,4,4,4),"Trotting"),
                                                                           // trotting(horizonLength, Vec4<int>(0,8,12,4), Vec4<int>(12,12,12,12),"Trotting"), //wenchun
                                                                          //  bounding(horizonLength, Vec4<int>(0, 0, 5, 5), Vec4<int>(5, 5, 5, 5), "Bounding"),
                                                                           bounding(horizonLength, Vec4<int>(5,5,0,0),Vec4<int>(4,4,4,4),"Bounding"),
                                                                           // bounding(horizonLength, Vec4<int>(5,5,0,0),Vec4<int>(3,3,3,3),"Bounding"),
                                                                           pronking(horizonLength, Vec4<int>(0, 0, 0, 0), Vec4<int>(4, 4, 4, 4), "Pronking"),
                                                                           jumping(horizonLength, Vec4<int>(0, 0, 0, 0), Vec4<int>(2, 2, 2, 2), "Jumping"),
                                                                           //galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(6,6,6,6),"Galloping"),
                                                                           //galloping(horizonLength, Vec4<int>(0,2,7,Vec4<int>(0,5,5,0), Vec4<int>(5,5,5,5)9),Vec4<int>(3,3,3,3),"Galloping"),
                                                                           galloping(horizonLength, Vec4<int>(0, 2, 7, 9), Vec4<int>(4, 4, 4, 4), "Galloping"),
                                                                           standing(horizonLength, Vec4<int>(0, 0, 0, 0), Vec4<int>(16, 16, 16, 16), "Standing"),
                                                                           //trotRunning(horizonLength, Vec4<int>(0,5,5,0),Vec4<int>(3,3,3,3),"Trot Running"),
                                                                           trotRunning(horizonLength, Vec4<int>(0, 5, 5, 0), Vec4<int>(4, 4, 4, 4), "Trot Running"),
                                                                           walking(horizonLength, Vec4<int>(0, 3, 5, 8), Vec4<int>(5, 5, 5, 5), "Walking"),
                                                                           walking2(horizonLength, Vec4<int>(0, 5, 5, 0), Vec4<int>(7, 7, 7, 7), "Walking2"),
                                                                           // pacing(horizonLength, Vec4<int>(4,0,4,0),Vec4<int>(4,4,4,4),"Pacing"),
                                                                           pacing(horizonLength, Vec4<int>(5, 0, 5, 0), Vec4<int>(5, 5, 5, 5), "Pacing"),
                                                                           // pacing(horizonLength, Vec4<int>(5,0,5,0),Vec4<int>(4,4,4,4),"Pacing"),
                                                                           //waving(horizonLength*2, Vec4<int>(0,horizonLength,1.5*horizonLength,    0.5*horizonLength), Vec4<int>(1.5*horizonLength,1.5*horizonLength,1.5*horizonLength,1.5*horizonLength),"Waving"),
                                                                           random(horizonLength, Vec4<int>(9, 13, 13, 9), 0.4, "Flying nine thirteenths trot"),
                                                                           random2(horizonLength, Vec4<int>(8, 16, 16, 8), 0.5, "Double Trot"),
                                                                           CoMOpt(2, 8),
                                                                           // CoMOpt(2,14),
                                                                           CoM_init(12, 8)
{
  _parameters = parameters;
  dtMPC = dt * iterationsBetweenMPC;
  default_iterations_between_mpc = iterationsBetweenMPC;
  //printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, iterationsBetweenMPC, dtMPC);
  setup_problem(dtMPC, horizonLength, 0.4, 120); //0.4 is the friction coefficient
  //setup_problem(dtMPC, horizonLength, 0.4, 650); // DH
  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;

  for (int i = 0; i < 4; i++)
    firstSwing[i] = true;

  initSparseMPC();

  pBody_des.setZero();
  vBody_des.setZero();
  aBody_des.setZero();

  /* different with MIT --- start ---------------------*/
  trajSolver = new QProblemEigen(horizon * 2, 2 * horizon, HST_SEMIDEF);
  trajSolver->options.setToMPC();
  trajSolver->options.printLevel = PL_NONE;
  trajSolver->options.enableRegularisation = BT_TRUE;
  trajSolver->options.enableEqualities = BT_TRUE;
  trajSolver->options.epsIterRef = 0.0001;
  trajSolver->options.numRefinementSteps = 100;
  trajSolver->options.enableRamping = BT_TRUE;

  Aa.resize(4 * horizon, 4);
  Ba.resize(4 * horizon, 2);
  H_traj.resize(2 * horizon, 2 * horizon);
  f_traj.resize(2 * horizon, 1);
  x_next.resize(horizon * 4, 1);
  Aa.setZero();
  Ba.setZero();
  H_traj.setZero();
  f_traj.setZero();
  x_next.setZero();

  Q_i.setZero();
  Vec4<float> par1;
  par1 << 1000.0f, 1000.0f, 1000.0f, 1000.f;
  Q_i.diagonal() = par1;

  R_i.setZero();
  Vec2<float> par2;
  par2 << 1.f, 1.f;
  R_i.diagonal() = par2;

  Q_w.resize(4 * horizon, 4 * horizon);
  R_w.resize(2 * horizon, 2 * horizon);
  Q_w.setZero();
  R_w.setZero();

  x_predict.resize(4 * horizon, 1);
  x_predict.setZero();

  sln = new qpOASES::real_t[2];
  qpOASES::Options op;
  op.setToMPC();
  op.printLevel = qpOASES::PL_NONE;
  CoMOpt.setOptions(op);

  sln1 = new qpOASES::real_t[12];
  qpOASES::Options op1;
  op1.enableEqualities = BT_TRUE;
  op1.setToMPC();
  op1.printLevel = qpOASES::PL_NONE;
  CoM_init.setOptions(op1);

  csvLog.open("simdata.txt");
  /* different with MIT --- end ---------------------*/
}

/* different with MIT --- start ---------------------*/
bool safe = true;
bool changesupport = true;
bool transition = true;
void ConvexMPCLocomotion::computex_des(ControlFSMData<float> &data)
{
  auto seResult = data._stateEstimator->getResult();
  Vec2<float> vd(0.0f, 0.0f);
  Eigen::Matrix<float, 2, 2> H;
  H.setIdentity();
  Eigen::Matrix<float, 2, 1> g;
  g = -seResult.position.block(0, 0, 2, 1);

  Eigen::Matrix<float, 8, 2> C; //bound & pace gait cycle 10
  Vec8<float> C_1;
  C_1 << 0, 0, 0, -0.985539, 0, 0.985539, 0, 0;
  Vec8<float> C_3;
  C_3 << 0, -0.985539, 0, 0, 0.985539, 0, 0, 0;
  C.block(0, 0, 8, 1) = -1 * C_1;
  C.block(0, 1, 8, 1) = -1 * C_3;
  Vec8<float> C_2;
  C_2 << 0, 0, -1, -0.16944873, 0, 0.16944873, 0, 1;
  Vec8<float> C_4;
  C_4 << -1, -0.16944873, 0, 0, 0.16944873, 0, 1, 0;
  Vec8<float> b_pl;
  b_pl << 10, 0.10840929, 10, 0.18725241, 0.10840929, 0.18725241, 10, 10;
  Eigen::Matrix<float, 8, 4> A_pl;
  A_pl.block(0, 0, 8, 1) = C_1;
  A_pl.block(0, 1, 8, 1) = C_2;
  A_pl.block(0, 2, 8, 1) = C_3;
  A_pl.block(0, 3, 8, 1) = C_4;
  Eigen::Matrix<float, 4, 1> sta;
  sta << seResult.position[0], seResult.vWorld[0], seResult.position[1], seResult.vWorld[1];
  Eigen::Matrix<float, 8, 1> ub;
  ub = b_pl - A_pl * sta;

  // Eigen::Matrix<float, 8, 2> C; //trot gait cycle 10
  // Vec8<float> C_1;
  // C_1 <<0,0,0,-0.4938,0.4938,-0.4938,0,0.4938;
  // Vec8<float> C_3;
  // C_3 <<0,0,0,0.8529,0.8529,-0.8529,0,-0.8529;
  // C.block(0, 0, 8, 1) =  -1 * C_1;
  // C.block(0, 1, 8, 1) =  -1 * C_3;
  // Vec8<float> C_2;
  // C_2 <<0,0,-1,-0.0849,0.0849,-0.0849,1,0.0849;
  // Vec8<float> C_4;
  // C_4 <<1,-1,0,0.1466,0.1466,-0.1466,0,-0.1466;
  // Vec8<float> b_pl;
  // b_pl <<6,6,6,0.1323,0.0553,0.0553,6,0.1323;
  // Eigen::Matrix<float, 8, 4> A_pl;
  // A_pl.block(0, 0, 8, 1) = C_1;
  // A_pl.block(0, 1, 8, 1) = C_2;
  // A_pl.block(0, 2, 8, 1) = C_3;
  // A_pl.block(0, 3, 8, 1) = C_4;
  // Eigen::Matrix<float, 4, 1> sta;
  // sta <<seResult.position[0], seResult.vWorld[0], seResult.position[1], seResult.vWorld[1];
  // Eigen::Matrix<float, 8, 1> ub;
  // ub = b_pl - A_pl *  sta;

  // Eigen::Matrix<float, 14, 2> C; //trot gait cycle 8
  // Vec14<float> C_1;
  // C_1 <<0,0,0.49379,0,-0.49379,-0.49379,-0.97633,-0.045428,-0.17318,0.49379,0.97633,0,0.17318,0.045428;
  // Vec14<float> C_3;
  // C_3 <<0,0,-0.85291,0,0.85291,-0.85291,-0.13441,-0.98449,-0.9702,0.85291,0.13441,0,0.9702,0.98449;
  // C.block(0, 0, 14, 1) =  -1 * C_1;
  // C.block(0, 1, 14, 1) =  -1 * C_3;
  // Vec14<float> C_2;
  // C_2 <<0,0,0.084897,-1,-0.084897,-0.084902,-0.16787,-0.0078138,-0.02978,0.084902,0.16787,1,0.02978,0.0078138;
  // Vec14<float> C_4;
  // C_4 <<1,-1,-0.14664,0,0.14664,-0.14665,-0.023119,-0.16927,-0.16681,0.14665,0.023119,0,0.16681,0.16927;
  // Vec14<float> b_pl;
  // b_pl <<4,4,0.06213,4,0.06213,0.12523,0.19006,0.1109,0.11752,0.12523,0.19006,4,0.11752,0.1109;
  // Eigen::Matrix<float, 14, 4> A_pl;
  // A_pl.block(0, 0, 14, 1) = C_1;
  // A_pl.block(0, 1, 14, 1) = C_2;
  // A_pl.block(0, 2, 14, 1) = C_3;
  // A_pl.block(0, 3, 14, 1) = C_4;
  // Eigen::Matrix<float, 4, 1> sta;
  // sta <<seResult.position[0], seResult.vWorld[0], seResult.position[1], seResult.vWorld[1];
  // Eigen::Matrix<float, 14, 1> ub;
  // ub = b_pl - A_pl *  sta;

  // Eigen::Matrix<float, 12, 2> C; //trot gait cycle 10
  // Vec12<float> C_1;
  // C_1 <<0,0,-0.49379,0,-0.49379,-0.97633,0.49379,0,0,0.49379,0,0.9855;
  // Vec12<float> C_3;
  // C_3 <<0,0,-0.85291,0,0.85291,0,-0.85291,-0.9855,0,0.85291,0.9855,0;
  // C.block(0, 0, 12, 1) =  -1 * C_1;
  // C.block(0, 1, 12, 1) =  -1 * C_3;
  // Vec12<float> C_2;
  // C_2 <<0,0,-0.084897,-1,-0.084897,-0.1694,0.084897,0,1,0.084897,0,0.1694;
  // Vec12<float> C_4;
  // C_4 <<1,-1,-0.14664,0,0.14664,0,-0.14665,-0.1694,0,0.14665,0.1694,0;
  // Vec12<float> b_pl;
  // b_pl <<10,10,0.1323,10,0.0553,0.1873,0.0553,0.1084,10,0.1323,0.1084,0.1873;
  // Eigen::Matrix<float, 12, 4> A_pl;
  // A_pl.block(0, 0, 12, 1) = C_1;
  // A_pl.block(0, 1, 12, 1) = C_2;
  // A_pl.block(0, 2, 12, 1) = C_3;
  // A_pl.block(0, 3, 12, 1) = C_4;
  // Eigen::Matrix<float, 4, 1> sta;
  // sta <<seResult.position[0], seResult.vWorld[0], seResult.position[1], seResult.vWorld[1];
  // Eigen::Matrix<float, 12, 1> ub;
  // ub = b_pl - A_pl *  sta;

  // if (!changesupport && safe)
  // // if (iterationCounter % (iterationsBetweenMPC*4) == 0)
  // {
  H_qp = new qpOASES::real_t[H.cols() * H.rows()];
  g_qp = new qpOASES::real_t[g.cols() * g.rows()];
  A_qp = new qpOASES::real_t[C.cols() * C.rows()];
  Aub_qp = new qpOASES::real_t[ub.cols() * ub.rows()];

  matrix_to_real_t(H_qp, H, 2, 2);
  matrix_to_real_t(g_qp, g, 2, 1);
  matrix_to_real_t(A_qp, C, 8, 2);
  matrix_to_real_t(Aub_qp, ub, 8, 1);
  // matrix_to_real_t(A_qp,C,12,2);
  // matrix_to_real_t(Aub_qp,ub,12,1);

  nWSR = 1000;
  int nVars = 2;
  // int nCons = 14;

  // qpOASES::QProblem CoMOpt(nVars,nCons);
  // qpOASES:: real_t sln[nVars];

  CoMOpt.reset();
  int rval = CoMOpt.init(H_qp, g_qp, A_qp, NULL, NULL, NULL, Aub_qp, nWSR);
  if (rval != qpOASES::SUCCESSFUL_RETURN)
    printf("failed to initialize\n");

  int rval2 = CoMOpt.getPrimalSolution(sln);
  if (rval2 != qpOASES::SUCCESSFUL_RETURN)
    printf("failed to solve!\n");
  else
  {
    for (int i = 0; i < nVars; i++)
    {
      x_qp[i] = sln[i];
    }
  }
  // safe = false;
  // }
  // }

  //  printf("x_desx: %.3f, x_desy %.3f\n", x_qp[0], x_qp[1]);
  //  printf("com_vy: %.3f", seResult.position[2]);
  //  printf("com_vy: %.3f", seResult.vWorld[1]);
  // }
}
/* different with MIT --- end ---------------------*/

/* different with MIT --- start ---------------------*/
void ConvexMPCLocomotion::CoMinit(ControlFSMData<float> &data, float t)
{
  auto seResult = data._stateEstimator->getResult();
  Eigen::Matrix<float, 12, 12> H;
  H.setZero();
  Eigen::Matrix<float, 6, 6> H1;
  H1.setZero();
  Vec6<float> H_1;
  H_1 << 400 / 7 * pow(t, 7), 240 / 6 * pow(t, 6), 120 / 5 * pow(t, 5), 40 / 4 * pow(t, 4), 0, 0;
  Vec6<float> H_2;
  H_2 << 240 / 6 * pow(t, 6), 144 / 5 * pow(t, 5), 72 / 4 * pow(t, 4), 24 / 3 * pow(t, 3), 0, 0;
  Vec6<float> H_3;
  H_3 << 120 / 5 * pow(t, 5), 72 / 4 * pow(t, 4), 36 / 3 * pow(t, 3), 12 / 2 * pow(t, 2), 0, 0;
  Vec6<float> H_4;
  H_4 << 10 * pow(t, 4), 8 * pow(t, 3), 6 * pow(t, 2), 4 * t, 0, 0;
  H1.block(0, 0, 1, 6) = H_1.transpose();
  H1.block(1, 0, 1, 6) = H_2.transpose();
  H1.block(2, 0, 1, 6) = H_3.transpose();
  H1.block(3, 0, 1, 6) = H_4.transpose();
  H.block(0, 0, 6, 6) = H1;
  H.block(6, 6, 6, 6) = H1;
  Eigen::Matrix<float, 12, 1> g;
  g.setZero();
  Eigen::Matrix<float, 8, 12> Aeq;
  Aeq.setZero();
  Vec6<float> Aeq_1;
  Aeq_1 << 0, 0, 0, 0, 0, 1;
  Vec6<float> Aeq_2;
  Aeq_2 << pow(t, 5), pow(t, 4), pow(t, 3), pow(t, 2), t, 1;
  Vec6<float> Aeq_3;
  Aeq_3 << 0, 0, 0, 0, 1, 0;
  Vec6<float> Aeq_4;
  Aeq_4 << 5 * pow(t, 4), 4 * pow(t, 3), 3 * pow(t, 2), 2 * t, 1, 0;
  Aeq.block(0, 0, 1, 6) = Aeq_1.transpose();
  Aeq.block(1, 6, 1, 6) = Aeq_1.transpose();
  Aeq.block(2, 0, 1, 6) = Aeq_2.transpose();
  Aeq.block(3, 6, 1, 6) = Aeq_2.transpose();
  Aeq.block(4, 0, 1, 6) = Aeq_3.transpose();
  Aeq.block(5, 6, 1, 6) = Aeq_3.transpose();
  Aeq.block(6, 0, 1, 6) = Aeq_4.transpose();
  Aeq.block(7, 6, 1, 6) = Aeq_4.transpose();
  Vec8<float> beq;
  beq << seResult.position(0), seResult.position(1), x_qp(0), x_qp(1), seResult.vWorld(0), seResult.vWorld(1), 0, 0;

  H_qp1 = new qpOASES::real_t[H.cols() * H.rows()];
  g_qp1 = new qpOASES::real_t[g.cols() * g.rows()];
  A_qp1 = new qpOASES::real_t[Aeq.cols() * Aeq.rows()];
  Aub_qp1 = new qpOASES::real_t[beq.cols() * beq.rows()];

  matrix_to_real_t(H_qp1, H, 12, 12);
  matrix_to_real_t(g_qp1, g, 12, 1);
  matrix_to_real_t(A_qp1, Aeq, 8, 12);
  matrix_to_real_t(Aub_qp1, beq, 8, 1);

  nWSR = 1000;
  int nVars = 12;
  // int nCons = 8;

  // qpOASES::QProblem CoM_init(nVars,nCons);
  // qpOASES:: real_t sln[nVars];

  CoM_init.reset();
  int rval = CoM_init.init(H_qp1, g_qp1, A_qp1, NULL, NULL, Aub_qp1, Aub_qp1, nWSR);
  if (rval != qpOASES::SUCCESSFUL_RETURN)
  {
    printf("failed to initialize\n");
  }
  else
  {
    optv = CoM_init.getObjVal();
  }
  int rval2 = CoM_init.getPrimalSolution(sln1);
  if (rval2 != qpOASES::SUCCESSFUL_RETURN)
    printf("failed to solve!\n");

  for (int i = 0; i < nVars; i++)
  {
    coeff[i] = sln1[i];
  }
}
/* different with MIT --- end ---------------------*/

/* different with MIT --- start ---------------------*/
void ConvexMPCLocomotion::matrix_to_real_t(qpOASES::real_t *dst, DMat<float> src, s16 rows, s16 cols)
{
  s32 a = 0;
  for (s16 r = 0; r < rows; r++)
  {
    for (s16 c = 0; c < cols; c++)
    {
      dst[a] = src(r, c);
      a++;
    }
  }
}
/* different with MIT --- end ---------------------*/

/* different with MIT --- start ---------------------*/
void ConvexMPCLocomotion::computeAiBi(int ith_horizon)
{
  Mat4<float> A_i;
  A_i.setZero();
  float par = 9.81 / _body_height;
  A_i(0, 1) = 1;
  A_i(1, 0) = par; // TODO: verify the sign of g
  A_i(2, 3) = 1;
  A_i(3, 2) = par;

  Eigen::Matrix<float, 4, 2> B_0;
  B_0.setZero();
  B_0(1, 0) = -par;
  B_0(3, 1) = -par;
  Eigen::Matrix<float, 4, 2> B_i;
  B_i.setZero();
  // B_i.block(0,0,4,1) = B_0 * (pFoot_u[0] - pFoot_u[1]);
  // B_i.block(0,1,4,1) = B_0 * pFoot_u[1];
  B_i = B_0;

  // TODO: set the dicrete time interval
  Eigen::Matrix<float, 6, 6> par_m, s;
  par_m.setZero();
  par_m.block(0, 0, 4, 4) = A_i;
  par_m.block(0, 4, 4, 2) = B_i;
  par_m = timeInterval * par_m;
  s = par_m.exp();

  Aa.block(ith_horizon * 4, 0, 4, 4) = s.block(0, 0, 4, 4);
  Ba.block(ith_horizon * 4, 0, 4, 2) = s.block(0, 4, 4, 2);
}
/* different with MIT --- end ---------------------*/

void ConvexMPCLocomotion::computeSxSu()
{
  Sx.resize(horizon * 4, 4);
  Su.resize(horizon * 4, horizon * 2);
  Sx.setZero();
  Su.setZero();
  Sx.block(0, 0, 4, 4) = Aa.block(0, 0, 4, 4);
  Su.block(0, 0, 4, 2) = Ba.block(0, 0, 4, 2);
  for (int i = 1; i < horizon; i++)
  {
    Sx.block(i * 4, 0, 4, 4) = Aa.block(i * 4, 0, 4, 4) * Sx.block((i - 1) * 4, 0, 4, 4);
    Su.block(i * 4, i * 2, 4, 2) = Ba.block(i * 4, 0, 4, 2);
    for (int j = 0; j < i; j++)
    {
      Su.block(i * 4, j * 2, 4, 2) = Aa.block(i * 4, 0, 4, 4) * Su.block((i - 1) * 4, j * 2, 4, 2);
    }
  }
}

/* different with MIT --- start ---------------------*/
void ConvexMPCLocomotion::trajOpt(ControlFSMData<float> &data)
{
  auto seResult = data._stateEstimator->getResult();

  /*--------------traj opt------------------*/

  delete trajSolver;
  trajSolver = new QProblemEigen(horizon * 2, 2 * horizon, HST_SEMIDEF);
  trajSolver->options.setToMPC();
  trajSolver->options.printLevel = PL_NONE;
  trajSolver->options.enableRegularisation = BT_TRUE;
  trajSolver->options.enableEqualities = BT_TRUE;
  trajSolver->options.epsIterRef = 0.0001;
  trajSolver->options.numRefinementSteps = 100;
  trajSolver->options.enableRamping = BT_TRUE;

  Aa.resize(4 * horizon, 4);
  Ba.resize(4 * horizon, 2);
  H_traj.resize(2 * horizon, 2 * horizon);
  f_traj.resize(2 * horizon, 1);
  Aa.setZero();
  Ba.setZero();
  H_traj.setZero();
  f_traj.setZero();

  Q_w.resize(4 * horizon, 4 * horizon);
  R_w.resize(2 * horizon, 2 * horizon);
  Q_w.setZero();
  R_w.setZero();

  x_predict.resize(4 * horizon, 1);
  x_predict.setZero();
  x_init.block(0, 0, 2, 1) = Vec2<float>(seResult.position(0), seResult.vWorld(0));
  x_init.block(2, 0, 2, 1) = Vec2<float>(seResult.position(1), seResult.vWorld(1));

  DMat<float> Ca, du, dl;
  Ca.resize(horizon * 2, horizon * 2);
  du.resize(horizon * 2, 1);
  dl.resize(horizon * 2, 1);

  Ca.setZero();
  du.setZero();
  dl.setZero();

  int *mpc_table = gait->getMpcTable();
  DVec<int> stance_table;
  stance_table.resize(horizonLength * 4, 1);
  stance_table.setZero();
  for (int i = 0; i < horizonLength * 4; i++)
  {
    stance_table(i) = mpc_table[i];
  }
  contact_table.resize(horizonLength * 4 * 10, 1);
  contact_table.setZero();
  for (int i = 0; i < 10; i++)
  {
    contact_table.block(horizonLength * 4 * i, 0, horizonLength * 4, 1) = stance_table;
  }

  nrest = (iterationsBetweenMPC * horizonLength / 2 - (iterationCounter % (iterationsBetweenMPC * horizonLength / 2))) / iterationsBetweenMPC;
  // nrest = ceil(horizonLength / 2 - (iterationCounter % (iterationsBetweenMPC * horizonLength / 2))/iterationsBetweenMPC);
  if (nrest == 5)
  {
    nrest = 4;
  }

  for (int ith_horizon = 0; ith_horizon < horizon; ith_horizon++)
  {
    // Vec2<float> v_des_world = seResult.vWorld.block(0,0,2,1);
    Vec2<float> v_des_world = Vec2<float>(0.f, 0.f);
    // Vec2<float> com_pos = seResult.position.block(0,0,2,1) + v_des_world * dtMPC * (float)(ith_horizon + 1);
    Vec2<float> com_pos;
    com_pos(0) = x_qp(0);
    com_pos(1) = x_qp(1);
    // x_predict.block(ith_horizon * 4, 0, 2, 1) = com_pos;
    // x_predict.block(ith_horizon * 4 + 2, 0, 2, 1) = v_des_world;
    x_predict(ith_horizon * 4, 0) = com_pos[0];
    x_predict(ith_horizon * 4 + 1, 0) = v_des_world[0];
    x_predict(ith_horizon * 4 + 2, 0) = com_pos[1];
    x_predict(ith_horizon * 4 + 3, 0) = v_des_world[1];

    int j = 0;
    for (int i = 0; i < 4; i++)
    {
      if (contact_table(ith_horizon * 4 + i) == 1)
      {
        if (ith_horizon <= nrest)
        {
          pFoot_u[j] = pos_foot_pred[i].block(0, 0, 2, 1);
        }
        else
        {
          pFoot_u[j] = pos_foot_pred[i].block((floor((ith_horizon - nrest - 1) / (horizonLength / 2)) + 1) * 2, 0, 2, 1);
        }
        j++;
      }
    }
    float a = pFoot_u[1][1] - pFoot_u[0][1];
    float b = pFoot_u[0][0] - pFoot_u[1][0];
    float c = pFoot_u[1][0] * pFoot_u[0][1] - pFoot_u[0][0] * pFoot_u[1][1];
    float uv, lv;
    Vec2<float> Ci;
    Ci = Vec2<float>(1.f, 0.f);
    uv = fmaxf(pFoot_u[1][0], pFoot_u[0][0]);
    lv = fminf(pFoot_u[1][0], pFoot_u[0][0]);

    if (b <= 0.0001 && b >= -0.0001)
    {
      b = 0;
      Ci = Vec2<float>(0.f, 1.f);
      uv = fmaxf(pFoot_u[1][1], pFoot_u[0][1]);
      lv = fminf(pFoot_u[1][1], pFoot_u[0][1]);
    }

    if (a <= 0.0001 && a >= -0.0001)
    {
      a = 0;
      Ci = Vec2<float>(1.f, 0.f);
      uv = fmaxf(pFoot_u[1][0], pFoot_u[0][0]);
      lv = fminf(pFoot_u[1][0], pFoot_u[0][0]);
    }

    Mat2<float> C_i;
    C_i.block(0, 0, 1, 2) = Vec2<float>(a, b).transpose();
    C_i.block(1, 0, 1, 2) = Ci.transpose();
    Vec2<float> d_u;
    d_u = Vec2<float>(-c, uv);
    Vec2<float> d_l;
    d_l = Vec2<float>(-c, lv);

    Ca.block(ith_horizon * 2, ith_horizon * 2, 2, 2) = C_i;
    du.block(ith_horizon * 2, 0, 2, 1) = d_u;
    dl.block(ith_horizon * 2, 0, 2, 1) = d_l;

    computeAiBi(ith_horizon);
    if (ith_horizon < horizon - 1)
    {
      Q_w.block(ith_horizon * 4, ith_horizon * 4, 4, 4) = Q_i;
    }
    else
    {
      Q_w.block(ith_horizon * 4, ith_horizon * 4, 4, 4) = 10000 * Q_i;
    }
    R_w.block(ith_horizon * 2, ith_horizon * 2, 2, 2) = R_i;
  }
  computeSxSu();
  // std::cout << "Q_w: \n" << Q_w << std::endl;
  // std::cout << "R_w: \n" << R_w << std::endl;

  // Compute H and f
  H_traj = Su.transpose() * Q_w * Su + R_w;
  f_traj = Su.transpose() * Q_w * Sx * x_init - Su.transpose() * Q_w * x_predict;
  // std::cout << "H_traj: \n" << H_traj << std::endl;

  trajSolver->nWSR = 1000;
  trajSolver->setup(H_traj, f_traj, Ca, dl, du);

  trajSolver->getSolution(f_opt_traj);
  trajSolver->getValue(v_opt_traj);

  x_next.resize(horizon * 4, 1);
  x_next = Sx * x_init + Su * f_opt_traj;
}
/* different with MIT --- end ---------------------*/

void ConvexMPCLocomotion::initialize()
{
  for (int i = 0; i < 4; i++)
    firstSwing[i] = true;
  firstRun = true;
}

void ConvexMPCLocomotion::recompute_timing(int iterations_per_mpc)
{
  iterationsBetweenMPC = iterations_per_mpc;
  dtMPC = dt * iterations_per_mpc;
}

void ConvexMPCLocomotion::_SetupCommand(ControlFSMData<float> &data)
{
  if (data._quadruped->_robotType == RobotType::MINI_CHEETAH)
  {
    _body_height = 0.29;
  }
  else if (data._quadruped->_robotType == RobotType::CHEETAH_3)
  {
    _body_height = 0.45;
  }
  else
  {
    assert(false);
  }

  /* different with MIT --- start ---------------------*/
  // float x_vel_cmd, y_vel_cmd;
  float filter(0.1);
  if (data.controlParameters->use_rc)
  {
    const rc_control_settings *rc_cmd = data._desiredStateCommand->rcCommand;
    data.userParameters->cmpc_gait = rc_cmd->variable[0];
    _yaw_turn_rate = -rc_cmd->omega_des[2];
    x_vel_cmd = rc_cmd->v_des[0];
    y_vel_cmd = rc_cmd->v_des[1] * 0.5;
    _body_height += rc_cmd->height_variation * 0.08;
  }
  else
  {
    _yaw_turn_rate = data._desiredStateCommand->rightAnalogStick[0];
    // x_vel_cmd = data._desiredStateCommand->leftAnalogStick[1];
    if (changesupport)
    {
      x_vel_cmd = 0.; //wenchun
      // y_vel_cmd = data._desiredStateCommand->leftAnalogStick[0];
      y_vel_cmd = 0.;
    }
    else
    {
      x_vel_cmd = 0.;
      y_vel_cmd = 0.;
    }
  }
  _x_vel_des = _x_vel_des * (1 - filter) + x_vel_cmd * filter;
  _y_vel_des = _y_vel_des * (1 - filter) + y_vel_cmd * filter;

  // _yaw_des = data._stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;
  _yaw_des = 0.;
  _roll_des = 0.;
  _pitch_des = 0.;
  /* different with MIT --- end ---------------------*/
}

bool foot1c = true;
bool foot2c = true;
void ConvexMPCLocomotion::changeSupportIfNeeded(ControlFSMData<float> &data)
{
  auto &seResult = data._stateEstimator->getResult();

  // Vec8<float> C_1;
  // C_1 <<-0.985539,0,0,0,0,0.985539,0,0;
  // Vec8<float> C_3;
  // C_3 <<0,0,0,-0.985539,0.985539,0,0,0;
  // Vec8<float> C_2;
  // C_2 <<-0.16944873,-1,0,0,0,0.16944873,0,1;
  // Vec8<float> C_4;
  // C_4 <<0,0,-1,-0.16944873,0.16944873,0,1,0;
  // b_pl <<0.067743521,10,10,0.10840929,0.10840929,-0.057887955,10,10; //BRS of bounding

  Vec8<float> C_1;
  C_1 << 0, 0, 0, -0.985539, 0, 0.985539, 0, 0;
  Vec8<float> C_3;
  C_3 << 0, -0.985539, 0, 0, 0.985539, 0, 0, 0;
  Vec8<float> C_2;
  C_2 << 0, 0, -1, -0.16944873, 0, 0.16944873, 0, 1;
  Vec8<float> C_4;
  C_4 << -1, -0.16944873, 0, 0, 0.16944873, 0, 1, 0;
  Vec8<float> b_pl;
  b_pl << 10, 0.10840929, 10, 0.18725241, 0.10840929, 0.18725241, 10, 10; //BRS of 4leg stance
  Eigen::Matrix<float, 8, 4> A_pl;
  A_pl.block(0, 0, 8, 1) = C_1;
  A_pl.block(0, 1, 8, 1) = C_2;
  A_pl.block(0, 2, 8, 1) = C_3;
  A_pl.block(0, 3, 8, 1) = C_4;

  // stat[0] = seResult.position[0];
  // stat[0] = seResult.position[0] - md_foot[0];
  stat[0] = 0;
  stat[1] = seResult.vWorld[0];
  // stat[2] = seResult.position[1];
  // stat[2] = seResult.position[1] - md_foot[1];
  stat[2] = 0;
  stat[3] = seResult.vWorld[1];
  Vec8<float> A_pls;
  A_pls = A_pl * stat;
  v_com1 = Vec2<float>(seResult.vWorld(0), seResult.vWorld(1));

  for (int r = 0; r < 8; r++)
  {
    //  if (A_pls[r] > b_pl[r] && v_com1.norm()>=0.5)
    if (A_pls[r] > b_pl[r])
    {
      changesupport = false;
      //  foot1c = true;
      //  gaitNumber = 8;
      //  horizonLength = 16;
    }
  }

  // Vec8<float> C_1;   //trot gait cycle 10
  // C_1 <<0,0,0,-0.4938,0.4938,-0.4938,0,0.4938;
  // Vec8<float> C_3;
  // C_3 <<0,0,0,0.8529,0.8529,-0.8529,0,-0.8529;
  // Vec8<float> C_2;
  // C_2 <<0,0,-1,-0.0849,0.0849,-0.0849,1,0.0849;
  // Vec8<float> C_4;
  // C_4 <<1,-1,0,0.1466,0.1466,-0.1466,0,-0.1466;
  // Vec8<float> b_pl;
  // b_pl <<6,6,6,0.1323,0.0553,0.0553,6,0.1323;
  // Eigen::Matrix<float, 8, 4> A_pl;
  // A_pl.block(0, 0, 8, 1) = C_1;
  // A_pl.block(0, 1, 8, 1) = C_2;
  // A_pl.block(0, 2, 8, 1) = C_3;
  // A_pl.block(0, 3, 8, 1) = C_4;

  // // stat[0] = seResult.position[0];
  // // stat[0] = seResult.position[0] - md_foot[0];
  // stat[0] = 0;
  // stat[1] = seResult.vWorld[0];
  // // stat[2] = seResult.position[1];
  // // stat[2] = seResult.position[1] - md_foot[1];
  // stat[2] = 0;
  // stat[3] = seResult.vWorld[1];
  // Vec8<float> A_pls;
  // A_pls = A_pl * stat;
  // v_com1 = Vec2<float>(seResult.vWorld(0), seResult.vWorld(1));

  // for (int r = 0; r < 8; r++)
  //   {
  //    if (A_pls[r] > b_pl[r])
  //    {
  //      changesupport = false;
  //    }
  //   }

  // Vec14<float> C_1; //trot gaitcycle 8
  // C_1 <<0,0,0.49379,0,-0.49379,-0.49379,-0.97633,-0.045428,-0.17318,0.49379,0.97633,0,0.17318,0.045428;
  // Vec14<float> C_3;
  // C_3 <<0,0,-0.85291,0,0.85291,-0.85291,-0.13441,-0.98449,-0.9702,0.85291,0.13441,0,0.9702,0.98449;
  // Vec14<float> C_2;
  // C_2 <<0,0,0.084897,-1,-0.084897,-0.084902,-0.16787,-0.0078138,-0.02978,0.084902,0.16787,1,0.02978,0.0078138;
  // Vec14<float> C_4;
  // C_4 <<1,-1,-0.14664,0,0.14664,-0.14665,-0.023119,-0.16927,-0.16681,0.14665,0.023119,0,0.16681,0.16927;
  // Vec14<float> b_pl;
  // b_pl <<4,4,0.06213,4,0.06213,0.12523,0.19006,0.1109,0.11752,0.12523,0.19006,4,0.11752,0.1109;
  // Eigen::Matrix<float, 14, 4> A_pl;
  // A_pl.block(0, 0, 14, 1) = C_1;
  // A_pl.block(0, 1, 14, 1) = C_2;
  // A_pl.block(0, 2, 14, 1) = C_3;
  // A_pl.block(0, 3, 14, 1) = C_4;
  // // stat[0] = seResult.position[0];
  // // stat[0] = seResult.position[0] - md_foot[0];
  // stat[0] = 0;
  // stat[1] = seResult.vWorld[0];
  // // stat[2] = seResult.position[1];
  // // stat[2] = seResult.position[1] - md_foot[1];
  // stat[2] = 0;
  // stat[3] = seResult.vWorld[1];
  // Vec14<float> A_pls;
  // A_pls = A_pl * stat;

  // for (int r = 0; r < 14; r++)
  //   {
  //    if (A_pls[r] > b_pl[r])
  //    {
  //      changesupport = false;
  //      foot1c = true;
  //     //  gaitNumber = 5;
  //     //  horizonLength = 16;
  //    }
  //   }

  // Eigen::Matrix<float, 12, 2> C; //trot gait cycle 10
  // Vec12<float> C_1;
  // C_1 <<0,0,-0.49379,0,-0.49379,-0.97633,0.49379,0,0,0.49379,0,0.9855;
  // Vec12<float> C_3;
  // C_3 <<0,0,-0.85291,0,0.85291,0,-0.85291,-0.9855,0,0.85291,0.9855,0;
  // Vec12<float> C_2;
  // C_2 <<0,0,-0.084897,-1,-0.084897,-0.1694,0.084897,0,1,0.084897,0,0.1694;
  // Vec12<float> C_4;
  // C_4 <<1,-1,-0.14664,0,0.14664,0,-0.14665,-0.1694,0,0.14665,0.1694,0;
  // Vec12<float> b_pl;
  // b_pl <<10,10,0.1323,10,0.0553,0.1873,0.0553,0.1084,10,0.1323,0.1084,0.1873;
  // Eigen::Matrix<float, 12, 4> A_pl;
  // A_pl.block(0, 0, 12, 1) = C_1;
  // A_pl.block(0, 1, 12, 1) = C_2;
  // A_pl.block(0, 2, 12, 1) = C_3;
  // A_pl.block(0, 3, 12, 1) = C_4;
  // stat[0] = 0;
  // stat[1] = seResult.vWorld[0];
  // stat[2] = 0;
  // stat[3] = seResult.vWorld[1];
  // Vec12<float> A_pls;
  // A_pls = A_pl * stat;

  // for (int r = 0; r < 12; r++)
  //   {
  //    if (A_pls[r] > b_pl[r])
  //    {
  //      changesupport = false;
  //    }
  //   }
}

void ConvexMPCLocomotion::proposedCoMAndFeetPlan(ControlFSMData<float> &data)
{
  auto &seResult = data._stateEstimator->getResult();

  // if (!changesupport && (iterationCounter%5)==0)
  if (!changesupport)
  {
    for (int i = 0; i < 3; i++)
    {
      // float t = (i + 1) * 0.15;
      float t = (i + 1) * 0.6;
      // float t = 0.24;
      CoMinit(data, t);
      optval[i] = optv * t;
      // optval[i] = optv;
      if (optval[i] < opttime || i == 0)
      {
        opttime = optval[i];
        optstep = i + 1;
      }
    }
    // printf("optstep: %.3i", optstep);
    s16 nstep = optstep;
    // if (iterationCounter==0){
    //   nstep = 2;
    // }

    int iteration = 0;
    int itemax = 5;
    float verror = 10.0f;
    DMat<float> optvalue;
    optvalue.resize(itemax, 1);
    // for(int iteration = 0; iteration < 3; iteration++)
    Eigen::Matrix<float, Eigen::Dynamic, 1> Uopt;
    while (iteration < itemax && fabs(verror) > 2)
    {
      for (int i = 0; i < 4; i++)
      {
        Vec3<float> offset(0, side_sign[i] * .065, 0);

        Vec3<float> pRobotFrame = (data._quadruped->getHipLocation(i) + offset);

        pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
        float stance_time = gait->getCurrentStanceTime(dtMPC, i);
        Vec3<float> pYawCorrected =
            coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate * stance_time / 2) * pRobotFrame;
        Vec3<float> des_vel;
        des_vel[0] = _x_vel_des;
        des_vel[1] = _y_vel_des;
        des_vel[2] = 0.0;
        poff = seResult.rBody.transpose() * (pYawCorrected + des_vel * swingTimeRemaining[i]);
        Vec2<float> poffset(poff(0), poff(1));
        // Vec2<float> poffset(pRobotFrame(0), pRobotFrame(1));
        x_des[i] = x_qp + poffset;
        x0[i] = cur_foothold[i].block(0, 0, 2, 1);
        p_com = Vec2<float>(seResult.position(0), seResult.position(1));
        Vec3<float> rpy;
        rpy << -seResult.rpy(0), seResult.rpy(1), seResult.rpy(2);
        Mat3<float> R;
        // R = seResult.rBody;
        R.setIdentity();
        Vec3<float> hipLength(0, side_sign[i] * .065, 0);
        // s16 nstep = ceil((x_qp - p_com).norm()/(0.12*v_com1.norm()));
        // s16 nstep = 2;
        // if (nstep > 1)
        // {
        // for(int ii = 0; ii < 3; ii++)
        for (int ii = 1; ii < 2; ii++)
        {
          footstepopt[ii].solve_qp(x0[i], x_des[i], nstep + ii - 1, seResult.position, data._quadruped->getHipLocation(i) + hipLength, v_com1, R, x_next, iteration, nrest);
          objvalue[ii] = footstepopt[ii].objval;
          if (objvalue[ii] < objmin[i] || ii == 1)
          {
            objmin[i] = objvalue[ii];
            Uopt = footstepopt[ii].getResult();
            nopt[i] = nstep + ii - 1;
          }
        }
        // }else{
        //   for(int ii = 0; ii < 3; ii++)
        //   {
        //   footstepopt[ii].solve_qp(x0[i],x_des[i],nstep+ii,seResult.position,data._quadruped->getHipLocation(i) + hipLength, v_com1, R, x_next, iteration, nrest);
        //   objvalue[ii] = footstepopt[ii].objval;
        //   if (objvalue[ii] < objmin[i] || ii == nstep)
        //   {
        //     objmin[i] = objvalue[ii];
        //     Uopt = footstepopt[ii].getResult();
        //     nopt[i] = nstep + ii;
        //   }
        //   }
        // }
        Vec3<float> pos_foot;
        pos_foot.block(0, 0, 2, 1) = x0[i] + Uopt.block(0, 0, 2, 1);
        pos_foot[2] = -0.003;
        footSwingTrajectories[i].setFinalPosition(pos_foot);
        pos_foot_pred[i].resize(2 * (nopt[i] + 1), 1);
        pos_foot_pred[i].setZero();
        pos_foot_pred[i].block(0, 0, 2, 1) = x0[i];
        for (int iii = 1; iii <= nopt[i]; iii++)
        {
          pos_foot_pred[i].block(2 * iii, 0, 2, 1) = pos_foot_pred[i].block(2 * (iii - 1), 0, 2, 1) + Uopt.block(2 * (iii - 1), 0, 2, 1);
        }
      }
      horizon = (nopt[0] + nopt[1] + nopt[2] + nopt[3]) / 4 * horizonLength / 2;
      trajOpt(data);
      v_opt_traj = v_opt_traj / 10000000;
      // printf("value: %.3f", v_opt_traj);
      // printf("value: %.3f", objmin);
      float stepvalue = 0;
      for (int i = 0; i < 4; i++)
      {
        if (contact_table((nrest + 1) * 4 + i) == 1)
        {
          stepvalue += objmin[i] * horizonLength / 2;
        }
      }
      // printf("svalue: %.3f", stepvalue);
      optvalue(iteration) = v_opt_traj + stepvalue;
      if (iteration > 0)
      {
        verror = optvalue(iteration) - optvalue(iteration - 1);
      }
      iteration++;
    }

    // save Uopt, x_next, horizon
    /* DVec<float> Uopt_vec_last, x_next_vec_last, horizon_vec_last;
    Uopt_vec_last = Uopt_vec;
    Uopt_vec.resize(Uopt_vec_last.size() + Uopt.size());
    Uopt_vec.head(Uopt_vec_last.size()) = Uopt_vec_last;
    Uopt_vec.tail(Uopt.size()) = Uopt;
    x_next_vec_last = x_next_vec;
    x_next_vec.resize(x_next_vec_last.size() + x_next.size());
    x_next_vec.head(x_next_vec_last.size()) = x_next_vec_last;
    x_next_vec.tail(x_next.size()) = x_next;
    horizon_vec_last = horizon_vec;
    horizon_vec.resize(horizon_vec_last.size() + 1);
    horizon_vec.head(horizon_vec_last.size()) = horizon_vec_last;
    horizon_vec(horizon_vec.size() - 1) = horizon;
    saveVec(Uopt_vec, "sqp_Uopt.mat", "Uopt_vec");
    saveVec(x_next_vec, "sqp_x_next.mat", "x_next_vec");
    saveVec(horizon_vec, "sqp_horizon.mat", "horizon_vec"); */
  }
}

template <>
void ConvexMPCLocomotion::run(ControlFSMData<float> &data)
{
  bool omniMode = false;

  // Command Setup
  _SetupCommand(data);
  // gaitNumber = data.userParameters->cmpc_gait; //set gait in GUI

  if (gaitNumber >= 10)
  {
    gaitNumber -= 10;
    omniMode = true;
  }

  auto &seResult = data._stateEstimator->getResult();
  // printf("rpy: %.5f %.5f %.5f\n", seResult.rpy[0], seResult.rpy[1], seResult.rpy[2]);

  // Check if transition to standing
  if (((gaitNumber == 4) && current_gait != 4) || firstRun)
  {
    stand_traj[0] = seResult.position[0];
    stand_traj[1] = seResult.position[1];
    stand_traj[2] = 0.21;
    stand_traj[3] = 0;
    stand_traj[4] = 0;
    stand_traj[5] = seResult.rpy[2];
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];

    // //wenchun:
    // for (int i=0;i<4;++i)
    // {
    //   printf("[%d]: %7.3f %7.3f %7.3f\n", i, pFoot[i][0], pFoot[i][1], pFoot[i][2]);
    // }
  }

  /* different with MIT --- start ---------------------*/
  /* ad_foot.setZero();
  ze = 0;
  for (int i = 0; i < 4; i++)
  {
    if (pFoot[i][2] <= 0)
    {
      ad_foot = ad_foot + pFoot[i];
      ze = ze + 1;
      // printf("contact_foot: %.3i", i);
    }
  }
  md_foot = ad_foot / ze; */

  // changeSupportIfNeeded(data);

  if (!changesupport)
  {
    computex_des(data);
  }

  /* capture[0] = seResult.position[0] + sqrt(0.29 / 9.81) * seResult.vWorld[0];
  capture[1] = seResult.position[1] + sqrt(0.29 / 9.81) * seResult.vWorld[1];
  float ct_error = pow(x_qp[0] - capture[0], 2) + pow(x_qp[1] - capture[1], 2); */
  /* different with MIT --- end ---------------------*/

  // // pick gait
  // Gait* gait = &standing;
  // // if(!changesupport && v_com1.norm() >= 0.1)
  // if(v_com1.norm() >= 0.02)
  // gait = &bounding; //zejun

  // Gait* gait = &trotting;
  gait = &bounding;
  if (gaitNumber == 1)
    gait = &bounding;
  // gait = &waving;
  else if (gaitNumber == 2)
    gait = &pronking;
  else if (gaitNumber == 3)
    gait = &random;
  else if (gaitNumber == 4)
    gait = &standing;
  // else if(gaitNumber == 5)
  //   gait = &trotRunning;
  // else if(gaitNumber == 6)
  //   gait = &random2;
  // else if(gaitNumber == 7)
  //   gait = &random2;
  else if (gaitNumber == 8)
    gait = &pacing;
  // else if(gaitNumber == 10)
  //   gait = &waving;
  else if (gaitNumber == 5)
    gait = &crawling1;
  else if (gaitNumber == 6)
    gait = &crawling2;
  else if (gaitNumber == 7)
    gait = &crawling3;
  // else if(gaitNumber == 8)
  //   gait = &crawling4;
  else if (gaitNumber == 9)
    gait = &trotting;
  current_gait = gaitNumber;

  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  jumping.setIterations(iterationsBetweenMPC, iterationCounter);

  // jumping.setIterations(27/2, iterationCounter);
  jumping.setIterations(30 / 2, iterationCounter); //wenchun

  //printf("[%d] [%d]\n", jumping.get_current_gait_phase(), gait->get_current_gait_phase());
  // check jump trigger
  jump_state.trigger_pressed(jump_state.should_jump(jumping.getCurrentGaitPhase()),
                             data._desiredStateCommand->trigger_pressed);

  // bool too_high = seResult.position[2] > 0.29;
  // check jump action
  if (jump_state.should_jump(jumping.getCurrentGaitPhase()))
  {
    gait = &jumping;
    recompute_timing(27 / 2);
    _body_height = _body_height_jumping;
    currently_jumping = true;
  }
  else
  {
    recompute_timing(default_iterations_between_mpc);
    currently_jumping = false;
  }

  if (_body_height < 0.02)
  {
    _body_height = 0.29;
  }

  // integrate position setpoint
  Vec3<float> v_robot = seResult.vWorld;
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
  // v_des_world[0] = (x_qp[0] - seResult.position[0]) / (dtMPC*8);
  // v_des_world[1] = (x_qp[1] - seResult.position[1]) / (dtMPC*8);

  //pretty_print(v_des_world, std::cout, "v des world");

  //Integral-esque pitche and roll compensation
  if (fabs(v_robot[0]) > .2) //avoid dividing by zero
  {
    rpy_int[1] += dt * (_pitch_des - seResult.rpy[1]) / v_robot[0];
  }
  if (fabs(v_robot[1]) > 0.1)
  {
    rpy_int[0] += dt * (_roll_des - seResult.rpy[0]) / v_robot[1];
  }

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber != 8); //turn off for pronking

  for (int i = 0; i < 4; i++)
  {
    pFoot[i] = seResult.position +
               seResult.rBody.transpose() * (data._quadruped->getHipLocation(i) +
                                             data._legController->datas[i].p);
  }

  if (gait != &standing)
  {
    // world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
    // world_position_desired += dt * Vec3<float>(seResult.vWorld[0], seResult.vWorld[1], 0);
    /* different with MIT --- start ---------------------*/
    world_position_desired = Vec3<float>(x_qp[0], x_qp[1], 0);
    /* different with MIT --- end ---------------------*/
  }

  // some first time initialization
  if (firstRun)
  {
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.rpy[2];

    for (int i = 0; i < 4; i++)
    {

      footSwingTrajectories[i].setHeight(0.05);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      /* different with MIT --- start ---------------------*/
      // footSwingTrajectories[i].setFinalPosition(pFoot[i]);
      cur_foothold[i] = pFoot[i]; //wenchun
      /* different with MIT --- end ---------------------*/
    }
    firstRun = false;
  }

  // foot placement
  for (int l = 0; l < 4; l++)
    swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);

  // v_abs = std::fabs(seResult.vBody[0]);
  v_abs = std::fabs(v_des_robot[0]);
  for (int i = 0; i < 4; i++)
  {

    if (firstSwing[i])
    {
      swingTimeRemaining[i] = swingTimes[i];
    }
    else
    {
      swingTimeRemaining[i] -= dt;
    }
    //if(firstSwing[i]) {
    //footSwingTrajectories[i].setHeight(.05);
    // footSwingTrajectories[i].setHeight(.07);
    footSwingTrajectories[i].setHeight(.12); //wenchun

    Vec3<float> offset(0, side_sign[i] * .065, 0);

    Vec3<float> pRobotFrame = (data._quadruped->getHipLocation(i) + offset);

    pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
    float stance_time = gait->getCurrentStanceTime(dtMPC, i);
    Vec3<float> pYawCorrected =
        coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate * stance_time / 2) * pRobotFrame;

    Vec3<float> des_vel;
    des_vel[0] = _x_vel_des;
    des_vel[1] = _y_vel_des;
    des_vel[2] = 0.0;

    Vec3<float> Pf = seResult.position + seResult.rBody.transpose() * (pYawCorrected + des_vel * swingTimeRemaining[i]);

    //+ seResult.vWorld * swingTimeRemaining[i];

    //float p_rel_max = 0.35f;
    float p_rel_max = 0.3f;

    if (changesupport) //zejun
    {
      // if ((iterationCounter % 2) == 0)
      // {
      // Using the estimated velocity is correct
      //Vec3<float> des_vel_world = seResult.rBody.transpose() * des_vel;
      float pfx_rel = seResult.vWorld[0] * (.5 + _parameters->cmpc_bonus_swing) * stance_time +
                      .03f * (seResult.vWorld[0] - v_des_world[0]) +
                      (0.5f * seResult.position[2] / 9.81f) * (seResult.vWorld[1] * _yaw_turn_rate);
      float pfy_rel = seResult.vWorld[1] * .5 * stance_time * dtMPC +
                      .03f * (seResult.vWorld[1] - v_des_world[1]) +
                      (0.5f * seResult.position[2] / 9.81f) * (-seResult.vWorld[0] * _yaw_turn_rate);
      pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
      pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
      Pf[0] += pfx_rel;
      Pf[1] += pfy_rel;
      // Pf[0] +=  0;
      // Pf[1] +=  0;
      Pf[2] = -0.003;
      //Pf[2] = 0.0;
      footSwingTrajectories[i].setFinalPosition(Pf);
    }
  }

  /* different with MIT --- start ---------------------*/
  Timer SQPTimer;
  proposedCoMAndFeetPlan(data);
  // printf("SOLVE TIME: %.3f\n", SQPTimer.getMs());
  float solvetime = SQPTimer.getMs();

  // if (v_com1.norm()<=0.1)
  // {
  //   for(int i = 0; i < 4; i++)
  //   {
  //     Vec3<float> offset(0, side_sign[i] * .065, 0);
  //     Vec3<float> Pf = seResult.position + seResult.rBody.transpose() * (data._quadruped->getHipLocation(i) + offset);
  //     Pf[2] = 0;
  //     footSwingTrajectories[i].setFinalPosition(Pf);
  //     float ferror = (Pf - pFoot[i]).norm();
  //     serror =+ ferror;
  //   }
  //   if (serror <= 0.01 && fabs(seResult.rpy[0])<=0.02 && seResult.rpy.norm()<=0.1)
  //   {
  //     gaitNumber = 4;
  //   }
  // }

  /* different with MIT --- end ---------------------*/

  // calc gait
  iterationCounter++;

  // load LCM leg swing gains
  Kp << 700, 0, 0,
      0, 700, 0,
      0, 0, 150;
  Kp_stance = 0 * Kp;

  Kd << 7, 0, 0,
      0, 7, 0,
      0, 0, 7;
  Kd_stance = Kd;

  // gait
  Vec4<float> contactStates = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();
  int *mpcTable = gait->getMpcTable();
  updateMPCIfNeeded(mpcTable, data, omniMode);

  //  StateEstimator* se = hw_i->state_estimator;
  Vec4<float> se_contactState(0, 0, 0, 0);

#ifdef DRAW_DEBUG_PATH
  auto *trajectoryDebug = data.visualizationData->addPath();
  if (trajectoryDebug)
  {
    trajectoryDebug->num_points = 10;
    trajectoryDebug->color = {0.2, 0.2, 0.7, 0.5};
    for (int i = 0; i < 10; i++)
    {
      trajectoryDebug->position[i][0] = trajAll[12 * i + 3];
      trajectoryDebug->position[i][1] = trajAll[12 * i + 4];
      trajectoryDebug->position[i][2] = trajAll[12 * i + 5];
      auto *ball = data.visualizationData->addSphere();
      ball->radius = 0.01;
      ball->position = trajectoryDebug->position[i];
      ball->color = {1.0, 0.2, 0.2, 0.5};
    }
  }
#endif
  // Eigen::Matrix<float,6,1> Uopt0,Uopt1;
  // Uopt0.setZero();
  // Uopt1.setZero();
  for (int foot = 0; foot < 4; foot++)
  {
    float contactState = contactStates[foot];
    float swingState = swingStates[foot];
    if (swingState > 0) // foot is in swing
    // if(pFoot[foot][2] > 0)
    {
      if (firstSwing[foot])
      {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
        cur_foothold[foot] = pFoot[foot]; //wenchun
      }

#ifdef DRAW_DEBUG_SWINGS
      auto *debugPath = data.visualizationData->addPath();
      if (debugPath)
      {
        debugPath->num_points = 100;
        debugPath->color = {0.2, 1, 0.2, 0.5};
        float step = (1.f - swingState) / 100.f;
        for (int i = 0; i < 100; i++)
        {
          footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState + i * step, swingTimes[foot]);
          debugPath->position[i] = footSwingTrajectories[foot].getPosition();
        }
      }
      auto *finalSphere = data.visualizationData->addSphere();
      if (finalSphere)
      {
        finalSphere->position = footSwingTrajectories[foot].getPosition();
        finalSphere->radius = 0.02;
        finalSphere->color = {0.6, 0.6, 0.2, 0.7};
      }
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
      auto *actualSphere = data.visualizationData->addSphere();
      auto *goalSphere = data.visualizationData->addSphere();
      goalSphere->position = footSwingTrajectories[foot].getPosition();
      actualSphere->position = pFoot[foot];
      goalSphere->radius = 0.02;
      actualSphere->radius = 0.02;
      goalSphere->color = {0.2, 1, 0.2, 0.7};
      actualSphere->color = {0.8, 0.2, 0.2, 0.7};
#endif
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

      //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
      //                                          hw_i->leg_controller->leg_datas[foot].qd, 0); // velocity dependent friction compensation todo removed
      //hw_i->leg_controller->leg_datas[foot].qd, fsm->main_control_settings.variable[2]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> hipWorld = seResult.rBody.transpose() * data._quadruped->getHipLocation(foot) + seResult.position;
      if (pDesFootWorld[2] > hipWorld[2] - 0.05)
      {
        // std::cout << "pf_des: " << pDesFootWorld.transpose() << std::endl
        //           << "hip: " << hipWorld.transpose() << std::endl;
        pDesFootWorld[2] = hipWorld[2] - 0.05;
        vDesFootWorld[2] = 0;
      }
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

      // Update for WBC
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();

      if (!data.userParameters->use_wbc)
      {
        // Update leg control command regardless of the usage of WBIC
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp;
        data._legController->commands[foot].kdCartesian = Kd;
      }
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;

#ifdef DRAW_DEBUG_SWINGS
      auto *actualSphere = data.visualizationData->addSphere();
      actualSphere->position = pFoot[foot];
      actualSphere->radius = 0.02;
      actualSphere->color = {0.2, 0.2, 0.8, 0.7};
#endif

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      //cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";

      if (!data.userParameters->use_wbc)
      {
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;

        data._legController->commands[foot].forceFeedForward = f_ff[foot];
        data._legController->commands[foot].kdJoint = Mat3<float>::Identity() * 0.2;

        //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
        //                                          hw_i->leg_controller->leg_datas[foot].qd, 0); todo removed
        // hw_i->leg_controller->leg_commands[foot].tau_ff += 0*footSwingController[foot]->getTauFF();
      }
      else
      { // Stance foot damping
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = 0. * Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;
      }
      //            cout << "Foot " << foot << " force: " << f_ff[foot].transpose() << "\n";
      se_contactState[foot] = contactState;

      // Update for WBC
      //Fr_des[foot] = -f_ff[foot];
    }
  }

  // se->set_contact_state(se_contactState); todo removed
  data._stateEstimator->setContactPhase(se_contactState);

  // Update For WBC
  pBody_des[0] = world_position_desired[0];
  pBody_des[1] = world_position_desired[1];
  pBody_des[2] = _body_height;

  vBody_des[0] = v_des_world[0];
  vBody_des[1] = v_des_world[1];
  vBody_des[2] = 0.;

  aBody_des.setZero();

  pBody_RPY_des[0] = 0.;
  pBody_RPY_des[1] = 0.;
  pBody_RPY_des[2] = _yaw_des;

  vBody_Ori_des[0] = 0.;
  vBody_Ori_des[1] = 0.;
  vBody_Ori_des[2] = _yaw_turn_rate;

  //contact_state = gait->getContactState();
  contact_state = gait->getContactState();
  // changesupport = true;  //zejun
  // END of WBC Update

  csvoutN(csvLog, seResult.vWorld, 3, firstRun);
  csvoutN(csvLog, seResult.vBody, 3, firstRun);
  csvoutN(csvLog, seResult.position, 3, firstRun);
  // csvoutN(csvLog,legforce[0],3,firstRun);
  // csvoutN(csvLog,legforce[1],3,firstRun);
  // csvoutN(csvLog,legforce[2],3,firstRun);
  // csvoutN(csvLog,legforce[3],3,firstRun);
  // csvoutN(csvLog,legtorque[0],3,firstRun);
  // csvoutN(csvLog,legtorque[1],3,firstRun);
  // csvoutN(csvLog,legtorque[2],3,firstRun);
  // csvoutN(csvLog,legtorque[3],3,firstRun);
  csvoutN(csvLog, seResult.rpy, 3, firstRun);
  csvoutN(csvLog, x_qp, 2, firstRun);
  csvout(csvLog, solvetime, firstRun);
  csvoutN(csvLog, capture, 2, firstRun);
  // csvout(csvLog, ct_error, firstRun);
  csvLog << "\n"; // must include for a new data line
}

template <>
void ConvexMPCLocomotion::run(ControlFSMData<double> &data)
{
  (void)data;
  //printf("call to old CMPC with double!\n");
}

void ConvexMPCLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData<float> &data, bool omniMode)
{
  //iterationsBetweenMPC = 30;
  if ((iterationCounter % iterationsBetweenMPC) == 0)
  {
    auto seResult = data._stateEstimator->getResult();
    // float* p = seResult.position.data();

    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
    Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
    //float trajInitial[12] = {0,0,0, 0,0,.25, 0,0,0,0,0,0};

    ////printf("Position error: %.3f, integral %.3f\n", pxy_err[0], x_comp_integral);

    if (current_gait == 4)
    {
      float trajInitial[12] = {
          _roll_des,
          _pitch_des /*-hw_i->state_estimator->se_ground_pitch*/,
          (float)stand_traj[5] /*+(float)stateCommand->data.stateDes[11]*/,
          (float)stand_traj[0] /*+(float)fsm->main_control_settings.p_des[0]*/,
          (float)stand_traj[1] /*+(float)fsm->main_control_settings.p_des[1]*/,
          (float)_body_height /*fsm->main_control_settings.p_des[2]*/,
          0, 0, 0, 0, 0, 0};

      for (int i = 0; i < horizonLength; i++)
        for (int j = 0; j < 12; j++)
          trajAll[12 * i + j] = trajInitial[j];
    }

    else
    {
      /* different with MIT --- start ---------------------*/
      if (changesupport)
      {
        // const float max_pos_error = .1;
        float xStart = world_position_desired[0];
        float yStart = world_position_desired[1];

        // if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
        // if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

        // if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
        // if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

        // world_position_desired[0] = xStart;
        // world_position_desired[1] = yStart;

        float trajInitial[12] = {(float)rpy_comp[0], // 0
                                 (float)rpy_comp[1], // 1
                                 _yaw_des,           // 2
                                 //yawStart,    // 2
                                 xStart, // 3
                                 // x_qp[0],
                                 yStart,
                                 // x_qp[1],                                   // 4
                                 (float)_body_height, // 5
                                 0,                   // 6
                                 0,                   // 7
                                 _yaw_turn_rate,      // 8
                                 v_des_world[0],      // 9
                                 v_des_world[1],      // 10
                                 0};                  // 11

        for (int i = 0; i < horizonLength; i++)
        {
          for (int j = 0; j < 12; j++)
            trajAll[12 * i + j] = trajInitial[j];

          if (i == 0) // start at current position  TODO consider not doing this
          {
            //trajAll[3] = hw_i->state_estimator->se_pBody[0];
            //trajAll[4] = hw_i->state_estimator->se_pBody[1];
            trajAll[2] = seResult.rpy[2];
          }
          else
          {
            trajAll[12 * i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
            trajAll[12 * i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
            trajAll[12 * i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
          }
        }
      }
      else
      {
        float xStart = x_next(0, 0);
        float yStart = x_next(2, 0);
        float trajInitial[12] = {(float)rpy_comp[0], // 0
                                 (float)rpy_comp[1], // 1
                                 _yaw_des,           // 2
                                 //yawStart,    // 2
                                 xStart, // 3
                                 // x_qp[0],
                                 yStart,
                                 // x_qp[1],                                   // 4
                                 (float)_body_height, // 5
                                 0,                   // 6
                                 0,                   // 7
                                 _yaw_turn_rate,      // 8
                                 // x_next(1,0),                           // 9
                                 // x_next(3,0),
                                 0,
                                 0,  // 10
                                 0}; // 11

        for (int i = 0; i < horizonLength; i++)
        {
          for (int j = 0; j < 12; j++)
            trajAll[12 * i + j] = trajInitial[j];

          if (i == 0) // start at current position  TODO consider not doing this
          {
            //trajAll[3] = hw_i->state_estimator->se_pBody[0];
            //trajAll[4] = hw_i->state_estimator->se_pBody[1];
            trajAll[2] = seResult.rpy[2];
          }
          else
          {
            if (i < horizon)
            {
              trajAll[12 * i + 3] = x_next(4 * i, 0);
              trajAll[12 * i + 4] = x_next(4 * i + 2, 0);
            }
            else
            {
              trajAll[12 * i + 3] = x_next(4 * (horizon - 1), 0);
              trajAll[12 * i + 4] = x_next(4 * (horizon - 1) + 2, 0);
            }
            // trajAll[12*i + 3] = x_next(4*i,0);
            // trajAll[12*i + 4] = x_next(4*i + 2,0);
            // trajAll[12*i + 9] = x_next(4*i + 1,0);
            // trajAll[12*i + 10] = x_next(4*i + 3,0);
            trajAll[12 * i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
          }
        }
      }
    }
    /* different with MIT --- end ---------------------*/
    Timer solveTimer;

    if (_parameters->cmpc_use_sparse > 0.5)
    {
      solveSparseMPC(mpcTable, data);
    }
    else
    {
      solveDenseMPC(mpcTable, data);
    }
    //printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.getMs());
    // solvetime = solveTimer.getMs();
  }
}

void ConvexMPCLocomotion::solveDenseMPC(int *mpcTable, ControlFSMData<float> &data)
{
  auto seResult = data._stateEstimator->getResult();

  /* different with MIT --- start ---------------------*/
  // float Q[12] = {0.25, 0.25, 10, 0.04, 0.04, 0.04, 0, 0, 0.3, 4, 4, 2};
  float Q[12] = {0.25, 0.25, 10, 0.2, 0.2, 50, 0, 0, 0.3, 0.8, 0.8, 0.2}; //good one
  // float Q[12] = {0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.3, 0.3, 0.2}; //trotting Ts0.12
  // float Q[12] = {0.25, 0.25, 10, 6, 6, 20, 0, 0, 0.3, 2, 2, 0.1};
  // float Q[12] = {0.25, 0.25, 10, 0.002, 0.002, 20, 0, 0, 0.3, 0.4, 0.4, 0.1};//zejun
  // float Q[12] = {0.25, 0.25, 10, 2, 2, 40, 0, 0, 0.3, 0.2, 0.2, 0.2};
  /* different with MIT --- end ---------------------*/

  float yaw = seResult.rpy[2];
  float *weights = Q;
  float alpha = 4e-5; // make setting eventually
  //float alpha = 4e-7; // make setting eventually: DH
  float *p = seResult.position.data();
  float *v = seResult.vWorld.data();
  float *w = seResult.omegaWorld.data();
  float *q = seResult.orientation.data();

  float r[12];
  for (int i = 0; i < 12; i++)
    r[i] = pFoot[i % 4][i / 4] - seResult.position[i / 4];

  ////printf("current posistion: %3.f %.3f %.3f\n", p[0], p[1], p[2]);

  if (alpha > 1e-4)
  {
    std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    alpha = 1e-5;
  }

  Vec3<float> pxy_act(p[0], p[1], 0);
  Vec3<float> pxy_des(world_position_desired[0], world_position_desired[1], 0);
  //Vec3<float> pxy_err = pxy_act - pxy_des;
  float pz_err = p[2] - _body_height;
  Vec3<float> vxy(seResult.vWorld[0], seResult.vWorld[1], 0);

  Timer t1;
  dtMPC = dt * iterationsBetweenMPC;
  setup_problem(dtMPC, horizonLength, 0.4, 120);
  //setup_problem(dtMPC,horizonLength,0.4,650); //DH
  update_x_drag(x_comp_integral);
  if (vxy[0] > 0.3 || vxy[0] < -0.3)
  {
    //x_comp_integral += _parameters->cmpc_x_drag * pxy_err[0] * dtMPC / vxy[0];
    x_comp_integral += _parameters->cmpc_x_drag * pz_err * dtMPC / vxy[0];
  }

  ////printf("pz err: %.3f, pz int: %.3f\n", pz_err, x_comp_integral);

  update_solver_settings(_parameters->jcqp_max_iter, _parameters->jcqp_rho,
                         _parameters->jcqp_sigma, _parameters->jcqp_alpha, _parameters->jcqp_terminate, _parameters->use_jcqp);
  //t1.stopPrint("Setup MPC");

  Timer t2;
  //cout << "dtMPC: " << dtMPC << "\n";
  update_problem_data_floats(p, v, q, w, r, yaw, weights, trajAll, alpha, mpcTable);
  //t2.stopPrint("Run MPC");
  ////printf("MPC Solve time %f ms\n", t2.getMs());

  for (int leg = 0; leg < 4; leg++)
  {
    Vec3<float> f;
    for (int axis = 0; axis < 3; axis++)
      f[axis] = get_solution(leg * 3 + axis);

    // //printf("[%d] %7.3f %7.3f %7.3f\n", leg, f[0], f[1], f[2]);

    f_ff[leg] = -seResult.rBody * f;
    // Update for WBC
    Fr_des[leg] = f;
  }
}

void ConvexMPCLocomotion::solveSparseMPC(int *mpcTable, ControlFSMData<float> &data)
{
  // X0, contact trajectory, state trajectory, feet, get result!
  (void)mpcTable;
  (void)data;
  auto seResult = data._stateEstimator->getResult();

  std::vector<ContactState> contactStates;
  for (int i = 0; i < horizonLength; i++)
  {
    contactStates.emplace_back(mpcTable[i * 4 + 0], mpcTable[i * 4 + 1], mpcTable[i * 4 + 2], mpcTable[i * 4 + 3]);
  }

  for (int i = 0; i < horizonLength; i++)
  {
    for (u32 j = 0; j < 12; j++)
    {
      _sparseTrajectory[i][j] = trajAll[i * 12 + j];
    }
  }

  Vec12<float> feet;
  for (u32 foot = 0; foot < 4; foot++)
  {
    for (u32 axis = 0; axis < 3; axis++)
    {
      feet[foot * 3 + axis] = pFoot[foot][axis] - seResult.position[axis];
    }
  }

  _sparseCMPC.setX0(seResult.position, seResult.vWorld, seResult.orientation, seResult.omegaWorld);
  _sparseCMPC.setContactTrajectory(contactStates.data(), contactStates.size());
  _sparseCMPC.setStateTrajectory(_sparseTrajectory);
  _sparseCMPC.setFeet(feet);
  _sparseCMPC.run();

  Vec12<float> resultForce = _sparseCMPC.getResult();

  for (u32 foot = 0; foot < 4; foot++)
  {
    Vec3<float> force(resultForce[foot * 3], resultForce[foot * 3 + 1], resultForce[foot * 3 + 2]);
    ////printf("[%d] %7.3f %7.3f %7.3f\n", foot, force[0], force[1], force[2]);
    f_ff[foot] = -seResult.rBody * force;
    Fr_des[foot] = force;
  }
}

void ConvexMPCLocomotion::initSparseMPC()
{
  Mat3<double> baseInertia;
  baseInertia << 0.07, 0, 0,
      0, 0.26, 0,
      0, 0, 0.242;
  double mass = 9;
  double maxForce = 120;

  std::vector<double> dtTraj;
  for (int i = 0; i < horizonLength; i++)
  {
    dtTraj.push_back(dtMPC);
  }

  /* different with MIT --- start ---------------------*/

  Vec12<double> weights;
  // weights << 0.25, 0.25, 10, 0.04, 0.04, 0.04, 0, 0, 0.3, 4, 4, 2;
  // weights << 0.25, 0.25, 10, .002, .002, 50, 0, 0, 0.3, 0.8, 0.8, 0.2;//zejun
  // weights << 0.25, 0.25, 10, 0.002, 0.002, 20, 0, 0, 0.3, 0.4, 0.4, 0.1;//zejun
  weights << 0.25, 0.25, 10, 0.2, 0.2, 50, 0, 0, 0.3, 0.8, 0.8, 0.2;
  //weights << 0,0,0,1,1,10,0,0,0,0.2,0.2,0;

  /* different with MIT --- end ---------------------*/

  _sparseCMPC.setRobotParameters(baseInertia, mass, maxForce);
  _sparseCMPC.setFriction(0.4);
  _sparseCMPC.setWeights(weights, 4e-5);
  _sparseCMPC.setDtTrajectory(dtTraj);

  _sparseTrajectory.resize(horizonLength);
}
