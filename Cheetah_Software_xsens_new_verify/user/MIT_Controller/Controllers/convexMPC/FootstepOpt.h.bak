#ifndef _FOOTSTEP_OPT_H_
#define _FOOTSTEP_OPT_H_

#include <cppTypes.h>
#include "common_types.h"
#include <cmath>
#include <qpOASES.hpp>
#include <eigen3/Eigen/Dense>
// #include <FSM_States/ControlFSMData.h>

using Eigen::Dynamic;

template <typename T>
class FootstepOpt
{
private:
    // s16 horizon;
    // Vec2<T> x0;
    // Vec2<T> x_des;

    // Eigen::Matrix<T,2,2> R;
    Vec3<T> sw_lb;
    Vec3<T> sw_ub;
    Eigen::Matrix<T, 2, 2> Q;
    Eigen::Matrix<T, 2, 2> Aq;
    Eigen::Matrix<T, Dynamic, 2> Aqp;
    Eigen::Matrix<T, 2, Dynamic> Bq;
    Eigen::Matrix<T, Dynamic, Dynamic> Bqp;
    Eigen::Matrix<T, Dynamic, 1> slb;
    Eigen::Matrix<T, Dynamic, 1> sub;
    Eigen::Matrix<T, Dynamic, Dynamic> Rmpc;
    Eigen::Matrix<T, Dynamic, Dynamic> qH;
    Eigen::Matrix<T, Dynamic, 1> qg;
    Eigen::Matrix<T, Dynamic, Dynamic> qA;
    Eigen::Matrix<T, Dynamic, 1> qlbA;
    Eigen::Matrix<T, Dynamic, 1> qubA;
    Eigen::Matrix<T, Dynamic, 1> qU;

    // qpOASES::QProblem stepOpt;
    qpOASES::real_t *H_qpoases;
    qpOASES::real_t *g_qpoases;
    qpOASES::real_t *A_qpoases;
    qpOASES::real_t *Alb_qpoases;
    qpOASES::real_t *Aub_qpoases;
    qpOASES::int_t nWSR;

    qpOASES::real_t *H_qp;
    qpOASES::real_t *g_qp;
    qpOASES::real_t *A_qp;
    qpOASES::real_t *Aub_qp;

public:
    double objval;

    FootstepOpt();
    ~FootstepOpt()
    {
        // delete[] H_qpoases;
        // delete[] g_qpoases;
        // delete[] A_qpoases;
        // delete[] Alb_qpoases;
        // delete[] Aub_qpoases;
    }

    T h_stair;
    T w_stair;
    T pos_stair;
    T gait_cycle;

    // void setParameters(s16 horizon, Vec2<T> x_init, Vec2<T> x_final);
    void resize_qp_mats(s16 horizon);
    // void ss2qp(s16 horizon,Vec3<T> p_com, Vec3<T> p_hip, Vec2<T> v_des, Mat3<T> R, ControlFSMData<T> &data);
    // void solve_qp(Vec2<T> x0, Vec2<T> x_des, s16 horizon, Vec3<T> p_com, Vec3<T> p_hip, Vec2<T> v_des, Mat3<T> R, ControlFSMData<T> &data);
    void ss2qp(s16 horizon, Vec2<T> x_des, Vec3<T> p_com, Vec3<T> p_hip, Vec2<T> v_com, Mat3<T> R, DMat<T> x_next, s16 iteration, s16 nrest);
    void solve_qp(Vec2<T> x0, Vec2<T> x_des, s16 horizon, Vec3<T> p_com, Vec3<T> p_hip, Vec2<T> v_com, Mat3<T> R, DMat<T> x_next, s16 iteration, s16 nrest);

    void matrix_to_real_t(qpOASES::real_t *dst, Eigen::Matrix<T, Dynamic, Dynamic> src, s16 rows, s16 cols);
    Vec2<T> computex_des_trotting(Vec2<T> com_pos, Vec2<T> v_com);
    Vec2<T> computex_des_bounding_pacing(Vec2<T> com_pos, Vec2<T> v_com);

    Eigen::Matrix<T, Dynamic, 1> getResult()
    {
        return qU;
    }
};

#endif
