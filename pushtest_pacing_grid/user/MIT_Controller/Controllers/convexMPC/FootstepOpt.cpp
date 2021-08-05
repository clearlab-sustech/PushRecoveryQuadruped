#include "FootstepOpt.h"
#include <iostream>

template <typename T>
FootstepOpt<T>::FootstepOpt()
{
    h_stair = 0.1;
    w_stair = 0.17;
    pos_stair = 0.5;
    // gait_cycle = 0.48;
    // gait_cycle = 0.12;
    gait_cycle = 0.15;

    sw_lb.setOnes();
    sw_lb *= -0.11;
    sw_lb(2) = -0.41;
    sw_ub.setOnes();
    sw_ub *= 0.11;
    sw_ub(2) = -0.09;
    // R << 2, 0, 0, 2;
    Q.setIdentity();
    Q *= 10;
    Aq.setIdentity();
}

// template<typename T>
// void FootstepOpt<T>::setParameters(s16 horizon, Vec2<T> x_init, Vec2<T> x_final)
// {
//     this->horizon = horizon;
//     this->x0 = x_init;
//     this->x_des = x_final;
// }

template <typename T>
void FootstepOpt<T>::resize_qp_mats(s16 horizon)
{
    Aqp.resize(2 * horizon, Eigen::NoChange);
    Bq.resize(2, 2 * horizon);
    Bqp.resize(2 * horizon, 2 * horizon);
    Rmpc.resize(2 * horizon, 2 * horizon);
    qH.resize(2 * horizon, 2 * horizon);
    qg.resize(2 * horizon, 1);
    qA.resize(2 * horizon, 2 * horizon);
    qlbA.resize(2 * horizon, 1);
    qubA.resize(2 * horizon, 1);
    qU.resize(2 * horizon, 1);
    slb.resize(2 * horizon, 1);
    sub.resize(2 * horizon, 1);
    // Xcom_pred.resize(2*horizon,1);

    Aqp.setZero();
    Bq.setZero();
    Bqp.setZero();
    Rmpc.setZero();
    qH.setZero();
    qg.setZero();
    qA.setIdentity();
    qlbA.setZero();
    qubA.setOnes();
    qU.setZero();
    slb.setZero();
    sub.setZero();
    // if (real_allocated)
    // {
    //     free(H_qpoases);
    //     free(g_qpoases);
    //     free(lb_qpoases);
    //     free(ub_qpoases);
    //     free(qp_sln);
    // }
}

template <typename T>
// void FootstepOpt<T>::ss2qp(s16 horizon,Vec3<T> p_com, Vec3<T> p_hip, Vec2<T> v_des, Mat3<T> R, ControlFSMData<T> &data)
void FootstepOpt<T>::ss2qp(s16 horizon, Vec3<T> p_com, Vec3<T> p_hip, Vec2<T> v_com, Mat3<T> R, DMat<T> x_next, s16 iteration, s16 nrest)
{
    // auto seResult = data._stateEstimator->getResult();
    Vec2<T> templb = (R * (p_hip + sw_lb)).block(0, 0, 2, 1);
    Vec2<T> tempub = (R * (p_hip + sw_ub)).block(0, 0, 2, 1);
    Vec2<T> x_com_pred;
    // Vec2<T> v_com = (seResult.vWorld).block(0,0,2,1);
    for (int r = 0; r < horizon; r++)
    {
        Aqp.block(2 * r, 0, 2, 2).setIdentity();
        if (iteration == 0)
        {
            // if (r == 0)
            // {
            //     x_com_pred = p_com.block(0,0,2,1) + ((nrest+1)/4)*v_com*gait_cycle;
            // }else if (r==1){
            //     x_com_pred = p_com.block(0,0,2,1) + ((nrest+1)/4+r)*v_com*gait_cycle;
            // }else if (r==2){
            //     x_com_pred = p_com.block(0,0,2,1) + r*v_com*gait_cycle;
            // }
            x_com_pred = p_com.block(0, 0, 2, 1) + (r + 1) * v_com * gait_cycle;
        }
        else
        {
            // if (r<horizon-1)
            // {
            //     x_com_pred[0] = x_next(4*(nrest + 4 * r ),0);
            //     x_com_pred[1] = x_next(4*(nrest + 4 * r ) + 2,0);
            // }else{
            //     x_com_pred = p_com.block(0,0,2,1) + 2*v_com*gait_cycle;
            // }
            x_com_pred[0] = x_next(4 * (nrest + 5 * r), 0);
            x_com_pred[1] = x_next(4 * (nrest + 5 * r) + 2, 0);

            x_com_pred[0] = x_next(4 * (4 + 5 * r), 0);
            x_com_pred[1] = x_next(4 * (4 + 5 * r) + 2, 0);

            // x_com_pred[0] = x_next(4*(4 + 3 * r ),0);
            // x_com_pred[1] = x_next(4*(4 + 3 * r ) + 2,0);
        }
        slb.block(2 * r, 0, 2, 1) = x_com_pred + templb;
        sub.block(2 * r, 0, 2, 1) = x_com_pred + tempub;

        for (int c = 0; c < horizon; c++)
        {
            if (r >= c)
            {
                Bqp.block(2 * r, 2 * c, 2, 2).setIdentity();
            }

            Bq.block(0, 2 * c, 2, 2).setIdentity();
        }
    }
    Rmpc.setIdentity();
    // Rmpc = Rmpc/horizon;
    Rmpc = Rmpc * 1;
}

template <typename T>
void FootstepOpt<T>::solve_qp(Vec2<T> x0, Vec2<T> x_des, s16 horizon, Vec3<T> p_com, Vec3<T> p_hip, Vec2<T> v_com, Mat3<T> R, DMat<T> x_next, s16 iteration, s16 nrest)
{
    resize_qp_mats(horizon);
    // ss2qp(horizon,p_com,p_hip,v_des,R,data);
    ss2qp(horizon, p_com, p_hip, v_com, R, x_next, iteration, nrest);
    // printf("horizon = %d\n", horizon);
    // std::cout<<"x0 = "<<x0.transpose()<<std::endl;
    // std::cout<<"x_des = "<<x_des.transpose()<<std::endl;
    qH = 2 * (Bq.transpose() * Q * Bq + Rmpc);
    qg = 2 * Bq.transpose() * Q * (x0 - x_des);
    qA = Bqp;
    qlbA = slb - Aqp * x0;
    qubA = sub - Aqp * x0;

    H_qpoases = new qpOASES::real_t[qH.cols() * qH.rows()];
    g_qpoases = new qpOASES::real_t[qg.cols() * qg.rows()];
    A_qpoases = new qpOASES::real_t[qA.cols() * qA.rows()];
    Alb_qpoases = new qpOASES::real_t[qlbA.cols() * qlbA.rows()];
    Aub_qpoases = new qpOASES::real_t[qubA.cols() * qubA.rows()];

    // std::cout<<"Rqp: \n"<<Rqp<<std::endl;
    // std::cout<<"qH: \n"<<qH<<std::endl;
    // std::cout<<"qg: \n"<<qg<<std::endl;
    // std::cout<<"qlb: \n"<<qlb<<std::endl;
    // std::cout<<"qub: \n"<<qub<<std::endl;

    matrix_to_real_t(H_qpoases, qH, 2 * horizon, 2 * horizon);
    matrix_to_real_t(g_qpoases, qg, 2 * horizon, 1);
    matrix_to_real_t(A_qpoases, qA, 2 * horizon, 2 * horizon);
    matrix_to_real_t(Alb_qpoases, qlbA, 2 * horizon, 1);
    matrix_to_real_t(Aub_qpoases, qubA, 2 * horizon, 1);

    nWSR = 1000;
    int nVars = 2 * horizon;
    int nCons = 2 * horizon;

    qpOASES::QProblem stepOpt(nVars, nCons);
    qpOASES::Options op;
    op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    stepOpt.setOptions(op);

    qpOASES::real_t sln[nVars];

    int rval = stepOpt.init(H_qpoases, g_qpoases, A_qpoases, NULL, NULL, Alb_qpoases, Aub_qpoases, nWSR);
    if (rval != qpOASES::SUCCESSFUL_RETURN)
    {
        printf("failed to initialize\n");
    }
    else
    {
        objval = stepOpt.getObjVal();
    }
    int rval2 = stepOpt.getPrimalSolution(sln);
    if (rval2 != qpOASES::SUCCESSFUL_RETURN)
        printf("failed to solve!\n");
    else
    {
        for (int i = 0; i < nVars; i++)
        {
            qU[i] = sln[i];
        }
    }
}

template <typename T>
void FootstepOpt<T>::matrix_to_real_t(qpOASES::real_t *dst, Eigen::Matrix<T, Dynamic, Dynamic> src, s16 rows, s16 cols)
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

template class FootstepOpt<double>;

template class FootstepOpt<float>;