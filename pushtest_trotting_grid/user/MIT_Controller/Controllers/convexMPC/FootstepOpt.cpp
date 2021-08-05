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
    // sw_lb *= -0.11;
    sw_lb *= -0.08;
    sw_lb(2) = -0.41;
    sw_ub.setOnes();
    // sw_ub *= 0.11;
    sw_ub *= 0.08;
    sw_ub(2) = -0.09;
    // R << 2, 0, 0, 2;
    Q.setIdentity();
    Q *= 10;
    Aq.setIdentity();
}

template <typename T>
Vec2<T> FootstepOpt<T>::computex_des_trotting(Vec2<T> com_pos, Vec2<T> v_com)
{
    Vec2<T> x_qp(0, 0);
    Vec2<float> vd(0.0f, 0.0f);
    Eigen::Matrix<float, 2, 2> H;
    H.setIdentity();
    Eigen::Matrix<float, 2, 1> g;
    g = -com_pos;

    Eigen::Matrix<float, 12, 2> C; //trot gait cycle 10
    Vec12<float> C_1;
    C_1 << 0, 0, -0.49379, 0, -0.49379, -0.97633, 0.49379, 0, 0, 0.49379, 0, 0.9855;
    Vec12<float> C_3;
    C_3 << 0, 0, -0.85291, 0, 0.85291, 0, -0.85291, -0.9855, 0, 0.85291, 0.9855, 0;
    C.block(0, 0, 12, 1) = -1 * C_1;
    C.block(0, 1, 12, 1) = -1 * C_3;
    Vec12<float> C_2;
    C_2 << 0, 0, -0.084897, -1, -0.084897, -0.1694, 0.084897, 0, 1, 0.084897, 0, 0.1694;
    Vec12<float> C_4;
    C_4 << 1, -1, -0.14664, 0, 0.14664, 0, -0.14665, -0.1694, 0, 0.14665, 0.1694, 0;
    Vec12<float> b_pl;
    b_pl << 10, 10, 0.1323, 10, 0.0553, 0.1873, 0.0553, 0.1084, 10, 0.1323, 0.1084, 0.1873;
    Eigen::Matrix<float, 12, 4> A_pl;
    A_pl.block(0, 0, 12, 1) = C_1;
    A_pl.block(0, 1, 12, 1) = C_2;
    A_pl.block(0, 2, 12, 1) = C_3;
    A_pl.block(0, 3, 12, 1) = C_4;
    Eigen::Matrix<float, 4, 1> sta;
    sta << com_pos[0], v_com[0], com_pos[1], v_com[1];
    Eigen::Matrix<float, 12, 1> ub;
    ub = b_pl - A_pl * sta;

    H_qp = new qpOASES::real_t[H.cols() * H.rows()];
    g_qp = new qpOASES::real_t[g.cols() * g.rows()];
    A_qp = new qpOASES::real_t[C.cols() * C.rows()];
    Aub_qp = new qpOASES::real_t[ub.cols() * ub.rows()];

    matrix_to_real_t(H_qp, H, 2, 2);
    matrix_to_real_t(g_qp, g, 2, 1);
    matrix_to_real_t(A_qp, C, 12, 2);
    matrix_to_real_t(Aub_qp, ub, 12, 1);

    nWSR = 1000;
    int nVars = 2;
    int nCons = 12;

    qpOASES::QProblem CoMOpt(nVars, nCons);
    qpOASES::real_t sln[nVars];
    qpOASES::Options op;
    op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    CoMOpt.setOptions(op);
    int rval = CoMOpt.init(H_qp, g_qp, A_qp, NULL, NULL, NULL, Aub_qp, nWSR);
    if (rval != qpOASES::SUCCESSFUL_RETURN)
        printf("[FootstepOpt] computex_des_trotting: qp failed to initialize\n");

    int rval2 = CoMOpt.getPrimalSolution(sln);
    if (rval2 != qpOASES::SUCCESSFUL_RETURN)
    {
        printf("[FootstepOpt] computex_des_trotting: qp failed to solve!\n");
        std::cout << "v_com: " << v_com.transpose() << std::endl;
    }

    for (int i = 0; i < nVars; i++)
    {
        x_qp[i] = sln[i];
    }

    delete[] H_qp;
    delete[] g_qp;
    delete[] A_qp;
    delete[] Aub_qp;

    return x_qp;
}

template <typename T>
Vec2<T> FootstepOpt<T>::computex_des_bounding_pacing(Vec2<T> com_pos, Vec2<T> v_com)
{
    Vec2<T> x_qp(0, 0);
    Vec2<float> vd(0.0f, 0.0f);
    Eigen::Matrix<float, 2, 2> H;
    H.setIdentity();
    Eigen::Matrix<float, 2, 1> g;
    g = -com_pos;
    Eigen::Matrix<float, 8, 2> C;

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
    sta << com_pos[0], v_com[0], com_pos[1], v_com[1];
    Eigen::Matrix<float, 8, 1> ub;
    ub = b_pl - A_pl * sta;

    H_qp = new qpOASES::real_t[H.cols() * H.rows()];
    g_qp = new qpOASES::real_t[g.cols() * g.rows()];
    A_qp = new qpOASES::real_t[C.cols() * C.rows()];
    Aub_qp = new qpOASES::real_t[ub.cols() * ub.rows()];

    matrix_to_real_t(H_qp, H, 2, 2);
    matrix_to_real_t(g_qp, g, 2, 1);
    matrix_to_real_t(A_qp, C, 8, 2);
    matrix_to_real_t(Aub_qp, ub, 8, 1);

    nWSR = 1000;
    int nVars = 2;
    int nCons = 8;

    qpOASES::QProblem CoMOpt(nVars, nCons);
    qpOASES::real_t sln[nVars];

    qpOASES::Options op;
    op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    CoMOpt.setOptions(op);
    int rval = CoMOpt.init(H_qp, g_qp, A_qp, NULL, NULL, NULL, Aub_qp, nWSR);
    if (rval != qpOASES::SUCCESSFUL_RETURN)
        printf("[FootstepOpt] computex_des_bounding_pacing: qp failed to initialize\n");
    int rval2 = CoMOpt.getPrimalSolution(sln);
    if (rval2 != qpOASES::SUCCESSFUL_RETURN)
    {
        printf("[FootstepOpt] computex_des_bounding_pacing: qp failed to solve!\n");
        std::cout << "v_com: " << v_com.transpose() << std::endl;
    }

    for (int i = 0; i < nVars; i++)
    {
        x_qp[i] = sln[i];
    }
    delete[] H_qp;
    delete[] g_qp;
    delete[] A_qp;
    delete[] Aub_qp;
    return x_qp;
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
void FootstepOpt<T>::ss2qp(s16 horizon, Vec2<T> x_des, Vec3<T> p_com, Vec3<T> p_hip, Vec2<T> v_com, Mat3<T> R, DMat<T> x_next, s16 iteration, s16 nrest)
{
    // auto seResult = data._stateEstimator->getResult();
    (void)x_des;
    Vec2<T> templb = (R * (p_hip + sw_lb)).block(0, 0, 2, 1);
    Vec2<T> tempub = (R * (p_hip + sw_ub)).block(0, 0, 2, 1);
    Vec2<T> x_com_pred;
    // Vec2<T> v_com = (seResult.vWorld).block(0,0,2,1);
    for (int r = 0; r < horizon; r++)
    {
        Aqp.block(2 * r, 0, 2, 2).setIdentity();
        float vx = 0;
        float vy = 0;
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
            /*             float ratio = float(r+1) / float(horizon);
            x_com_pred(0) = p_com(0) + ratio * (x_des(0) - p_com(0));
            x_com_pred(1) = p_com(1) + ratio * (x_des(1) - p_com(1)); */
            x_com_pred = p_com.block(0, 0, 2, 1) + (r + 1) * v_com * gait_cycle;

            /* std::cout << "x_com_pred: " << x_com_pred.transpose() << std::endl;
            std::cout << "templb: " << templb.transpose() << std::endl;
            std::cout << "tempub: " << tempub.transpose() << std::endl; */
        }
        else
        {
            if (r == 0)
            {
                x_com_pred = p_com.block(0, 0, 2, 1);
                if (horizon < 2)
                {
                    vx = x_next(4 * (4 + 4 * r) + 1, 0);
                    vy = x_next(4 * (4 + 4 * r) + 3, 0);
                }
                else
                {
                    vx = x_next(4 * (nrest + 5 * r) + 1, 0);
                    vy = x_next(4 * (nrest + 5 * r) + 3, 0);
                    // std::cout << "x_next: " << x_next.transpose() << std::endl;
                }
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
                if (horizon < 2)
                {
                    x_com_pred[0] = x_next(4 * (4 + 4 * r), 0);
                    x_com_pred[1] = x_next(4 * (4 + 4 * r) + 2, 0);
                    vx = x_next(4 * (4 + 4 * r) + 1, 0);
                    vy = x_next(4 * (4 + 4 * r) + 3, 0);
                }
                else
                {
                    x_com_pred[0] = x_next(4 * (nrest + 5 * r), 0);
                    x_com_pred[1] = x_next(4 * (nrest + 5 * r) + 2, 0);
                    vx = x_next(4 * (nrest + 5 * r) + 1, 0);
                    vy = x_next(4 * (nrest + 5 * r) + 3, 0);
                    // std::cout << "x_next: " << x_next.transpose() << std::endl;
                }
                // x_com_pred[0] = x_next(4*(4 + 3 * r ),0);
                // x_com_pred[1] = x_next(4*(4 + 3 * r ) + 2,0);
            }
        }

        // Vec2<T> offset = pow(0.8, double(horizon - r - 1)) * computex_des_trotting(Vec2<T>(0, 0), Vec2<T>(vx, vy));
        // Vec2<T> offset = pow(0.8, double(horizon - r - 1)) * computex_des_bounding_pacing(Vec2<T>(0, 0), Vec2<T>(vx, vy));

        Vec2<T> offset = pow(0.8, double(horizon - r - 1)) * sqrt(0.2 / 9.81) * Vec2<T>(vx, vy);
        slb.block(2 * r, 0, 2, 1) = x_com_pred + pow(1.0, double(r)) * templb + offset;
        sub.block(2 * r, 0, 2, 1) = x_com_pred + pow(1.0, double(r)) * tempub + offset;

        computex_des_bounding_pacing(Vec2<T>(0, 0), Vec2<T>(vx, vy));

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
    ss2qp(horizon, x_des, p_com, p_hip, v_com, R, x_next, iteration, nrest);
    // printf("horizon = %d\n", horizon);
    // std::cout<<"x0 = "<<x0.transpose()<<std::endl;
    // std::cout<<"x_des = "<<x_des.transpose()<<std::endl;
    qH = 2 * (Bq.transpose() * Q * Bq + Rmpc);
    qg = 2 * Bq.transpose() * Q * (x0 - x_des);
    qA = Bqp;
    qlbA = slb - Aqp * x0;
    qubA = sub - Aqp * x0;
    // std::cout << "slb = " << slb.transpose() << std::endl;
    // std::cout << "sub = " << sub.transpose() << std::endl;
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
        printf("FootstepOpt: qp failed to initialize\n");
    }
    else
    {
        objval = stepOpt.getObjVal();
    }
    int rval2 = stepOpt.getPrimalSolution(sln);
    if (rval2 != qpOASES::SUCCESSFUL_RETURN)
        printf("FootstepOpt: qp failed to solve!\n");

    for (int i = 0; i < nVars; i++)
    {
        qU[i] = sln[i];
    }
    delete[] H_qpoases;
    delete[] g_qpoases;
    delete[] A_qpoases;
    delete[] Alb_qpoases;
    delete[] Aub_qpoases;
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

// template class FootstepOpt<double>;

template class FootstepOpt<float>;