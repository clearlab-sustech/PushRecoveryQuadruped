#ifndef _QPROBLEMEIGEN_H
#define _QPROBLEMEIGEN_H

#include <cppTypes.h>
#include <qpOASES.hpp>
#include <cmath>

using namespace qpOASES;

class QProblemEigen
{
public:
    QProblemEigen(int nVars, int nConstrs, HessianType hst_type);

    void setup(const Eigen::MatrixXf &H,
               const Eigen::MatrixXf &g,
               const Eigen::MatrixXf &A,
               const Eigen::MatrixXf &Aub,
               const Eigen::MatrixXf &Alb,
               const Eigen::MatrixXf &ub,
               const Eigen::MatrixXf &lb,
               bool isHotStart = false);

    void setup(const Eigen::MatrixXf &H,
               const Eigen::MatrixXf &g,
               const Eigen::MatrixXf &A,
               const Eigen::MatrixXf &Alb,
               bool isHotStart = false);

    void setup(const Eigen::MatrixXf &H,
               const Eigen::MatrixXf &g,
               const Eigen::MatrixXf &A,
               const Eigen::MatrixXf &Alb,
               const Eigen::MatrixXf &Aub,
               bool isHotStart = false);

    void getSolution(DMat<float> &opt_solution);

    void getValue(double &opt_value);

    qpOASES::Options options;

    int_t nWSR = 100;

private:
    void setConstant(qpOASES::real_t *array, int size, qpOASES::real_t num);

    void MatToOASESArray(const Eigen::MatrixXf &mat, qpOASES::real_t *array);

    void setOpt() { _QProblem->setOptions(options); }

    int _nVars, _nConstrs;
    qpOASES::real_t *qp_H, *qp_g, *qp_A, *qp_Aub, *qp_Alb, *qp_ub, *qp_lb;
    qpOASES::QProblem *_QProblem;

    qpOASES::real_t big_num = (qpOASES::real_t)1.0e13;
};

#endif