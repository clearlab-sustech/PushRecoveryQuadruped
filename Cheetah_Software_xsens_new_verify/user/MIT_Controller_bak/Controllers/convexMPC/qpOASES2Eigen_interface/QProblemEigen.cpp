#include "QProblemEigen.h"

QProblemEigen::QProblemEigen(int nVars, int nConstrs, HessianType hst_type) : _nVars(nVars), _nConstrs(nConstrs)
{
    assert(nVars > 0 && nConstrs >= 0);
    _QProblem = new QProblem(nVars, nConstrs, hst_type);
    qp_H = new qpOASES::real_t[nVars * nVars];
    qp_g = new qpOASES::real_t[nVars * 1];
    qp_A = new qpOASES::real_t[nConstrs * nVars];
    qp_Aub = new qpOASES::real_t[nConstrs * 1];
    qp_Alb = new qpOASES::real_t[nConstrs * 1];
    qp_ub = new qpOASES::real_t[nVars * 1];
    qp_lb = new qpOASES::real_t[nVars * 1];
}

void QProblemEigen::setup(const Eigen::MatrixXf &H,
                          const Eigen::MatrixXf &g,
                          const Eigen::MatrixXf &A,
                          const Eigen::MatrixXf &Aub,
                          const Eigen::MatrixXf &Alb,
                          const Eigen::MatrixXf &ub,
                          const Eigen::MatrixXf &lb,
                          bool isHotStart)
{
    // printf("H.cols=%ld, H.rows=%ld, nVar=%d\n", H.cols(), H.rows(), _nVars);
    assert(H.cols() == _nVars && H.rows() == _nVars);
    MatToOASESArray(H, qp_H);

    assert(g.cols() == 1 && g.rows() == _nVars);
    MatToOASESArray(g, qp_g);

    assert(A.cols() == _nVars && A.rows() == _nConstrs);
    MatToOASESArray(A, qp_A);

    assert(Alb.cols() == 1 && Alb.rows() == _nConstrs);
    MatToOASESArray(Alb, qp_Alb);

    assert(Aub.cols() == 1 && Aub.rows() == _nConstrs);
    MatToOASESArray(Aub, qp_Aub);

    assert(lb.cols() == 1 && lb.rows() == _nVars);
    MatToOASESArray(lb, qp_lb);

    assert(ub.cols() == 1 && ub.rows() == _nVars);
    MatToOASESArray(ub, qp_ub);

    real_t *_old_opt_solution = new real_t[_nVars];
    if (_QProblem->isSolved() && isHotStart)
    {
        _QProblem->getPrimalSolution(_old_opt_solution);
    }
    if (!_QProblem->isInitialised())
        setOpt();
    if (_QProblem->isInitialised())
        _QProblem->reset();

    nWSR = 5000;
    if (isHotStart)
    {
        if (_QProblem->init(qp_H, qp_g, qp_A, qp_lb, qp_ub, qp_Alb, qp_Aub, nWSR, 0, _old_opt_solution) != SUCCESSFUL_RETURN)
            throw std::runtime_error("QP initializing failed");
    }
    else
    {
        if (_QProblem->init(qp_H, qp_g, qp_A, qp_lb, qp_ub, qp_Alb, qp_Aub, nWSR, 0) != SUCCESSFUL_RETURN)
            throw std::runtime_error("QP initializing failed");
    }
}

void QProblemEigen::setup(const Eigen::MatrixXf &H,
                          const Eigen::MatrixXf &g,
                          const Eigen::MatrixXf &A,
                          const Eigen::MatrixXf &Alb,
                          bool isHotStart)
{
    // printf("H.cols=%ld, H.rows=%ld, nVar=%d\n", H.cols(), H.rows(), _nVars);
    assert(H.cols() == _nVars && H.rows() == _nVars);
    MatToOASESArray(H, qp_H);

    assert(g.cols() == 1 && g.rows() == _nVars);
    MatToOASESArray(g, qp_g);

    assert(A.cols() == _nVars && A.rows() == _nConstrs);
    MatToOASESArray(A, qp_A);

    assert(Alb.cols() == 1 && Alb.rows() == _nConstrs);
    MatToOASESArray(Alb, qp_Alb);

    setConstant(qp_Aub, _nConstrs, big_num);
    setConstant(qp_lb, _nVars, -big_num);
    setConstant(qp_ub, _nVars, big_num);

    real_t *_old_opt_solution = new real_t[_nVars];
    if (_QProblem->isSolved() && isHotStart)
    {
        _QProblem->getPrimalSolution(_old_opt_solution);
    }
    if (!_QProblem->isInitialised())
        setOpt();
    if (_QProblem->isInitialised())
        _QProblem->reset();
    nWSR = 5000;
    if (isHotStart)
    {
        if (_QProblem->init(qp_H, qp_g, qp_A, qp_lb, qp_ub, qp_Alb, qp_Aub, nWSR, 0, _old_opt_solution) != SUCCESSFUL_RETURN)
            throw std::runtime_error("QP initializing failed");
    }
    else
    {
        if (_QProblem->init(qp_H, qp_g, qp_A, qp_lb, qp_ub, qp_Alb, qp_Aub, nWSR, 0) != SUCCESSFUL_RETURN)
            throw std::runtime_error("QP initializing failed");
    }
}

void QProblemEigen::setup(const Eigen::MatrixXf &H,
                          const Eigen::MatrixXf &g,
                          const Eigen::MatrixXf &A,
                          const Eigen::MatrixXf &Alb,
                          const Eigen::MatrixXf &Aub,
                          bool isHotStart)
{
    // printf("H.cols=%ld, H.rows=%ld, nVar=%d\n", H.cols(), H.rows(), _nVars);
    assert(H.cols() == _nVars && H.rows() == _nVars);
    MatToOASESArray(H, qp_H);

    assert(g.cols() == 1 && g.rows() == _nVars);
    MatToOASESArray(g, qp_g);

    assert(A.cols() == _nVars && A.rows() == _nConstrs);
    MatToOASESArray(A, qp_A);

    assert(Alb.cols() == 1 && Alb.rows() == _nConstrs);
    MatToOASESArray(Alb, qp_Alb);

    assert(Aub.cols() == 1 && Aub.rows() == _nConstrs);
    MatToOASESArray(Aub, qp_Aub);

    setConstant(qp_lb, _nVars, -big_num);
    setConstant(qp_ub, _nVars, big_num);

    real_t *_old_opt_solution = new real_t[_nVars];
    if (_QProblem->isSolved() && isHotStart)
    {
        _QProblem->getPrimalSolution(_old_opt_solution);
    }
    else
    {
        std::memset(_old_opt_solution, 0, _nVars * sizeof(real_t));
    }
    if (!_QProblem->isInitialised())
        setOpt();
    if (_QProblem->isInitialised())
        _QProblem->reset();
    
    nWSR = 5000;
    if (isHotStart)
    {
        if (_QProblem->init(qp_H, qp_g, qp_A, qp_lb, qp_ub, qp_Alb, qp_Aub, nWSR, 0, _old_opt_solution) != SUCCESSFUL_RETURN)
            throw std::runtime_error("QP initializing failed");
    }
    else
    {
        if (_QProblem->init(qp_H, qp_g, qp_A, qp_lb, qp_ub, qp_Alb, qp_Aub, nWSR, 0) != SUCCESSFUL_RETURN)
            throw std::runtime_error("QP initializing failed");
    }
}

void QProblemEigen::setConstant(qpOASES::real_t *array, int size, qpOASES::real_t num)
{
    for (int i = 0; i < size; i++)
    {
        array[i] = (qpOASES::real_t)num;
    }
}

void QProblemEigen::MatToOASESArray(const Eigen::MatrixXf &mat, qpOASES::real_t *array)
{
    for (int row = 0; row < mat.rows(); row++)
    {
        for (int col = 0; col < mat.cols(); col++)
        {
            array[row * mat.cols() + col] = (qpOASES::real_t)mat(row, col);
        }
    }
}

void QProblemEigen::getSolution(DMat<float> &opt_solution)
{
    if (_QProblem->isSolved() == BT_TRUE)
    {
        real_t *_opt_solution = new real_t[_nVars];
        _QProblem->getPrimalSolution(_opt_solution);

        opt_solution.resize(_nVars, 1);
        for (int i = 0; i < _nVars; i++)
        {
            opt_solution(i, 0) = (float)_opt_solution[i];
        }
    }
    else
    {
        throw std::runtime_error("QP got no solution");
    }
}

void QProblemEigen::getValue(double &opt_value)
{
    if (_QProblem->isSolved() == BT_TRUE)
    {
        opt_value = _QProblem->getObjVal();
    }
    else
    {
        throw std::runtime_error("QP got no solution");
    }
}