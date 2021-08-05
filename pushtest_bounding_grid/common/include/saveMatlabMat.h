#ifndef _SAVEMATLABMAT_H_
#define _SAVEMATLABMAT_H_

#include <string.h>
#include <iostream>
#include <stdio.h>
#include <eigen3/Eigen/Core>
#include <matio.h>

using namespace Eigen;

using namespace std;

template <typename T>
bool SaveMatlabMat(T *src, string savePath, string matrixName, int cols, int rows);

bool savemat(MatrixXd var, std::string file_name, std::string var_name);

bool saveVec(VectorXf var, std::string file_name, std::string var_name);

bool readMatFromFile(const char *file, Eigen::MatrixXd &mat, const char *name);

#endif