#ifndef CONVEXMPC_UTILS_H
#define CONVEXMPC_UTILS_H

#include <stdio.h>
#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include "common_types.h"

namespace convexmpc
{
template <class T>
inline T t_min(T a, T b)
{
  return (a < b ? a : b);
}

template <class T>
T sq(T a)
{
    return a * a;
}

inline void quat2rpy(Eigen::Quaternionf q, Eigen::Matrix<fpt,3,1>& rpy)
{
  //from my MATLAB implementation

  //edge case!
  fpt as = t_min(-2.*(q.x()*q.z()-q.w()*q.y()),.99999);
  rpy(0) = atan2(2.f*(q.x()*q.y()+q.w()*q.z()),sq(q.w()) + sq(q.x()) - sq(q.y()) - sq(q.z()));
  rpy(1) = asin(as);
  rpy(2) = atan2(2.f*(q.y()*q.z()+q.w()*q.x()),sq(q.w()) - sq(q.x()) - sq(q.y()) + sq(q.z()));
}

inline Eigen::Matrix<fpt,3,3> cross_mat(Eigen::Matrix<fpt,3,3> I_inv, Eigen::Matrix<fpt,3,1> r)
{
  Eigen::Matrix<fpt,3,3> cm;
  cm << 0.f, -r(2), r(1),
    r(2), 0.f, -r(0),
    -r(1), r(0), 0.f;
  return I_inv * cm;
}

} // namespace convexmpc
#endif  // CONVEXMPC_UTILS_H