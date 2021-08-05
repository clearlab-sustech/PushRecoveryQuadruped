#ifndef _RobotState
#define _RobotState

#include <eigen3/Eigen/Dense>
#include "common_types.h"

#include "common_types.h"
class RobotState
{
    public:
        void set(flt* p, flt* v, flt* q, flt* w, flt* r, flt yaw);
        void set(const flt* p_, const flt* v_, const flt* q_, const flt* w_, const flt* r_, const flt yaw_);
        //void compute_rotations();
        void print();
        Eigen::Matrix<fpt,3,1> p,v,w;
        Eigen::Matrix<fpt,3,4> r_feet;
        Eigen::Matrix<fpt,3,3> R;
        Eigen::Matrix<fpt,3,3> R_yaw;
        Eigen::Matrix<fpt,3,3> I_body;
        Eigen::Quaternionf q;
        fpt yaw;
        fpt m = 9;
        //fpt m = 50.236; //DH
    //private:
};
#endif
