//
// Created by nimapng on 12/17/19.
//

#ifndef DS_COMPONENTS_H
#define DS_COMPONENTS_H

#include "cppTypes.h"
#include "Math/orientation_tools.h"
#include "printColor.h"

typedef vector<uint> ID;

template<typename T>
struct Body {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string name;
    uint selfID;
    ID child_J;
    Vec3<T> rpy;
    Vec3<T> xyz;
    T mass;
    Mat3<T> inertia;
    Vec3<T> collisionBoxSize;

    void print() {
        std::cout << CYAN << "BodyselfID: " << selfID << ", name: " << name << RESET << std::endl
                  << "child: ";
        for (auto &child: child_J) {
            std::cout << child << ", ";
        }
        std::cout << std::endl
                  << "rpy: " << rpy[0] << ", " << rpy[1] << ", " << rpy[2] << std::endl
                  << "xyz: " << xyz[0] << ", " << xyz[1] << ", " << xyz[2] << std::endl
                  << "mass: " << mass << std::endl
                  << "inertia: " << std::endl << inertia << std::endl
                  << "collisionBoxSize: " << collisionBoxSize[0] << ", " << collisionBoxSize[1] << ", "
                  << collisionBoxSize[2] << std::endl
                  << GREEN << "*------------------------------------------------*" << std::endl;
    }
};

template<typename T>
struct Link {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string name;
    uint selfID;
    ID parent_J, child_J;
    Vec3<T> rpy;
    Vec3<T> xyz;
    T mass;
    Mat3<T> inertia;
    Vec3<T> contactPoint;

    void print() {
        std::cout << CYAN << "LinkselfID: " << selfID << ", name: " << name << RESET << std::endl
                  << "child: ";
        for (auto &child: child_J) {
            std::cout << child << ", ";
        }
        std::cout << std::endl << "parent: ";
        for (auto &parent: parent_J) {
            std::cout << parent << ", ";
        }
        std::cout << std::endl
                  << "rpy: " << rpy[0] << ", " << rpy[1] << ", " << rpy[2] << std::endl
                  << "xyz: " << xyz[0] << ", " << xyz[1] << ", " << xyz[2] << std::endl
                  << "mass: " << mass << std::endl
                  << "inertia: " << std::endl << inertia << std::endl
                  << "contactPoint: " << contactPoint[0] << ", " << contactPoint[1] << ", "
                  << contactPoint[2] << std::endl
                  << GREEN << "*------------------------------------------------*" << std::endl;
    }
};

template<typename T>
struct Rotor {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string name;
    uint selfID;
    ID parent_L, child_L;
    Vec3<T> rpy;
    Vec3<T> xyz;
    T mass;
    Mat3<T> inertia;
    T gearRatio;
    JointType type;
    CoordinateAxis revoluteAxis = CoordinateAxis::X;

    void print() {
        std::cout << CYAN << "RotorselfID: " << selfID << ", name: " << name << RESET << std::endl
                  << "child: ";
        for (auto &child: child_L) {
            std::cout << child << ", ";
        }
        std::cout << std::endl << "parent: ";
        for (auto &parent: parent_L) {
            std::cout << parent << ", ";
        }
        std::cout << std::endl
                  << "rpy: " << rpy[0] << ", " << rpy[1] << ", " << rpy[2] << std::endl
                  << "xyz: " << xyz[0] << ", " << xyz[1] << ", " << xyz[2] << std::endl
                  << "mass: " << mass << std::endl
                  << "inertia: " << std::endl << inertia << std::endl
                  << "gearRatio: " << gearRatio << std::endl
                  << "JointType[0:Prismatic, 1:Revolute, 2:FloatingBase, 3:Nothing]: " << int(type) << std::endl
                  << "revoluteAxis[0:X, 1:Y, 2:Z, 3:NX, 4:NY, 5:NZ]: " << int(revoluteAxis) << std::endl
                  << GREEN << "*------------------------------------------------*" << std::endl;
    }
};


#endif //DS_COMPONENTS_H
