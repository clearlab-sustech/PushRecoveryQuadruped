//
// Created by nimapng on 12/10/19.
//

#ifndef DS_HUMANOID_H
#define DS_HUMANOID_H

#include "Dynamics/FloatingBaseModel.h"
#include "Dynamics/ActuatorModel.h"
#include "Dynamics/SpatialInertia.h"
#include "cppTypes.h"
#include "humanoid/Components.h"
#include <eigen3/Eigen/StdVector>

using std::vector;

template<typename T>
class Humanoid {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Humanoid() {};

    ~Humanoid() {};

    FloatingBaseModel<T> getFloatingBaseModel();

    Body<T> _body;

    vector<Link<T>> _links;

    vector<Rotor<T>> _rotors;

    // TODO: Check the actuator model
    vector<ActuatorModel<T>> _actuators;

    std::map<uint, uint> jointMap;

private:

    bool buildFloatingBaseModel();

    void getRotor(Rotor<T> &rotor, uint rotor_ID);

    FloatingBaseModel<T> _floatingBaseModel;

};


#endif //DS_HUMANOID_H
