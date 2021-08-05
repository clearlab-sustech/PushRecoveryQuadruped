//
// Created by nimapng on 12/17/19.
//


#include "Dynamics/FloatingBaseModel.h"
#include "humanoid/Humanoid.h"
#include "humanoid/walker_param.h"
#include "Configuration.h"
#include "urdf/model.h"
#include "printColor.h"

template<typename T>
bool AddBaseLink(std::string baseLink_name, urdf::Model &model, std::vector<Link<T>> &links,
                 std::vector<Rotor<T>> &rotors);

template<typename T>
Mat3<T> inertialVecToMatrix(urdf::InertialSharedPtr inertial);

template<typename T>
bool RecusiveAddLinksAndRotors(urdf::LinkSharedPtr current_link, urdf::JointSharedPtr current_joint,
                               std::vector<Link<T>> &links,
                               std::vector<Rotor<T>> &rotors);

template<typename T>
Humanoid<T> buildWalkerFromURDF(std::string file) {
    urdf::Model _model;
    _model.initFile(file);

    Humanoid<T> humanoid;
    AddBaseLink<T>(BASE_LINK, _model, humanoid._body);
    bool ignore;
    for (auto &child_l: _model.getLink(std::string(BASE_LINK))->child_links) {
        ignore = false;
        for (auto stop_link : StopLinks) {
            if (!child_l->name.compare(stop_link)) ignore = true;
        }
        if (ignore) {
            std::cout << YELLOW << "ignore: " << child_l->name << " and its child links" << RESET << std::endl;
            continue;
        }
        assert(RecusiveAddLinksAndRotors<T>(child_l, child_l->parent_joint, humanoid._links, humanoid._rotors));
    }
    return humanoid;
}


template<typename T>
void AddBaseLink(std::string baseLink_name, urdf::Model &model, Body<T> &body) {
    urdf::LinkConstSharedPtr _baseLink = model.getLink(baseLink_name);

    body.name = _baseLink->name;
    _baseLink->inertial->origin.rotation.getRPY(body.rpy[0], body.rpy[1], body.rpy[2]);
    body.xyz << _baseLink->inertial->origin.position.x,
            _baseLink->inertial->origin.position.y, _baseLink->inertial->origin.position.z;
    body.mass = _baseLink->inertial->mass;
    body.inertia = inertialVecToMatrix<T>(_baseLink->inertial);
    for (auto &child_J: _baseLink->child_joints) {
        uint index = 0;
        for (auto &joint: Joints) {
            if (!joint.compare(child_J->name)) break;
            index++;
        }
        body.child_J.push_back(index);
    }

    uint index = 0;
    for (auto &link: Links) {
        if (!link.compare(_baseLink->name)) break;
        index++;
    }
    body.selfID = index;

    if (urdf::Box *boxPtr = dynamic_cast<urdf::Box *>(_baseLink->collision->geometry.get())) {
        body.collisionBoxSize << boxPtr->dim.x, boxPtr->dim.y, boxPtr->dim.z;
    } else if (urdf::Mesh *meshPtr = dynamic_cast<urdf::Mesh *>(_baseLink->collision->geometry.get())) {
        body.collisionBoxSize << meshPtr->scale.x, meshPtr->scale.y, meshPtr->scale.z;
    } else {
        assert(false);
    }
}

template<typename T>
bool RecusiveAddLinksAndRotors(urdf::LinkSharedPtr current_link, urdf::JointSharedPtr current_joint,
                               std::vector<Link<T>> &links,
                               std::vector<Rotor<T>> &rotors) {
    Link<T> link;
    link.name = current_link->name;
    current_link->inertial->origin.rotation.getRPY(link.rpy[0], link.rpy[1], link.rpy[2]);
    link.xyz << current_link->inertial->origin.position.x,
            current_link->inertial->origin.position.y, current_link->inertial->origin.position.z;
    link.mass = current_link->inertial->mass;
    link.inertia = inertialVecToMatrix<T>(current_link->inertial);

    uint index = 0;
    for (auto &joint_: Joints) {
        if (!joint_.compare(current_link->parent_joint->name)) break;
        index++;
    }
    link.parent_J.push_back(index);

    for (auto &child_J: current_link->child_joints) {
        index = 0;
        for (auto &joint_: Joints) {
            if (!joint_.compare(child_J->name)) break;
            index++;
        }
        link.child_J.push_back(index);
    }
    index = 0;
    for (auto &link_: Links) {
        if (!link_.compare(current_link->name)) break;
        index++;
    }
    link.selfID = index;
    link.contactPoint = link.xyz;
    links.push_back(link);

    Rotor<T> joint;
    joint.name = current_joint->name;
    urdf::Pose &pose = current_joint->parent_to_joint_origin_transform;
    pose.rotation.getRPY(joint.rpy[0], joint.rpy[1], joint.rpy[2]);
    joint.xyz << pose.position.x, pose.position.y, pose.position.z;
    joint.mass = zero;
    joint.inertia = inertial_zero;
    joint.gearRatio = gearRatio;
    switch ((int) current_joint->type) {
        case (int) URDFJointType::REVOLUTE:
            joint.type = JointType::Revolute;
            break;
        case (int) URDFJointType::FLOATING:
            joint.type = JointType::FloatingBase;
            break;
        case (int) URDFJointType::PRISMATIC:
            joint.type = JointType::Prismatic;
            break;
        case (int) URDFJointType::FIXED:
            //TODO： check the fixed type joint's inertia
//            joint.inertia = inertial_infinte;
            joint.inertia = link.inertia;
            joint.type = JointType::Fixed;
            break;
        default:
            joint.type = JointType::Nothing;
    }

    if (current_joint->axis.x == 1) {
        joint.revoluteAxis = CoordinateAxis::X;
    } else if (current_joint->axis.x == -1) {
        joint.revoluteAxis = CoordinateAxis::NX;
    } else if (current_joint->axis.y == 1) {
        joint.revoluteAxis = CoordinateAxis::Y;
    } else if (current_joint->axis.y == -1) {
        joint.revoluteAxis = CoordinateAxis::NY;
    } else if (current_joint->axis.z == 1) {
        joint.revoluteAxis = CoordinateAxis::Z;
    } else if (current_joint->axis.z == -1) {
        joint.revoluteAxis = CoordinateAxis::NZ;
    }

    index = 0;
    for (auto &link_: Links) {
        if (!link_.compare(current_joint->parent_link_name)) break;
        index++;
    }
    joint.parent_L.push_back(index);

    index = 0;
    for (auto &link_: Links) {
        if (!link_.compare(current_joint->child_link_name)) break;
        index++;
    }
    joint.child_L.push_back(index);
    index = 0;
    for (auto &joint_: Joints) {
        if (!joint_.compare(current_joint->name)) break;
        index++;
    }
    joint.selfID = index;
    rotors.push_back(joint);

    if (current_link->child_links.size() == 0) return true;
    bool ignore;
    for (auto &child_l: current_link->child_links) {
        ignore = false;
        for (auto stop_link : StopLinks) {
            if (!child_l->name.compare(stop_link)) ignore = true;
        }
        if (!ignore) {
            if (RecusiveAddLinksAndRotors<T>(child_l, child_l->parent_joint, links, rotors)) return true;
            else return false;
        } else {
            std::cout << YELLOW << "ignore: " << child_l->name << " and its child links" << RESET << std::endl;
        }
    }
    return true;
}

template<typename T>
Mat3<T> inertialVecToMatrix(urdf::InertialSharedPtr inertial) {
    Mat3<T> _inertial;
    _inertial << inertial->ixx, inertial->ixy, inertial->ixz, inertial->ixy, inertial->iyy,
            inertial->iyz, inertial->ixz, inertial->iyz, inertial->izz;
    return _inertial;
}