//
// Created by nimapng on 12/11/19.
//

#ifndef DS_WALKER_PARAM_H
#define DS_WALKER_PARAM_H

#include "cppTypes.h"
#include "Configuration.h"

#define URDF_MODEL "/resources/urdf/walker_new.urdf"
// #define URDF_MODEL "/resources/urdf/Clear_Dog_Ros.urdf"

#define BASE_LINK "base_link"

#define LCM_TOPIC "ds"

double zero = 0.1;
Mat3<double> inertial_zero = Mat3<double>::Ones() * zero;
Mat3<double> inertial_infinte = Mat3<double>::Ones() * zero;

double gearRatio = 5.0;

std::vector<std::string> StopLinks = {"left_palm_link1", "right_palm_link1", "left_limb_l11", "right_limb_l11"};

std::vector<std::string> Links =
        {
                "base_link", "head_l1", "head_l2", "head_l3", "left_index_l1", "left_index_l2", "left_leg_l1",
                "left_leg_l2", "left_leg_l3", "left_leg_l4", "left_leg_l5", "left_leg_l6", "left_leg_link1",
                "left_leg_link2", "left_limb_l1", "left_limb_l2", "left_limb_l3", "left_limb_l4", "left_limb_l5",
                "left_limb_l6", "left_limb_l7", "left_middle_l1", "left_middle_l2", "left_palm_link", "left_pinky_l1",
                "left_pinky_l2", "left_ring_l1", "left_ring_l2", "left_thumb_l1", "left_thumb_l2", "right_index_l1",
                "right_index_l2", "right_leg_l1", "right_leg_l2", "right_leg_l3", "right_leg_l4", "right_leg_l5",
                "right_leg_l6", "right_leg_link1", "right_leg_link2", "right_limb_l1", "right_limb_l2", "right_limb_l3",
                "right_limb_l4", "right_limb_l5", "right_limb_l6", "right_limb_l7", "right_middle_l1",
                "right_middle_l2", "right_palm_link", "right_pinky_l1", "right_pinky_l2", "right_ring_l1",
                "right_ring_l2", "right_thumb_l1", "right_thumb_l2"
        };

// std::vector<std::string> Links =
//        {
//                "base_link", "fl_hip_link", "fl_upper_link", "fl_lower_link", "fr_hip_link", "fr_upper_link",
//                "fr_lower_link",
//                "hl_hip_link", "hl_upper_link", "hl_lower_link", "hr_hip_link", "hr_upper_link", "hr_lower_link"
//        };

std::vector<std::string> Joints =
        {
                "head_j1", "head_j2", "head_j3", "left_index_j1", "left_index_j2", "left_leg_j1", "left_leg_j2",
                "left_leg_j3", "left_leg_j4", "left_leg_j5", "left_leg_j6", "left_leg_joint1", "left_leg_joint2",
                "left_limb_j1", "left_limb_j2", "left_limb_j3", "left_limb_j4", "left_limb_j5", "left_limb_j6",
                "left_limb_j7", "left_middle_j1", "left_middle_j2", "left_palm_joint", "left_pinky_j1", "left_pinky_j2",
                "left_ring_j1", "left_ring_j2", "left_thumb_j1", "left_thumb_j2", "right_index_j1", "right_index_j2",
                "right_leg_j1", "right_leg_j2", "right_leg_j3", "right_leg_j4", "right_leg_j5", "right_leg_j6",
                "right_leg_joint1", "right_leg_joint2", "right_limb_j1", "right_limb_j2", "right_limb_j3",
                "right_limb_j4", "right_limb_j5", "right_limb_j6", "right_limb_j7", "right_middle_j1",
                "right_middle_j2", "right_palm_joint", "right_pinky_j1", "right_pinky_j2", "right_ring_j1",
                "right_ring_j2", "right_thumb_j1", "right_thumb_j2"
        };

// std::vector<std::string> Joints =
//        {
//                "fl_hip_link", "fl_upper_link", "fl_lower_link", "fr_hip_link", "fr_upper_link", "fr_lower_link",
//                "hl_hip_link", "hl_upper_link", "hl_lower_link", "hr_hip_link", "hr_upper_link", "hr_lower_link"
//        };
enum class LinkIndex {
    BODY,
    HEAD_L1,
    HEAD_L2,
    HEAD_L3,
    LEFT_LIMB_L1,
    LEFT_LIMB_L2,
    LEFT_LIMB_L3,
    LEFT_LIMB_L4,
    LEFT_LIMB_L5,
    LEFT_LIMB_L6,
    LEFT_LIMB_L7,
    LEG_L1,
    LEG_L2,
    LEG_L3,
    LEG_LINK1,
    LEG_J4,
    LEG_J5,
    LEG_J6,
    LEG_J2,
    NONE
};

enum class RotorIndex {
    HEAD_J1,
    HEAD_J2,
    HEAD_J3,
    LEFT_LIMB_J1,
    LEFT_LIMB_J2,
    LEFT_LIMB_J3,
    LEFT_LIMB_J4,
    LEFT_LIMB_J5,
    LEFT_LIMB_J6,
    LEFT_LIMB_J7,
    NONE
};

#endif //DS_WALKER_PARAM_H