#pragma once

#define SP_EPSILON          1e-9
#define SP_EPSILON_FLOAT    1e-6
#define SP_INFINITY         1e+9

#define SP_PT_RBP            0
#define SP_PT_SCP            1

#define SP_IPT_ECBS          0

#include <octomap/OcTree.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

typedef std::vector<std::vector<octomap::point3d>> initTraj_t;
typedef std::vector<std::vector<std::pair<std::vector<double>, double>>> SFC_t;
typedef std::vector<std::vector<std::vector<std::pair<octomap::point3d, double> >>> RSFC_t;

namespace SwarmPlanning{
    struct PlanResult{
        initTraj_t initTraj; // discrete initial trajectory: pi_0,...,pi_M
        std::vector<double> T; // segment time: T_0,...,T_M
        SFC_t SFC; // safe flight corridors to avoid obstacles
        RSFC_t RSFC; // relative safe flight corridors to avoid inter-collision
        std_msgs::Float64MultiArray msgs_traj_info; // [N, n, T_0, ... , T_M]
        std::vector<std_msgs::Float64MultiArray> msgs_traj_coef; //
    };
}