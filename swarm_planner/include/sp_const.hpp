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



namespace SwarmPlanning {
    enum PlanningState {
        START,
        INITTRAJ,
        SFC,
        RSFC,
        OPTIMIZATION,
    };

    struct SFC_internal{
        std::vector<double> box; // [x_min, y_min, z_min, x_max, y_max, z_max]
        double start_time;
        double end_time;
    };

    struct RSFC_internal{
        octomap::point3d normal_vector;
        double b;
        double start_time;
        double end_time;
    };

    typedef std::vector<std::vector<double>> T_t;
    typedef std::vector<std::vector<octomap::point3d>> initTraj_t;
    typedef std::vector<std::vector<SwarmPlanning::SFC_internal>> SFC_t;
    typedef std::vector<std::vector<std::vector<SwarmPlanning::RSFC_internal>>> RSFC_t;
}