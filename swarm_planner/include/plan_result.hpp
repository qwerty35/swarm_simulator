#pragma once
#include <sp_const.hpp>

namespace SwarmPlanning {
    class PlanResult {
    public:
        PlanningState state;
        initTraj_t initTraj; // discrete initial trajectory: [quad_idx][pi_0,...,pi_M_i]
        T_t T; // segment time: [quad_idx][T_0,...,T_M_i]
        SFC_t SFC; // safe flight corridors to avoid obstacles: [quad_idx][SFC_idx]([x_min, y_min, z_min, x_max, y_max, z_max], SFC_end_time)
        RSFC_t RSFC; // relative safe flight corridors to avoid inter-collision: [quad_idx][quad_jdx][RSFC_idx](normal_vector, RSFC_end_time)
//        std_msgs::Float64MultiArray msgs_traj_info; // dim[0] = N, dim[1] = n, data = [T_0, ... , T_M_0]; [T_0, ... , T_M_1]; ...
        std::vector<std_msgs::Float64MultiArray> msgs_traj_coef;

        // find segment index at time t
        int findSegmentIdx(int qi, double t){
            return findSegmentIdx(qi, t, t);
        }

        // find segment index at time interval [start_time, end_time]
        // if invalid time interval, return -1;
        int findSegmentIdx(int qi, double start_time, double end_time){
            if(start_time > end_time){
                return -1;
            }
            for(int m = 0; m < initTraj[qi].size() - 1; m++){
                if(T[qi][m] - SP_EPSILON < start_time && end_time < T[qi][m + 1] + SP_EPSILON){
                    return m;
                }
            }
            return -1; //
        }


        // combine T[qi], T[qj]
        std::vector<double> combineSegmentTimes(int qi, int qj) {
            std::vector<double> T_ij;
            T_ij.insert(T_ij.end(), T[qi].begin(), T[qi].end());
            T_ij.insert(T_ij.end(), T[qj].begin(), T[qj].end());

            //remove duplicated elements
            std::sort(T_ij.begin(), T_ij.end());
            T_ij.erase(std::unique(T_ij.begin(), T_ij.end()), T_ij.end());
            int iter = 0;
            while (iter < T_ij.size() - 1) {
                if (T_ij[iter + 1] - T_ij[iter] < SP_EPSILON) {
                    T_ij.erase(T_ij.begin() + iter + 1);
                } else {
                    iter++;
                }
            }
            return T_ij;
        }

        octomap::point3d initTraj_interpolation(int qi, double t){
            octomap::point3d p;
            int m = findSegmentIdx(qi, t);
            octomap::point3d p_0 = initTraj[qi][m];
            octomap::point3d p_1 = initTraj[qi][m + 1];

            double alpha = (t - T[qi][m]) / (T[qi][m+1] - T[qi][m]);
            p.x() = (1-alpha) * p_0.x() + alpha * p_1.x();
            p.y() = (1-alpha) * p_0.y() + alpha * p_1.y();
            p.z() = (1-alpha) * p_0.z() + alpha * p_1.z();
            return p;
        }
    };
}