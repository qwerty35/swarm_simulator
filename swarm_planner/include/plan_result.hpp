#pragma once
#include <sp_const.hpp>
#include <Eigen/Dense>

namespace SwarmPlanning {
    class PlanResult {
    public:
        PlanningState state; // current planning state
        initTraj_t initTraj; // discrete initial trajectory: [quad_idx][pi_0,...,pi_M_i]
        T_t T; // segment times: [quad_idx][T_0,...,T_M_i]
        SFC_t SFC; // safe flight corridors to avoid obstacles: [quad_idx][SFC_0,...,SFC_M_i]
        RSFC_t RSFC; // safe flight corridors to avoid inter-collision: [quad_idx][quad_jdx][RSFC_0,...,RSFC_M_i]
        std::vector<Eigen::MatrixXd> coef; // trajectory coefficients: [quad_idx][M_i*(n+1) x outdim matrix]

        // Find segment index at time t
        int findSegmentIdx(int qi, double t){
            return findSegmentIdx(qi, t, t);
        }

        // Find segment index at time interval [start_time, end_time]
        // return -1 when the time interval is invalid
        int findSegmentIdx(int qi, double start_time, double end_time){
            if(state < INITTRAJ){
                ROS_ERROR("PlanResult: Initial trajectory is not planned yet.");
                return -1;
            }
            if(start_time <= end_time) {
                for (int m = 0; m < initTraj[qi].size() - 1; m++) {
                    if (T[qi][m] - SP_EPSILON < start_time && end_time < T[qi][m + 1] + SP_EPSILON) {
                        return m;
                    }
                }
            }
            ROS_ERROR("PlanResult: Invalid time interval");
            return -1;
        }

        // Combine two segment times T[qi], T[qj]
        // e.g. T[qi] = [0, 1, 5], T[qj] = [0,2,4,5]
        //      T_ij = [0,1,2,4,5]
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

        // Find the interpolated waypoint of initial trajectory when time is t
        octomap::point3d initTraj_interpolation(int qi, double t){
            octomap::point3d p;
            if(state < INITTRAJ){
                ROS_ERROR("PlanResult: Initial trajectory is not planned yet.");
            }

            int m = findSegmentIdx(qi, t);
            octomap::point3d p_0 = initTraj[qi][m];
            octomap::point3d p_1 = initTraj[qi][m + 1];

            double alpha = (t - T[qi][m]) / (T[qi][m+1] - T[qi][m]);
            p.x() = (1-alpha) * p_0.x() + alpha * p_1.x();
            p.y() = (1-alpha) * p_0.y() + alpha * p_1.y();
            p.z() = (1-alpha) * p_0.z() + alpha * p_1.z();
            return p;
        }

        // Find current position, velocity, accel
        // result: [px, py, pz;
        //          vx, vy, vz;
        //          ax, ay, az;]
        Eigen::MatrixXd currentState(const Param& param, int qi, double current_time) {
            if(state < OPTIMIZATION){
                ROS_ERROR("PlanResult: There is no trajectory coefficients");
            }

            int m = findSegmentIdx(qi, current_time);
            double t = current_time - T[qi][m];

            Eigen::MatrixXd polyder, currentState;
            polyder.resize(param.phi, param.n + 1);
            for (int i = 0; i < param.phi; i++) {
                for (int j = 0; j < param.n + 1; j++) {
                    if (i <= j)
                        polyder(i, param.n - j) =
                                ((i == 0) * 1 + (i == 1) * j + (i == 2) * j * (j - 1)) * pow(t, j - i);
                    else
                        polyder(i, param.n - j) = 0;
                }
            }

            currentState = polyder * coef[qi].block(m * (param.n + 1), 0, (param.n + 1), 3);
            return currentState;
        }
    };
}