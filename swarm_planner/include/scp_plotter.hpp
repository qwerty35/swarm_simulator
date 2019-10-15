#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "scp_planner.hpp"
#include <mission.hpp>
#include <param.hpp>

#define _USE_MATH_DEFINES
#include <cmath>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

namespace SwarmPlanning {
    class SCPPlotter {
    public:
        SCPPlotter(std::shared_ptr<SCPPlanner> _SCPPlanner_obj,
                   Mission _mission,
                   Param _param)
                : SCPPlanner_obj(_SCPPlanner_obj),
                  mission(_mission),
                  param(_param) {
            N = round(SCPPlanner_obj->msgs_traj_info.data[0]);
            K = round(SCPPlanner_obj->msgs_traj_info.data[1]);
            h = SCPPlanner_obj->msgs_traj_info.data[2];
            T = 34;
            outdim = 3;

            p_curr.resize(N);

            build_mapping_mtx();
            std::vector<double> data = SCPPlanner_obj->msgs_traj_input.data;
            u = Eigen::Map<Eigen::MatrixXd>(data.data(), outdim * N * K, 1);
            p = P * u + p_start;
            v = V * u;
            a = A * u;
        }

        void plot() {
            update_traj(0.1);
            plot_distance_between_quad();
            ROS_INFO_STREAM("TrajPlotter: total length=" << trajectory_length_sum());
        }

    private:
        std::shared_ptr<SCPPlanner> SCPPlanner_obj;
        SwarmPlanning::Mission mission;
        SwarmPlanning::Param param;

        int N, K, outdim;
        double h, T;
        Eigen::MatrixXd P, V, A, J, p_start, p_goal, u, p, v, a;
        std::vector<double> t;
        std::vector<Eigen::MatrixXd> p_curr;

        void build_mapping_mtx() {
            P = Eigen::MatrixXd::Zero(outdim * N * K, outdim * N * K); // position matrix p = Pu + p_start
            V = Eigen::MatrixXd::Zero(outdim * N * K, outdim * N * K); // velocity matrix v = Vu, assume v_start = 0
            A = Eigen::MatrixXd::Identity(outdim * N * K, outdim * N * K); // accelation matrix a = Au
            J = Eigen::MatrixXd::Zero(outdim * N * K, outdim * N * K); // accelation matrix a = Au

            p_start = Eigen::MatrixXd::Zero(outdim * N * K, 1);
            p_goal = Eigen::MatrixXd::Zero(outdim * N, 1);

            for (int dim = 0; dim < outdim; dim++) {
                for (int qi = 0; qi < N; qi++) {
                    int offset = dim * N * K + qi * K;
                    for (int k = 0; k < K; k++) {
                        for (int j = 0; j < k; j++) {
                            P(offset + k, offset + j) = 0.5 * h * h * (2 * (k - j) - 1);
                            V(offset + k, offset + j) = h;
                        }
                        if (k != 0) {
                            J(offset + k, offset + k) = 1 / h;
                            J(offset + k, offset + k - 1) = -1 / h;
                        }

                        p_start(offset + k, 0) = mission.startState[qi][dim];
                    }
                    p_goal(dim * N + qi, 0) = mission.goalState[qi][dim];
                }
            }
        }

//    void plot_quad_dynamics() {
//        plt::figure_size(1500, 1000);
//
//        // Plot Quad Velocity
//        plt::subplot(3, 2, 1);
//        for(int qi = 0; qi < qn; qi++) {
//            plt::named_plot("agent" + std::to_string(qi) , t, quad_state[qi][3]);
//        }
//        plt::title("velocity -x axis");
//
//        plt::subplot(3, 2, 3);
//        for(int qi = 0; qi < qn; qi++) {
//            plt::named_plot("agent" + std::to_string(qi) , t, quad_state[qi][4]);
//        }
//        plt::title("velocity -y axis");
//
//        plt::subplot(3, 2, 5);
//        for(int qi = 0; qi < qn; qi++) {
//            plt::named_plot("agent" + std::to_string(qi) , t, quad_state[qi][5]);
//        }
//        plt::title("velocity -z axis");
//
//        // Plot Quad Acceleration
//        plt::subplot(3, 2, 2);
//        for(int qi = 0; qi < qn; qi++) {
//            plt::named_plot("agent" + std::to_string(qi) , t, quad_state[qi][6]);
//        }
//        plt::title("acceleration -x axis");
//
//        plt::subplot(3, 2, 4);
//        for(int qi = 0; qi < qn; qi++) {
//            plt::named_plot("agent" + std::to_string(qi) , t, quad_state[qi][7]);
//        }
//        plt::title("acceleration -y axis");
//
//        plt::subplot(3, 2, 6);
//        for(int qi = 0; qi < qn; qi++) {
//            plt::named_plot("agent" + std::to_string(qi) , t, quad_state[qi][8]);
//        }
//        plt::title("acceleration -z axis");
//
//        plt::legend();
//        plt::show(false);
//    }

//    void update_traj(double current_time) {
//        int k = floor(current_time / h);
//        if (k >= K - 1) {
//            return;
//        }
//
//        for (int qi = 0; qi < N; qi++) {
//            p_curr[qi] = Eigen::MatrixXd::Zero(outdim, 1);
//            Eigen::MatrixXd p_0 = Eigen::MatrixXd::Zero(outdim, 1);
//            Eigen::MatrixXd p_1 = Eigen::MatrixXd::Zero(outdim, 1);
//            for (int dim = 0; dim < outdim; dim++) {
//                p_0(dim, 0) = p(dim * N * K + qi * K + k, 0);
//                p_1(dim, 0) = p(dim * N * K + qi * K + k + 1, 0);
//            }
//            p_curr[qi] = p_0 + (current_time - k * h) / h * (p_1 - p_0);
//        }
//    }

        void update_traj(double dt) {
            t.resize(floor(T / dt));
            for (int i = 0; i < t.size(); i++) {
                t[i] = i * dt;
            }

            int k;
            Eigen::MatrixXd p_0, p_1;
            for (int qi = 0; qi < N; qi++) {
                p_curr[qi] = Eigen::MatrixXd::Zero(outdim, t.size());
                for (int i = 0; i < t.size(); i++) {
                    p_0 = Eigen::MatrixXd::Zero(outdim, 1);
                    p_1 = Eigen::MatrixXd::Zero(outdim, 1);
                    k = floor(t[i] / h);

                    for (int dim = 0; dim < outdim; dim++) {
                        p_0(dim, 0) = p(dim * N * K + qi * K + k, 0);
                        p_1(dim, 0) = p(dim * N * K + qi * K + k + 1, 0);
                    }
                    p_curr[qi].block(0, i, outdim, 1) = p_0 + (t[i] - k * h) / h * (p_1 - p_0);
                }
            }
        }


        double trajectory_length_sum() {
            double length_sum = 0;
            for (int i = 0; i < t.size() - 1; i++) {
                for (int qi = 0; qi < N; qi++) {
                    length_sum += sqrt(pow(p_curr[qi](0, i + 1) - p_curr[qi](0, i), 2) +
                                       pow(p_curr[qi](1, i + 1) - p_curr[qi](1, i), 2) +
                                       pow(p_curr[qi](2, i + 1) - p_curr[qi](2, i), 2));
                }
            }
            return length_sum;
        }

        void plot_distance_between_quad() {
            plt::figure(1);
            plt::figure_size(1500, 1000);
            std::vector<double> max_dist, min_dist;
            double max_dist_, min_dist_, dist, global_min_dist;

            max_dist.resize(t.size());
            min_dist.resize(t.size());

            for (int i = 0; i < t.size(); i++) {
                max_dist[i] = mission.quad_size[0] + mission.quad_size[0];
            }
            plt::plot(t, max_dist);

            global_min_dist = SP_INFINITY;
            for (int i = 0; i < t.size(); i++) {
                max_dist_ = 0;
                min_dist_ = SP_INFINITY;
                for (int qi = 0; qi < N; qi++) {
                    for (int qj = qi + 1; qj < N; qj++) {
                        dist = sqrt(pow(p_curr[qi](0, i) - p_curr[qj](0, i), 2) +
                                    pow(p_curr[qi](1, i) - p_curr[qj](1, i), 2) +
                                    pow(p_curr[qi](2, i) - p_curr[qj](2, i), 2));
                        if (dist > max_dist_) {
                            max_dist_ = dist;
                        }
                        if (dist < min_dist_) {
                            min_dist_ = dist;
                        }
                        if (dist < global_min_dist) {
                            global_min_dist = dist;
                        }
                    }
                }
                max_dist[i] = max_dist_;
                min_dist[i] = min_dist_;
            }
            plt::plot(t, max_dist);
            plt::plot(t, min_dist);

            ROS_INFO_STREAM("global min_dist: " << global_min_dist);

            plt::title("Ellipsoidal Distance between Quadrotor");

            plt::show(false);
        }
    };
}