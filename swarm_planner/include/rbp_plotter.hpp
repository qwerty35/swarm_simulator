#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "SwarmPlannerGeometric.hpp"
#include "BoxGeneratorGeometric.hpp"
#include "InitTrajPlanner.hpp"
#include <mission.hpp>
#include <param.hpp>

#define _USE_MATH_DEFINES
#include <cmath>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

class TrajPlotter {
public:
    TrajPlotter(std::shared_ptr<SwarmPlanner> _swarm_planner,
                std::shared_ptr<BoxGenerator> _box_generator,
                std::shared_ptr<InitTrajPlanner> _init_traj_planner,
                SwarmPlanning::Mission _mission,
                SwarmPlanning::Param _param)
            : swarm_planner(_swarm_planner),
              box_generator(_box_generator),
              init_traj_planner(_init_traj_planner),
              mission(_mission),
              param(_param) {
        qn = mission.qn;
        M = init_traj_planner->ts_total.size();
        ts_total = init_traj_planner->ts_total;
        outdim = 3;
        outdim_pva = 3 * outdim;

        double dt = 0.1;
        t.resize(floor(ts_total.back() / dt));
        for (int i = 0; i < t.size(); i++) {
            t[i] = i * dt;
        }

        coef.resize(qn);
        quad_state.resize(qn);
        for (int qi = 0; qi < qn; qi++) {
            float rows = swarm_planner->msgs_traj_coef[qi].layout.dim.at(0).size;
            float cols = swarm_planner->msgs_traj_coef[qi].layout.dim.at(1).size;
            std::vector<double> data = swarm_planner->msgs_traj_coef[qi].data;
            coef[qi] = Eigen::Map<Eigen::MatrixXd>(data.data(), rows, cols);

            quad_state[qi].resize(outdim_pva);
            for (int i = 0; i < outdim_pva; i++) {
                quad_state[qi][i].resize(t.size());
            }
        }
    }

    void plot(bool log) {
        update_quad_state();
        update_distance_between_agents();
        if(log){
            plot_quad_dynamics();
            plot_distance_between_agents();
        }
        ROS_INFO_STREAM("Global min_dist between agents: " << global_min_dist);
        ROS_INFO_STREAM("Total flight distance: " << trajectory_length_sum());
    }

private:
    std::shared_ptr<SwarmPlanner> swarm_planner;
    std::shared_ptr<BoxGenerator> box_generator;
    std::shared_ptr<InitTrajPlanner> init_traj_planner;
    SwarmPlanning::Mission mission;
    SwarmPlanning::Param param;

    int qn, M, outdim, outdim_pva;
    double global_min_dist;

    std::vector<double> ts_total, t, max_dist, min_dist;
    std::vector<Eigen::MatrixXd> coef;
    std::vector<std::vector<std::vector<double>>> quad_state;


    void timeMatrix(double current_time, int &index, Eigen::MatrixXd &polyder) {
        double tseg = 0;
        double tcand;

        // find segment start time tseg
        for (int m = 0; m < M; m++) {
            tcand = init_traj_planner->ts_total[m];
            if (tcand < current_time) {
                tseg = tcand;
                index = m;
            } else {
                break;
            }
        }
        tseg = current_time - tseg;
        polyder.resize(3, param.N_poly + 1);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < param.N_poly + 1; j++) {
                if (i <= j)
                    polyder(i, param.N_poly - j) =
                            ((i == 0) * 1 + (i == 1) * j + (i == 2) * j * (j - 1)) * pow(tseg, j - i);
                else
                    polyder(i, param.N_poly - j) = 0;
            }
        }
    }

    void update_quad_state() {
        for (int qi = 0; qi < qn; qi++) {
            for (int j = 0; j < t.size(); j++) {
                int index = 0;
                Eigen::MatrixXd pva, polyder;
                timeMatrix(t[j], index, polyder);
                pva = polyder * coef[qi].block((param.N_poly + 1) * index, 0, (param.N_poly + 1), 3);

                for (int i = 0; i < outdim_pva; i++) {
                    quad_state[qi][i][j] = pva(i / outdim, i % outdim);
                }
            }
        }
    }

    double trajectory_length_sum(){
        double length_sum = 0;
        for(int qi = 0; qi < qn; qi++){
            for (int i = 0; i < t.size() - 1; i++) {
                length_sum += sqrt(pow(quad_state[qi][0][i+1] - quad_state[qi][0][i], 2) +
                                   pow(quad_state[qi][1][i+1] - quad_state[qi][1][i], 2) +
                                   pow(quad_state[qi][2][i+1] - quad_state[qi][2][i], 2));
            }
        }
        return length_sum;
    }

    void plot_quad_dynamics() {
        plt::figure_size(1500, 1000);

        // Plot Quad Velocity
        plt::subplot(3, 2, 1);
        for(int qi = 0; qi < qn; qi++) {
            plt::named_plot("agent" + std::to_string(qi) , t, quad_state[qi][3]);
        }
        plt::title("velocity -x axis");

        plt::subplot(3, 2, 3);
        for(int qi = 0; qi < qn; qi++) {
            plt::named_plot("agent" + std::to_string(qi) , t, quad_state[qi][4]);
        }
        plt::title("velocity -y axis");

        plt::subplot(3, 2, 5);
        for(int qi = 0; qi < qn; qi++) {
            plt::named_plot("agent" + std::to_string(qi) , t, quad_state[qi][5]);
        }
        plt::title("velocity -z axis");

        // Plot Quad Acceleration
        plt::subplot(3, 2, 2);
        for(int qi = 0; qi < qn; qi++) {
            plt::named_plot("agent" + std::to_string(qi) , t, quad_state[qi][6]);
        }
        plt::title("acceleration -x axis");

        plt::subplot(3, 2, 4);
        for(int qi = 0; qi < qn; qi++) {
            plt::named_plot("agent" + std::to_string(qi) , t, quad_state[qi][7]);
        }
        plt::title("acceleration -y axis");

        plt::subplot(3, 2, 6);
        for(int qi = 0; qi < qn; qi++) {
            plt::named_plot("agent" + std::to_string(qi) , t, quad_state[qi][8]);
        }
        plt::title("acceleration -z axis");

        plt::legend();
        plt::show(false);
    }

    void update_distance_between_agents(){
        double max_dist_, min_dist_, dist;
        max_dist.resize(t.size());
        min_dist.resize(t.size());

        global_min_dist = SP_INFINITY;
        for(int i = 0; i < t.size(); i++) {
            max_dist_ = 0;
            min_dist_ = SP_INFINITY;
            for (int qi = 0; qi < qn; qi++) {
                for (int qj = qi + 1; qj < qn; qj++) {
                    dist = sqrt(pow(quad_state[qi][0][i] - quad_state[qj][0][i], 2) +
                                pow(quad_state[qi][1][i] - quad_state[qj][1][i], 2) +
                                pow((quad_state[qi][2][i] - quad_state[qj][2][i]) / param.downwash, 2));

                    if(dist > max_dist_){
                        max_dist_ = dist;
                    }
                    if(dist < min_dist_){
                        min_dist_ = dist;
                    }
                    if(dist < global_min_dist){
                        global_min_dist = dist;
                    }
                }
            }
            max_dist[i] = max_dist_;
            min_dist[i] = min_dist_;
        }
    }

    void plot_distance_between_agents(){
        plt::figure(1);
        plt::figure_size(1500, 1000);

        std::vector<double> collision_dist;
        collision_dist.resize(t.size());
        for(int i = 0; i < t.size(); i++){
            collision_dist[i] = 2 * mission.quad_size[0]; //TODO: heterogeous case
        }

        plt::plot(t, collision_dist);
        plt::plot(t, max_dist);
        plt::plot(t, min_dist);

        plt::title("Ellipsoidal Distance between Quadrotor");

        plt::show(false);
    }
};