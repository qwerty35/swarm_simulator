#pragma once

#include "init_traj_planner.hpp"
#include "mission.h"

//
// This code uses third-party SIPP algorithm from https://github.com/PathPlanning/AA-SIPP-m
//
namespace SwarmPlanning {
    class SIPPPlanner : public InitTrajPlanner {
    public:
        SIPPPlanner(std::shared_ptr<DynamicEDTOctomap> _distmap_obj,
                    const SwarmPlanning::Mission &_mission,
                    const SwarmPlanning::Param &_param)
                : InitTrajPlanner(_distmap_obj, _mission, _param) {
            setConfig();
        }

        bool update(bool log, SwarmPlanning::PlanResult* planResult_ptr) override {
            if (!setEnvironment()) {
                return false;
            }

            sipp_mission.createSearch();
            sipp_mission.startSearch();
            SearchResult sr = sipp_mission.getSearchResult();
            if (sr.pathfound) {
                planResult_ptr->initTraj.resize(mission.qn);
                planResult_ptr->T.resize(mission.qn);

                if(param.initTraj_equalizeM){
                    std::vector<double> T;
                    T.emplace_back(0); // start point
                    for (size_t qi = 0; qi < mission.qn; qi++) {
                        for (auto sec : sr.pathInfo[qi].sections) {
                            T.emplace_back(sec.g + 1);
                        }
                    }
                    T.emplace_back(sr.makespan + 2); // goal point
                    std::sort(T.begin(), T.end());
                    T.erase(std::unique(T.begin(), T.end()), T.end());
                    int iter = 0;
                    while (iter < T.size() - 1) {
                        if (T[iter + 1] - T[iter] < SP_EPSILON) {
                            T.erase(T.begin() + iter + 1);
                        } else {
                            iter++;
                        }
                    }
                    // initTraj interpolation
                    for (size_t qi = 0; qi < mission.qn; ++qi) {
                        planResult_ptr->initTraj[qi].emplace_back(octomap::point3d(mission.startState[qi][0],
                                                                                   mission.startState[qi][1],
                                                                                   mission.startState[qi][2]));

                        int iter = 0;
                        for (int idx = 1; idx < T.size() - 1; idx++) {
                            while (iter < sr.pathInfo[qi].sections.size() &&
                                   sr.pathInfo[qi].sections[iter].g + 1 <= T[idx]) {
                                iter++;
                            }

                            if (iter >= sr.pathInfo[qi].sections.size()) {
                                iter = sr.pathInfo[qi].sections.size() - 1;

                                Node sec = sr.pathInfo[qi].sections[iter];
                                planResult_ptr->initTraj[qi].emplace_back(
                                        octomap::point3d(sec.i * param.grid_xy_res + grid_x_min,
                                                         sec.j * param.grid_xy_res + grid_y_min,
                                                         sec.k * param.grid_z_res + grid_z_min));
                            } else {
                                Node sec1 = sr.pathInfo[qi].sections[iter - 1];
                                Node sec2 = sr.pathInfo[qi].sections[iter];

                                double x = sec1.i * (sec2.g + 1 - T[idx]) / (sec2.g - sec1.g) +
                                           sec2.i * (T[idx] - sec1.g - 1) / (sec2.g - sec1.g);
                                double y = sec1.j * (sec2.g + 1 - T[idx]) / (sec2.g - sec1.g) +
                                           sec2.j * (T[idx] - sec1.g - 1) / (sec2.g - sec1.g);
                                double z = sec1.k * (sec2.g + 1 - T[idx]) / (sec2.g - sec1.g) +
                                           sec2.k * (T[idx] - sec1.g - 1) / (sec2.g - sec1.g);

                                planResult_ptr->initTraj[qi].emplace_back(
                                        octomap::point3d(x * param.grid_xy_res + grid_x_min,
                                                         y * param.grid_xy_res + grid_y_min,
                                                         z * param.grid_z_res + grid_z_min));
                            }
                        }

                        planResult_ptr->initTraj[qi].emplace_back(octomap::point3d(mission.goalState[qi][0],
                                                                                   mission.goalState[qi][1],
                                                                                   mission.goalState[qi][2]));
                    }
                    for (int i = 0; i < T.size(); i++) {
                        T[i] *= param.time_step;
                    }
                    for(int qi = 0; qi < mission.qn; qi++) {
                        planResult_ptr->T[qi] = T;
                    }
                }
                else { // feeling lucky version - do not linear interpolation
                    for (size_t qi = 0; qi < mission.qn; ++qi) {
                        planResult_ptr->T[qi].emplace_back(0); // start point
                        planResult_ptr->initTraj[qi].emplace_back(octomap::point3d(mission.startState[qi][0],
                                                                                   mission.startState[qi][1],
                                                                                   mission.startState[qi][2]));
                        for (int iter = 0; iter < sr.pathInfo[qi].sections.size(); iter++) {
                            Node sec = sr.pathInfo[qi].sections[iter];
                            planResult_ptr->T[qi].emplace_back((sec.g + 1) * param.time_step);
                            planResult_ptr->initTraj[qi].emplace_back(
                                    octomap::point3d(sec.i * param.grid_xy_res + grid_x_min,
                                                     sec.j * param.grid_xy_res + grid_y_min,
                                                     sec.k * param.grid_z_res + grid_z_min));
                        }
                        planResult_ptr->T[qi].emplace_back((sr.makespan + 2) * param.time_step); // goal point
                        planResult_ptr->initTraj[qi].emplace_back(octomap::point3d(mission.goalState[qi][0],
                                                                                   mission.goalState[qi][1],
                                                                                   mission.goalState[qi][2]));
                    }
                }

                ROS_INFO("SIPPPlanner: SIPP complete!");
                if(param.log){
                    ROS_INFO_STREAM("SIPPPlanner: makespan=" << sr.makespan);
                }
                planResult_ptr->state = INITTRAJ;
                return true;
            } else {
                ROS_ERROR("SIPPLauncher: SIPP Failed!");
                return false;
            }
        }

    private:
        SIPP::Mission sipp_mission;

        void setConfig() {
            Config config;
            config.allowanyangle = false;
            config.additionalwait = 60;
            config.startsafeinterval = 0;
            config.initialprioritization = CN_IP_FIFO;
            config.timelimit = 60;
            config.rescheduling = CN_DEFAULT_RESCHEDULING;
            config.planforturns = false;
            config.additionalwait = 0;
            config.connectedness = CN_DEFAULT_CONNECTEDNESS;
            config.inflatecollisionintervals = 0;
            config.loglevel = 0;
            sipp_mission.setConfig(config);
        }

        bool setEnvironment() {
            SIPP::Map map;
            map.dimx = dimx;
            map.dimy = dimy;
            map.dimz = dimz;
            map.Grid.resize(dimx);
            for (int i = 0; i < dimx; i++) {
                map.Grid[i].resize(dimy);
                for(int j = 0; j < dimy; j++){
                    map.Grid[i][j].resize(dimz);
                }
            }

            double r = 0;
            for (int qi = 0; qi < mission.qn; qi++) {
                if (r < mission.quad_size[qi]) {
                    r = mission.quad_size[qi];
                }
            }

            int x, y, z;
            double grid_margin = sqrt(pow(param.grid_xy_res/2, 2) + pow(r, 2)) - r; //for random forest case
            for (double i = grid_x_min; i <= grid_x_max; i += param.grid_xy_res) {
                for (double j = grid_y_min; j <= grid_y_max; j += param.grid_xy_res) {
                    for(double k = grid_z_min; k <= grid_z_max; k += param.grid_z_res) {
                        octomap::point3d cur_point(i, j, k);
                        float dist = distmap_obj.get()->getDistance(cur_point);
                        assert(dist >= 0);

                        if (dist < r + grid_margin + SP_EPSILON) {
                            x = round((i - grid_x_min) / param.grid_xy_res);
                            y = round((j - grid_y_min) / param.grid_xy_res);
                            z = round((k - grid_z_min) / param.grid_z_res);
                            map.Grid[x][y][z] = 1;
                        }
                    }
                }
            }

            sipp_mission.setMap(map);

            int xig, yig, zig, xfg, yfg, zfg;
            Task task;
            for (int qi = 0; qi < mission.qn; qi++) {
                xig = round((mission.startState[qi][0] - grid_x_min) / param.grid_xy_res);
                yig = round((mission.startState[qi][1] - grid_y_min) / param.grid_xy_res);
                zig = round((mission.startState[qi][2] - grid_z_min) / param.grid_xy_res);
                xfg = round((mission.goalState[qi][0] - grid_x_min) / param.grid_xy_res);
                yfg = round((mission.goalState[qi][1] - grid_y_min) / param.grid_xy_res);
                zfg = round((mission.goalState[qi][2] - grid_z_min) / param.grid_z_res);

                Agent agent;
                agent.start_i = xig;
                agent.start_j = yig;
                agent.start_k = zig;
                agent.goal_i = xfg;
                agent.goal_j = yfg;
                agent.goal_k = zfg;
                agent.id = std::to_string(qi);
                agent.size = mission.quad_size[qi] / param.grid_xy_res;
                agent.mspeed = mission.quad_speed[qi] / param.grid_xy_res;

                task.agents.emplace_back(agent);
            }
            sipp_mission.setTask(task);

            if (task.validateTask(map)) {
                return true;
            } else {
                return false;
            }
        }
    };
}