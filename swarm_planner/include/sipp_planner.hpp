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
            z = 1;
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

                // update segment time T
                for (size_t qi = 0; qi < mission.qn; qi++) {
                    planResult_ptr->T[qi].emplace_back(0); // start point
                    for (auto sec : sr.pathInfo[qi].sections) {
                        planResult_ptr->T[qi].emplace_back((sec.g + 1) * param.time_step);
                    }
                    planResult_ptr->T[qi].emplace_back((sr.makespan + 2) * param.time_step); // goal point
                }

//                std::vector<double> ts_total;
//                ts_total.emplace_back(0); // start point
//                for (size_t qi = 0; qi < mission.qn; qi++) {
//                    for (auto sec : sr.pathInfo[qi].sections) {
//                        ts_total.emplace_back(sec.g + 1);
//                    }
//                }
//                ts_total.emplace_back(sr.makespan + 2); // goal point
//                std::sort(ts_total.begin(), ts_total.end());
//                ts_total.erase(std::unique(ts_total.begin(), ts_total.end()), ts_total.end());
//                int iter = 0;
//                while (iter < ts_total.size() - 1) {
//                    if (ts_total[iter + 1] - ts_total[iter] < SP_EPSILON) {
//                        ts_total.erase(ts_total.begin() + iter + 1);
//                    } else {
//                        iter++;
//                    }
//                }

                // feeling lucky version - do not linear interpolation
                for (size_t qi = 0; qi < mission.qn; ++qi) {
                    planResult_ptr->initTraj[qi].emplace_back(octomap::point3d(mission.startState[qi][0],
                                                                               mission.startState[qi][1],
                                                                               mission.startState[qi][2]));

                    for (int iter = 0; iter < sr.pathInfo[qi].sections.size(); iter++) {
                        Node sec = sr.pathInfo[qi].sections[iter];
                        planResult_ptr->initTraj[qi].emplace_back(octomap::point3d(sec.i * param.grid_xy_res + grid_x_min,
                                                                                   sec.j * param.grid_xy_res + grid_y_min,
                                                                                      z));
                    }

                    planResult_ptr->initTraj[qi].emplace_back(octomap::point3d(mission.goalState[qi][0],
                                                                               mission.goalState[qi][1],
                                                                               mission.goalState[qi][2]));
                }

//                // initTraj interpolation
//                for (size_t qi = 0; qi < mission.qn; ++qi) {
//                    initTraj[qi].emplace_back(octomap::point3d(mission.startState[qi][0],
//                                                               mission.startState[qi][1],
//                                                               mission.startState[qi][2]));
//
//                    int iter = 0;
//                    for (int idx = 1; idx < ts_total.size() - 1; idx++) {
//                        while (iter < sr.pathInfo[qi].sections.size() &&
//                               sr.pathInfo[qi].sections[iter].g + 1 <= ts_total[idx]) {
//                            iter++;
//                        }
//
//                        if (iter >= sr.pathInfo[qi].sections.size()) {
//                            iter = sr.pathInfo[qi].sections.size() - 1;
//
//                            Node sec = sr.pathInfo[qi].sections[iter];
//                            initTraj[qi].emplace_back(octomap::point3d(sec.i * param.grid_xy_res + grid_x_min,
//                                                                       sec.j * param.grid_xy_res + grid_y_min,
//                                                                       z));
//                        } else {
//                            Node sec1 = sr.pathInfo[qi].sections[iter - 1];
//                            Node sec2 = sr.pathInfo[qi].sections[iter];
//
//                            double x = sec1.i * (sec2.g + 1 - ts_total[idx]) / (sec2.g - sec1.g) +
//                                       sec2.i * (ts_total[idx] - sec1.g - 1) / (sec2.g - sec1.g);
//                            double y = sec1.j * (sec2.g + 1 - ts_total[idx]) / (sec2.g - sec1.g) +
//                                       sec2.j * (ts_total[idx] - sec1.g - 1) / (sec2.g - sec1.g);
//
//                            initTraj[qi].emplace_back(octomap::point3d(x * param.grid_xy_res + grid_x_min,
//                                                                       y * param.grid_xy_res + grid_y_min,
//                                                                       z));
//                        }
//                    }
//
//                    initTraj[qi].emplace_back(octomap::point3d(mission.goalState[qi][0],
//                                                               mission.goalState[qi][1],
//                                                               mission.goalState[qi][2]));
//                }
//
//                for (int i = 0; i < ts_total.size(); i++) {
//                    ts_total[i] *= param.time_step;
//                }

                ROS_INFO("SIPPPlanner: SIPP complete!");
                planResult_ptr->state = INITTRAJ;
                return true;
            } else {
                ROS_ERROR("SIPPLauncher: SIPP Failed!");
                return false;
            }
        }

    private:
        double z;
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
            Map map;
            map.width = dimx;
            map.height = dimy;
            map.Grid.resize(dimx);
            for (int i = 0; i < dimx; i++) {
                map.Grid[i].resize(dimy);
            }

            double r = 0;
            for (int qi = 0; qi < mission.qn; qi++) {
                if (r < mission.quad_size[qi]) {
                    r = mission.quad_size[qi];
                }
            }

            int x, y;
            for (double i = grid_x_min; i <= grid_x_max; i += param.grid_xy_res) {
                for (double j = grid_y_min; j <= grid_y_max; j += param.grid_xy_res) {
                    octomap::point3d cur_point(i, j, z);
                    float dist = distmap_obj.get()->getDistance(cur_point);
                    assert(dist >= 0);

                    if (dist < r + param.grid_margin) {
                        x = round((i - grid_x_min) / param.grid_xy_res);
                        y = round((j - grid_y_min) / param.grid_xy_res);
                        map.Grid[x][y] = 1;
                    }
                }
            }

            sipp_mission.setMap(map);

            int xig, yig, zig, xfg, yfg, zfg;
            Task task;
            for (int qi = 0; qi < mission.qn; qi++) {
                xig = round((mission.startState[qi][0] - grid_x_min) / param.grid_xy_res);
                yig = round((mission.startState[qi][1] - grid_y_min) / param.grid_xy_res);
                xfg = round((mission.goalState[qi][0] - grid_x_min) / param.grid_xy_res);
                yfg = round((mission.goalState[qi][1] - grid_y_min) / param.grid_xy_res);

                if (mission.startState[qi][2] != z || mission.goalState[qi][2] != z) {
                    ROS_WARN("SIPPPlanner : SIPP supports only 2D case, z is fixed to 1"); //TODO: SIPP 2D to 3D
                    continue;
                }

                Agent agent;
                agent.start_i = xig;
                agent.start_j = yig;
                agent.goal_i = xfg;
                agent.goal_j = yfg;
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