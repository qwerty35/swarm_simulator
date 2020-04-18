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
                    const SwarmPlanning::Param &_param,
                    std::shared_ptr<octomap::OcTree> _octomap_obj)
                : InitTrajPlanner(_distmap_obj, _mission, _param) {
            octomap_obj = _octomap_obj;
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
                                        octomap::point3d(sec.i * param.grid_resolution + grid_x_min,
                                                         sec.j * param.grid_resolution + grid_y_min,
                                                         sec.k * param.grid_resolution + grid_z_min));
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
                                        octomap::point3d(x * param.grid_resolution + grid_x_min,
                                                         y * param.grid_resolution + grid_y_min,
                                                         z * param.grid_resolution + grid_z_min));
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
                                    octomap::point3d(sec.i * param.grid_resolution + grid_x_min,
                                                     sec.j * param.grid_resolution + grid_y_min,
                                                     sec.k * param.grid_resolution + grid_z_min));
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
        std::shared_ptr<octomap::OcTree> octomap_obj;

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
            map.GridEdge.resize(dimx);
            for (int i = 0; i < dimx; i++) {
                map.Grid[i].resize(dimy);
                map.GridEdge[i].resize(dimy);
                for(int j = 0; j < dimy; j++){
                    map.Grid[i][j].resize(dimz);
                    map.GridEdge[i][j].resize(dimz);
                    for(int k = 0; k < dimz; k++){
                        map.GridEdge[i][j][k].resize(6); // moves.size()
                    }
                }
            }

            //sort r
            std::vector<double> quad_radius;
            for(int qi = 0; qi < mission.qn; qi++){
                quad_radius.emplace_back(mission.quad_collision_model[qi][qi].r);
            }
            std::sort(quad_radius.begin(), quad_radius.end());
            quad_radius.erase(std::unique(quad_radius.begin(), quad_radius.end()), quad_radius.end());

            std::vector<Node> moves = {Node(1,0,0,1.0), Node(0,1,0,1.0), Node(0,0,1,1.0), Node(-1,0,0,1.0), Node(0,-1,0,1.0), Node(0,0,-1,1.0)};

            double x0, y0, z0, x1, y1, z1;
            int i0, j0, k0, i1, j1, k1;
            for (i0 = 0; i0 < dimx; i0++) {
                for (j0 = 0; j0 < dimy; j0++) {
                    for(k0 = 0; k0 < dimz; k0++) {
                        x0 = i0 * param.grid_resolution + grid_x_min;
                        y0 = j0 * param.grid_resolution + grid_y_min;
                        z0 = k0 * param.grid_resolution + grid_z_min;

                        for (int n = 0; n < moves.size(); n++) {
                            i1 = i0 + moves[n].i;
                            j1 = j0 + moves[n].j;
                            k1 = k0 + moves[n].k;
                            x1 = i1 * param.grid_resolution + grid_x_min;
                            y1 = j1 * param.grid_resolution + grid_y_min;
                            z1 = k1 * param.grid_resolution + grid_z_min;

                            if(i1 < 0 || i1 >= dimx || j1 < 0 || j1 >= dimy || k1 < 0 || k1 >= dimz){
                                continue;
                            }
                            if(map.GridEdge[i1][j1][k1][(n+3)%6] != 0){
                                continue;
                            }

                            int iter = quad_radius.size() - 1;
                            while (iter >= 0) {
                                double r = quad_radius[iter];
                                std::vector<double> box = {std::min(x0, x1) - r + SP_EPSILON_FLOAT,
                                                           std::min(y0, y1) - r + SP_EPSILON_FLOAT,
                                                           std::min(z0, z1) - r + SP_EPSILON_FLOAT,
                                                           std::max(x0, x1) + r - SP_EPSILON_FLOAT,
                                                           std::max(y0, y1) + r - SP_EPSILON_FLOAT,
                                                           std::max(z0, z1) + r - SP_EPSILON_FLOAT};
                                if(!isObstacleInBox(box)){
                                    double size = -r / param.grid_resolution - SP_EPSILON;
                                    if(map.Grid[i0][j0][k0] > size){
                                        map.Grid[i0][j0][k0] = size;
                                    }
                                    if(map.Grid[i1][j1][k1] > size){
                                        map.Grid[i1][j1][k1] = size;
                                    }
                                    map.GridEdge[i0][j0][k0][n] = size;
                                    map.GridEdge[i1][j1][k1][(n+3)%6] = size;
                                    break;
                                }
                                iter--;
                            }
                            if(iter == -1){ // if there is an obstacle in any radius
                                if(map.Grid[i0][j0][k0] == 0){
                                    map.Grid[i0][j0][k0] = 1;
                                }
                                if(map.Grid[i1][j1][k1] == 0){
                                    map.Grid[i1][j1][k1] = 1;
                                }
                                map.GridEdge[i0][j0][k0][n] = 1;
                                map.GridEdge[i1][j1][k1][(n+3)%6] = 1;
                            }
                        }
                    }
                }
            }

            sipp_mission.setMap(map);

            int xig, yig, zig, xfg, yfg, zfg;
            Task task;
            for (int qi = 0; qi < mission.qn; qi++) {
                xig = round((mission.startState[qi][0] - grid_x_min) / param.grid_resolution);
                yig = round((mission.startState[qi][1] - grid_y_min) / param.grid_resolution);
                zig = round((mission.startState[qi][2] - grid_z_min) / param.grid_resolution);
                xfg = round((mission.goalState[qi][0] - grid_x_min) / param.grid_resolution);
                yfg = round((mission.goalState[qi][1] - grid_y_min) / param.grid_resolution);
                zfg = round((mission.goalState[qi][2] - grid_z_min) / param.grid_resolution);

                Agent agent;
                agent.start_i = xig;
                agent.start_j = yig;
                agent.start_k = zig;
                agent.goal_i = xfg;
                agent.goal_j = yfg;
                agent.goal_k = zfg;
                agent.id = std::to_string(qi);
                agent.agent_id = qi;
                agent.collision_models.resize(mission.qn);
                for(int qj = 0; qj < mission.qn; qj++){
                    if(qj == qi)
                        continue;
                    agent.collision_models[qj] = CollisionModel(mission.quad_collision_model[qj][qi].r / param.grid_resolution,
                                                                mission.quad_collision_model[qj][qi].a / param.grid_resolution,
                                                                mission.quad_collision_model[qj][qi].b / param.grid_resolution);
                }
                agent.size = mission.quad_collision_model[qi][qi].r / param.grid_resolution;
                agent.mspeed = mission.quad_speed[qi] / param.grid_resolution;

                task.agents.emplace_back(agent);
            }
            sipp_mission.setTask(task);

            if (task.validateTask(map)) {
                return true;
            } else {
                return false;
            }
        }

        bool isObstacleInBox(const std::vector<double>& box){
            for(octomap::OcTree::leaf_bbx_iterator it = octomap_obj->begin_leafs_bbx(
                    octomap::point3d(box[0],box[1],box[2]),
                    octomap::point3d(box[3],box[4],box[5])),
                            end = octomap_obj->end_leafs_bbx(); it != end; ++it){
                if(octomap_obj->isNodeOccupied(*it)){
                    return true;
                }
            }
            return false;
        }
    };
}