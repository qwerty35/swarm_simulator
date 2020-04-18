#pragma once

#include "init_traj_planner.hpp"
#include <environment.hpp>

using namespace libMultiRobotPlanning;

namespace SwarmPlanning {
    class ECBSPlanner : public InitTrajPlanner {
    public:
        ECBSPlanner(std::shared_ptr<DynamicEDTOctomap> _distmap_obj,
                    Mission _mission,
                    Param _param,
                    std::shared_ptr<octomap::OcTree> _octomap_obj)
                : InitTrajPlanner(std::move(_distmap_obj),
                                  std::move(_mission),
                                  std::move(_param)) {
            octomap_obj = _octomap_obj;
            setGrid();
            setObstacles();
            setWaypoints();
        }

        bool update(bool log, SwarmPlanning::PlanResult* planResult_ptr) override {
            Environment mapf(ecbs_dimx, ecbs_dimy, ecbs_dimz, ecbs_grid_x_min, ecbs_grid_y_min, ecbs_grid_z_min,
                             ecbs_grid_res, ecbs_obstacles, ecbs_obstacle_sections, ecbs_goalLocations,
                             mission.quad_collision_model);
            ECBS<State, Action, int, Conflict, libMultiRobotPlanning::Constraints, Environment> ecbs(mapf, param.ecbs_w);
            std::vector<libMultiRobotPlanning::PlanResult<State, Action, int>> solution;

            // Execute ECBS algorithm
            bool success = ecbs.search(ecbs_startStates, solution, param.log);
            if (!success) {
                ROS_ERROR("ECBSPlanner: ECBS Failed!");
                return false;
            }

            // Update segment time T
            planResult_ptr->T.resize(mission.qn);
            int cost = 0;
            int makespan = 0;
            for (const auto &s : solution) {
                cost += s.cost;
                makespan = std::max<int>(makespan, s.cost);
            }
            for(int qi = 0; qi < mission.qn; qi++) {
                for (int i = 0; i <= makespan + 2; i++) {
                    planResult_ptr->T[qi].emplace_back(i * param.time_step);
                }
            }
            if (log) {
                ROS_INFO_STREAM("ECBSPlanner: M=" << planResult_ptr->T.size() - 1);
                ROS_INFO_STREAM("ECBSPlanner: makespan=" << planResult_ptr->T[0].back());
            }

            planResult_ptr->initTraj.resize(mission.qn);
            for (int qi = 0; qi < mission.qn; qi++) {
                // Append start, goal points to both ends of initial trajectory respectively
                planResult_ptr->initTraj[qi].emplace_back(octomap::point3d(mission.startState[qi][0],
                                                                           mission.startState[qi][1],
                                                                           mission.startState[qi][2]));

                for (const auto &state : solution[qi].states) {
                    planResult_ptr->initTraj[qi].emplace_back(
                            octomap::point3d(state.first.x * ecbs_grid_res[qi] + ecbs_grid_x_min[qi],
                                             state.first.y * ecbs_grid_res[qi] + ecbs_grid_y_min[qi],
                                             state.first.z * ecbs_grid_res[qi] + ecbs_grid_z_min[qi])
                            );
                }

                // The length of the initial trajectories should be equal
                while (planResult_ptr->initTraj[qi].size() <= makespan + 2) {
                    planResult_ptr->initTraj[qi].emplace_back(octomap::point3d(mission.goalState[qi][0],
                                                                               mission.goalState[qi][1],
                                                                               mission.goalState[qi][2]));
                }
            }
            planResult_ptr->state = INITTRAJ;
            return true;
        }

    private:
        std::shared_ptr<octomap::OcTree> octomap_obj;
        std::vector<std::unordered_set<Location>> ecbs_obstacles;
        std::vector<std::unordered_set<EdgeConstraint>> ecbs_obstacle_sections;
        std::vector<State> ecbs_startStates;
        std::vector<Location> ecbs_goalLocations;
        std::vector<double> ecbs_grid_x_min, ecbs_grid_y_min, ecbs_grid_z_min,
                            ecbs_grid_x_max, ecbs_grid_y_max, ecbs_grid_z_max, ecbs_grid_res;
        std::vector<int> ecbs_dimx, ecbs_dimy, ecbs_dimz;

        //
        bool setGrid(){
            ecbs_grid_x_min.resize(mission.qn);
            ecbs_grid_y_min.resize(mission.qn);
            ecbs_grid_z_min.resize(mission.qn);
            ecbs_grid_x_max.resize(mission.qn);
            ecbs_grid_y_max.resize(mission.qn);
            ecbs_grid_z_max.resize(mission.qn);
            ecbs_grid_res.resize(mission.qn);
            ecbs_dimx.resize(mission.qn);
            ecbs_dimy.resize(mission.qn);
            ecbs_dimz.resize(mission.qn);

            for(int qi = 0; qi < mission.qn; qi++){
                ecbs_grid_res[qi] = param.grid_resolution * mission.quad_speed[qi] / param.grid_standard_speed;
                ecbs_grid_x_min[qi] = ceil((param.world_x_min - SP_EPSILON_FLOAT) / ecbs_grid_res[qi]) * ecbs_grid_res[qi];
                ecbs_grid_y_min[qi] = ceil((param.world_y_min - SP_EPSILON_FLOAT) / ecbs_grid_res[qi]) * ecbs_grid_res[qi];
                ecbs_grid_z_min[qi] = ceil((param.world_z_min - SP_EPSILON_FLOAT) / ecbs_grid_res[qi]) * ecbs_grid_res[qi];
                ecbs_grid_x_max[qi] = floor((param.world_x_max + SP_EPSILON_FLOAT) / ecbs_grid_res[qi]) * ecbs_grid_res[qi];
                ecbs_grid_y_max[qi] = floor((param.world_y_max + SP_EPSILON_FLOAT) / ecbs_grid_res[qi]) * ecbs_grid_res[qi];
                ecbs_grid_z_max[qi] = floor((param.world_z_max + SP_EPSILON_FLOAT) / ecbs_grid_res[qi]) * ecbs_grid_res[qi];

                ecbs_dimx[qi] = (int) round((ecbs_grid_x_max[qi] - ecbs_grid_x_min[qi]) / ecbs_grid_res[qi]) + 1;
                ecbs_dimy[qi] = (int) round((ecbs_grid_y_max[qi] - ecbs_grid_y_min[qi]) / ecbs_grid_res[qi]) + 1;
                ecbs_dimz[qi] = (int) round((ecbs_grid_z_max[qi] - ecbs_grid_z_min[qi]) / ecbs_grid_res[qi]) + 1;
            }
        }

        // Find the location of obstacles in grid-space
        bool setObstacles() {
            ecbs_obstacles.resize(mission.qn);
            ecbs_obstacle_sections.resize(mission.qn);
            std::vector<State> moves = {State(0,1,0,0), State(0,0,1,0), State(0,0,0,1), State(0,-1,0,0), State(0,0,-1,0), State(0,0,0,-1)};

            for(int qi = 0; qi < mission.qn; qi++) {
                double r = mission.quad_collision_model[qi][qi].r;

                double x0, y0, z0, x1, y1, z1;
                int i0, j0, k0, i1, j1, k1;
                for (i0 = 0; i0 < ecbs_dimx[qi]; i0++) {
                    for (j0 = 0; j0 < ecbs_dimy[qi]; j0++) {
                        for (k0 = 0; k0 < ecbs_dimz[qi]; k0++) {
                            x0 = i0 * ecbs_grid_res[qi] + ecbs_grid_x_min[qi];
                            y0 = j0 * ecbs_grid_res[qi] + ecbs_grid_y_min[qi];
                            z0 = k0 * ecbs_grid_res[qi] + ecbs_grid_z_min[qi];

                            int countAvailableMoves = moves.size();
                            for (int n = 0; n < moves.size(); n++) {
                                i1 = i0 + moves[n].x;
                                j1 = j0 + moves[n].y;
                                k1 = k0 + moves[n].z;
                                x1 = i1 * ecbs_grid_res[qi] + ecbs_grid_x_min[qi];
                                y1 = j1 * ecbs_grid_res[qi] + ecbs_grid_y_min[qi];
                                z1 = k1 * ecbs_grid_res[qi] + ecbs_grid_z_min[qi];

                                if(i1 < 0 || i1 >= dimx || j1 < 0 || j1 >= dimy || k1 < 0 || k1 >= dimz){
                                    continue;
                                }

                                std::vector<double> box = {std::min(x0, x1) - r + SP_EPSILON_FLOAT,
                                                           std::min(y0, y1) - r + SP_EPSILON_FLOAT,
                                                           std::min(z0, z1) - r + SP_EPSILON_FLOAT,
                                                           std::max(x0, x1) + r - SP_EPSILON_FLOAT,
                                                           std::max(y0, y1) + r - SP_EPSILON_FLOAT,
                                                           std::max(z0, z1) + r - SP_EPSILON_FLOAT};
                                if(isObstacleInBox(box)){
                                    EdgeConstraint edgeConstraint(0, i0, j0, k0, i1, j1, k1);
                                    if(ecbs_obstacle_sections[qi].find(edgeConstraint) == ecbs_obstacle_sections[qi].end()){
                                        ecbs_obstacle_sections[qi].insert(edgeConstraint);
                                    }
                                    countAvailableMoves--;
                                }
                            }
                            if (countAvailableMoves == 0) {
                                ecbs_obstacles[qi].insert(Location(x0, y0, z0));
                            }
                        }
                    }
                }
            }
            return true;
        }

        // Set start, goal points of ECBS
        bool setWaypoints() {
            int xig, yig, zig, xfg, yfg, zfg;
            for (int qi = 0; qi < mission.qn; qi++) {
                // For start, goal point of ECBS, we use the nearest grid point.
                xig = (int) round((mission.startState[qi][0] - ecbs_grid_x_min[qi]) / ecbs_grid_res[qi]);
                yig = (int) round((mission.startState[qi][1] - ecbs_grid_y_min[qi]) / ecbs_grid_res[qi]);
                zig = (int) round((mission.startState[qi][2] - ecbs_grid_z_min[qi]) / ecbs_grid_res[qi]);
                xfg = (int) round((mission.goalState[qi][0] - ecbs_grid_x_min[qi]) / ecbs_grid_res[qi]);
                yfg = (int) round((mission.goalState[qi][1] - ecbs_grid_y_min[qi]) / ecbs_grid_res[qi]);
                zfg = (int) round((mission.goalState[qi][2] - ecbs_grid_z_min[qi]) / ecbs_grid_res[qi]);

                if (ecbs_obstacles[qi].find(Location(xig, yig, zig)) != ecbs_obstacles[qi].end()) {
                    ROS_ERROR_STREAM("ECBSPlanner: start of agent " << qi << " is occluded by obstacle");
                    return false;
                }
                if (ecbs_obstacles[qi].find(Location(xfg, yfg, zfg)) != ecbs_obstacles[qi].end()) {
                    ROS_ERROR_STREAM("ECBSPlanner: goal of agent " << qi << " is occluded by obstacle");
                    return false;
                }

                ecbs_startStates.emplace_back(State(0, xig, yig, zig));
                ecbs_goalLocations.emplace_back(Location(xfg, yfg, zfg));
            }
            return true;
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