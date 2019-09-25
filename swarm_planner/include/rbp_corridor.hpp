#pragma once

#include <Eigen/Dense>

#include <init_traj_planner.hpp>
#include <mission.hpp>
#include <param.hpp>
#include <timer.hpp>

class Corridor {
public:
    SFC_t SFC; // safe flight corridors to avoid obstacles
    RSFC_t RSFC; // relative safe flight corridors to avoid inter-collision

    Corridor(std::shared_ptr<InitTrajPlanner> _initTrajPlanner_obj,
             std::shared_ptr<DynamicEDTOctomap> _distmap_obj,
             SwarmPlanning::Mission _mission,
             SwarmPlanning::Param _param)
            : initTrajPlanner_obj(std::move(_initTrajPlanner_obj)),
              distmap_obj(std::move(_distmap_obj)),
              mission(std::move(_mission)),
              param(std::move(_param))
    {
        initTraj = initTrajPlanner_obj.get()->initTraj;
        T = initTrajPlanner_obj.get()->T;
        makespan = T.back();
    }

    bool update(bool log){
        return updateObsBox(log) && updateRelBox(log);
    }

private:
    std::shared_ptr<InitTrajPlanner> initTrajPlanner_obj;
    std::shared_ptr<DynamicEDTOctomap> distmap_obj;
    SwarmPlanning::Mission mission;
    SwarmPlanning::Param param;

    initTraj_t initTraj;
    std::vector<double> T;
    double makespan;

    bool isObstacleInBox(const std::vector<double>& box, double margin){
        double x,y,z;
        int count1 = 0;
        for(double i = box[0]; i < box[3] + SP_EPSILON_FLOAT; i += param.box_xy_res) {
            int count2 = 0;
            for(double j = box[1]; j < box[4] + SP_EPSILON_FLOAT; j += param.box_xy_res) {
                int count3 = 0;
                for(double k = box[2]; k < box[5] + SP_EPSILON_FLOAT; k += param.box_z_res) {
                    x = i + SP_EPSILON_FLOAT;
                    if(count1 == 0)
                        x = box[0] - SP_EPSILON_FLOAT;
                    y = j + SP_EPSILON_FLOAT;
                    if(count2 == 0)
                        y = box[1] - SP_EPSILON_FLOAT;
                    z = k + SP_EPSILON_FLOAT;
                    if(count3 == 0)
                        z = box[2] - SP_EPSILON_FLOAT;

                    octomap::point3d cur_point(x, y, z);
                    float dist = distmap_obj.get()->getDistance(cur_point);

                    assert(dist>=0);
                    if(dist < margin - SP_EPSILON_FLOAT){
                        return true;
                    }
                    count3++;
                }
                count2++;
            }
            count1++;
        }

        return false;
    }

    bool isBoxInBoundary(const std::vector<double>& box){
        return box[0] >= param.world_x_min &&
               box[1] >= param.world_y_min &&
               box[2] >= param.world_z_min &&
               box[3] <= param.world_x_max &&
               box[4] <= param.world_y_max &&
               box[5] <= param.world_z_max;
    }

    bool isPointInBox(const octomap::point3d& point,
                      const std::vector<double>& box){
        return point.x() >= box[0] - SP_EPSILON &&
               point.y() >= box[1] - SP_EPSILON &&
               point.z() >= box[2] - SP_EPSILON &&
               point.x() <= box[3] + SP_EPSILON &&
               point.y() <= box[4] + SP_EPSILON &&
               point.z() <= box[5] + SP_EPSILON;
    }

    bool isBoxInBox(const std::vector<double>& box1,
                      const std::vector<double>& box2){
        return box1[0] <= box2[3] &&
               box1[1] <= box2[4] &&
               box1[2] <= box2[5] &&
               box2[0] <= box1[3] &&
               box2[1] <= box1[4] &&
               box2[2] <= box1[5];
    }

    void expand_box(std::vector<double>& box, double margin) {
        std::vector<double> box_cand, box_update;
        std::vector<int> axis_cand{0, 1, 2, 3, 4, 5};

        int i = -1;
        int axis;
        while (!axis_cand.empty()) {
            box_cand = box;
            box_update = box;

            //check update_box only! update_box + current_box = cand_box
            while ( !isObstacleInBox(box_update, margin) && isBoxInBoundary(box_update) ) {
                i++;
                if(i >= axis_cand.size()){
                    i = 0;
                }
                axis = axis_cand[i];

                //update current box
                box = box_cand;
                box_update = box_cand;

                //expand cand_box and get updated part of box(update_box)
                if (axis < 3) {
                    box_update[axis+3] = box_cand[axis];
                    if (axis == 2){
                        box_cand[axis] = box_cand[axis] - param.box_z_res;
                    }
                    else {
                        box_cand[axis] = box_cand[axis] - param.box_xy_res;
                    }
                    box_update[axis] = box_cand[axis];
                }
                else{
                    box_update[axis-3] = box_cand[axis];
                    if (axis == 5){
                        box_cand[axis] = box_cand[axis] + param.box_z_res;
                    }
                    else{
                        box_cand[axis] = box_cand[axis] + param.box_xy_res;
                    }
                    box_update[axis] = box_cand[axis];
                }
            }
            axis_cand.erase(axis_cand.begin() + i);
            if(i > 0) {
                i--;
            }
            else{
                i = axis_cand.size()-1;
            }
        }
    }

    bool updateObsBox(bool log){
        double x_next, y_next, z_next, dx, dy, dz;
        Timer timer;

        SFC.resize(mission.qn);
        for (size_t qi = 0; qi < mission.qn; ++qi) {
            std::vector<double> box_prev;
            for(int i=0; i<6; i++) box_prev.emplace_back(0);

            for (int i = 0; i < initTraj[qi].size()-1; i++) {
                auto state = initTraj[qi][i];
                double x = state.x();
                double y = state.y();
                double z = state.z();

                std::vector<double> box;
                auto state_next = initTraj[qi][i+1];
                x_next = state_next.x();
                y_next = state_next.y();
                z_next = state_next.z();

                if(isPointInBox(octomap::point3d(x_next, y_next, z_next), box_prev)){
                    continue;
                }

                // Initialize box
                box.emplace_back(round(std::min(x,x_next) / param.box_xy_res) * param.box_xy_res);
                box.emplace_back(round(std::min(y,y_next) / param.box_xy_res) * param.box_xy_res);
                box.emplace_back(round(std::min(z,z_next) / param.box_z_res) * param.box_z_res);
                box.emplace_back(round(std::max(x,x_next) / param.box_xy_res) * param.box_xy_res);
                box.emplace_back(round(std::max(y,y_next) / param.box_xy_res) * param.box_xy_res);
                box.emplace_back(round(std::max(z,z_next) / param.box_z_res) * param.box_z_res);
//                box.emplace_back(round(std::max(x,x_next) / param.box_xy_res) * param.box_xy_res + param.box_xy_res);
//                box.emplace_back(round(std::max(y,y_next) / param.box_xy_res) * param.box_xy_res + param.box_xy_res);
//                box.emplace_back(round(std::max(z,z_next) / param.box_z_res) * param.box_z_res + param.box_z_res);


                if (isObstacleInBox(box, mission.quad_size[qi])) {
                    ROS_ERROR("BoxGenerator: Invalid initTraj. Obstacle invades initTraj");
                    return false;
                }
                expand_box(box, mission.quad_size[qi]);

                SFC[qi].emplace_back(std::make_pair(box, -1));

                box_prev = box;
            }

            // Generate box time segment
            int box_max = SFC[qi].size();
            int path_max = initTraj[qi].size();
            Eigen::MatrixXd box_log = Eigen::MatrixXd::Zero(box_max, path_max);
            std::vector<int> ts;

            for(int i=0; i<box_max; i++){
                for(int j=0; j<path_max; j++){
                    if(isPointInBox(initTraj[qi][j], SFC[qi][i].first)){
                        if (j==0){
                            box_log(i,j) = 1;
                        }
                        else{
                            box_log(i,j) = box_log(i,j-1) + 1;
                        }
                    }
                }
            }

            int box_iter = 0;
            for(int path_iter = 0; path_iter < path_max; path_iter++) {
                if (box_iter == box_max-1){
                    if(box_log(box_iter, path_iter) > 0){
                        continue;
                    }
                    else{
                        box_iter--;
                    }
                }
                if(box_log(box_iter, path_iter) > 0 && box_log(box_iter+1, path_iter) > 0){
                    int count = 1;
                    while(path_iter+count < path_max && box_log(box_iter, path_iter+count)>0
                            && box_log(box_iter+1, path_iter+count)>0){
                        count++;
                    }
                    int obs_index = path_iter + count/2;
                    SFC[qi][box_iter].second = T[obs_index];

                    path_iter = path_iter + count/2;
                    box_iter++;
                }
                else if(box_log(box_iter, path_iter) == 0){
                    box_iter--;
                    path_iter--;
                }
            }
            SFC[qi][box_max-1].second = makespan;
        }

        timer.stop();
        ROS_INFO_STREAM("SFC runtime: " << timer.elapsedSeconds());

        return true;
    }

    bool updateRelBox(bool log){
        Timer timer;

        RSFC.resize(mission.qn);
        for(int qi = 0; qi < mission.qn; qi++){
            RSFC[qi].resize(mission.qn);
            for(int qj = qi + 1; qj < mission.qn; qj++){
                int path_size = initTraj[qi].size();
                if(initTraj[qi].size() != initTraj[qj].size()) {
                    ROS_ERROR("BoxGenerator: InitTraj's sizes must be equal");
                }

                octomap::point3d a, b, c, n, m;
                double dist, dist_min;
                for(int iter = 1; iter < T.size(); iter++){
                    a = initTraj[qj][iter - 1] - initTraj[qi][iter - 1];
                    b = initTraj[qj][iter] - initTraj[qi][iter];

                    // Coordinate transformation
                    a.z() = a.z() / param.downwash;
                    b.z() = b.z() / param.downwash;

                    // get closest point of L from origin
                    if(a == b){
                        m = a;
                    }
                    else{
                        m = a;
                        dist_min = a.norm();

                        dist = b.norm();
                        if(dist_min > dist){
                            m = b;
                            dist_min = dist;
                        }

                        n = b - a;
                        n.normalize();
                        c = a - n * a.dot(n);
                        dist = c.norm();
                        if((c-a).dot(c-b) < 0 && dist_min > dist){
                            m = c;
                        }
                    }
                    m.normalize();

                    m.z() = m.z() / param.downwash;
                    if(m.norm() == 0){
                        ROS_ERROR("BoxGenerator: initTrajs are collided with each other");
                        return false;
                    }

                    RSFC[qi][qj].emplace_back(std::make_pair(m, T[iter]));
                }
            }
        }

        timer.stop();
        ROS_INFO_STREAM("RSFC runtime: " << timer.elapsedSeconds());
        return true;
    }
};