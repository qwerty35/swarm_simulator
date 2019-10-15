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
    std::vector<double> T; // segment time

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
        makespan = initTrajPlanner_obj.get()->T.back();
    }

    bool update(bool log){
        T = initTrajPlanner_obj.get()->T;
        return updateObsBox(log) && updateRelBox(log);
    }

    bool update_flat_box(bool log){
        return updateFlatObsBox(log) && updateFlatRelBox(log) && updateTs();
    }

private:
    std::shared_ptr<InitTrajPlanner> initTrajPlanner_obj;
    std::shared_ptr<DynamicEDTOctomap> distmap_obj;
    SwarmPlanning::Mission mission;
    SwarmPlanning::Param param;

    initTraj_t initTraj;
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
        return box[0] > param.world_x_min - SP_EPSILON&&
               box[1] > param.world_y_min - SP_EPSILON&&
               box[2] > param.world_z_min - SP_EPSILON&&
               box[3] < param.world_x_max + SP_EPSILON&&
               box[4] < param.world_y_max + SP_EPSILON&&
               box[5] < param.world_z_max + SP_EPSILON;
    }

    bool isPointInBox(const octomap::point3d& point,
                      const std::vector<double>& box){
        return point.x() > box[0] - SP_EPSILON &&
               point.y() > box[1] - SP_EPSILON &&
               point.z() > box[2] - SP_EPSILON &&
               point.x() < box[3] + SP_EPSILON &&
               point.y() < box[4] + SP_EPSILON &&
               point.z() < box[5] + SP_EPSILON;
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
            std::vector<double> box_prev{0,0,0,0,0,0};

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

    bool updateFlatObsBox(bool log){
        double x_next, y_next, z_next, dx, dy, dz;

        Timer timer1;

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
                box.emplace_back(std::min(x,x_next)-param.box_xy_res/2.0);
                box.emplace_back(std::min(y,y_next)-param.box_xy_res/2.0);
                box.emplace_back(std::min(z,z_next)-param.box_z_res/2.0);
                box.emplace_back(std::max(x,x_next)+param.box_xy_res/2.0);
                box.emplace_back(std::max(y,y_next)+param.box_xy_res/2.0);
                box.emplace_back(std::max(z,z_next)+param.box_z_res/2.0);


                if (isObstacleInBox(box, mission.quad_size[qi])) {
                    std::cerr << "Invalid ecbs path! obstacle in path" << std::endl;
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

            if(log) {
                std::cout << qi << std::endl;
                std::cout << box_log << std::endl;
            }

            int box_iter = 0;
            for(int path_iter = 0; path_iter< path_max; path_iter++) {
                if (box_iter >= box_max-1){
                    break;
                }
                if(box_log(box_iter, path_iter) > 0 && box_log(box_iter+1, path_iter) > 0){
                    int count = 1;
                    while(path_iter+count < path_max && box_log(box_iter, path_iter+count)>0
                          && box_log(box_iter+1, path_iter+count)>0){
                        count++;
                    }
                    double obs_index = path_iter+count/2;
                    SFC[qi][box_iter].second = obs_index * param.time_step;
                    T.emplace_back(obs_index);

                    path_iter = path_iter+count/2;
                    box_iter++;
                }
            }
            SFC[qi][box_max-1].second = makespan * param.time_step;
        }

        timer1.stop();
        std::cout << "SFC complete!" << std::endl;
        std::cout << "SFC runtime: " << timer1.elapsedSeconds() << std::endl;
    }

    bool updateFlatRelBox(bool log){
        int sector_range[6] = {-3,-2,-1,1,2,3};

        Timer timer2;

        RSFC.resize(mission.qn);
        for(int qi=0; qi<mission.qn; qi++){
            RSFC[qi].resize(mission.qn);
            for(int qj=qi+1; qj<mission.qn; qj++){
                int path_max = std::max<int>(initTraj[qi].size(), initTraj[qj].size());
                int path_min = std::min<int>(initTraj[qi].size(), initTraj[qj].size());
                Eigen::MatrixXd sector_log = Eigen::MatrixXd::Zero(6, path_max);

                for(int iter=0; iter<path_max; iter++){
                    // Get rel_pose
                    int rel_pose[4];
                    double dx, dy, dz;

                    if(iter < path_min){
                        dx = round((initTraj[qj][iter].x()-initTraj[qi][iter].x())/param.grid_xy_res);
                        dy = round((initTraj[qj][iter].y()-initTraj[qi][iter].y())/param.grid_xy_res);
                        dz = round((initTraj[qj][iter].z()-initTraj[qi][iter].z())/param.grid_z_res);
                    }
                    else if(initTraj[qi].size() == path_min){
                        dx = round((initTraj[qj][iter].x()-initTraj[qi][path_min-1].x())/param.grid_xy_res);
                        dy = round((initTraj[qj][iter].y()-initTraj[qi][path_min-1].y())/param.grid_xy_res);
                        dz = round((initTraj[qj][iter].z()-initTraj[qi][path_min-1].z())/param.grid_z_res);
                    }
                    else{
                        dx = round((initTraj[qj][path_min-1].x()-initTraj[qi][iter].x())/param.grid_xy_res);
                        dy = round((initTraj[qj][path_min-1].y()-initTraj[qi][iter].y())/param.grid_xy_res);
                        dz = round((initTraj[qj][path_min-1].z()-initTraj[qi][iter].z())/param.grid_z_res);
                    }
                    // Caution: (q1_size+q2_size)/grid_size should be small enough!
                    rel_pose[1] = (dx>SP_EPSILON_FLOAT)-(dx<-SP_EPSILON_FLOAT);
                    rel_pose[2] = (dy>SP_EPSILON_FLOAT)-(dy<-SP_EPSILON_FLOAT);
                    rel_pose[3] = (dz>SP_EPSILON_FLOAT)-(dz<-SP_EPSILON_FLOAT);

                    // Save sector information
                    for(int i=0; i<6; i++){
                        int sector = sector_range[i];
                        int sgn = (i>2)-(i<3);
                        if(rel_pose[abs(sector)]*sgn > 0){
                            if(iter == 0){
                                sector_log(i,iter) = 1;
                            }
                            else{
                                sector_log(i,iter) = sector_log(i,iter-1)+1;
                            }
                        }
                    }
                }

                //find minimum jump sector path (heuristic greedy search)
                int iter = path_max-1;
                int sector_next = -1;
                int count_next = sector_log.col(iter).maxCoeff(&sector_next);

                RSFC[qi][qj].emplace_back(std::make_pair(sec2normVec(sector_range[sector_next]), makespan*param.time_step));
                iter = iter - count_next + 1;

                while(iter > 1){
                    int sector_curr;
                    int count;

                    // if there is no intersection then allow to jump sector
                    // except jumping through quadrotor (i.e. +x -> -x jumping is not allowed)
                    if(sector_log.col(iter).maxCoeff(&sector_curr) <= 1){
                        iter = iter - 1;
                        int sector_opp = 6-1-sector_next;

                        if (sector_log.col(iter).maxCoeff(&sector_curr) <= 0){
                            std::cerr << "Invalid Path, missing link" << std::endl;
                        }
                        else if(sector_curr == sector_opp){
                            bool flag = false;
                            for(int i=0; i<6; i++){
                                if(i != sector_opp && sector_log(i,iter) == sector_log.col(iter).maxCoeff(&sector_curr)){
                                    flag = true;
                                    break;
                                }
                            }
                            if(!flag) {
                                std::cerr << "Invalid Path, jumping through quadrotor" << std::endl;
                            }
                        }
                        count = 0;
                    }
                    else{
                        count = 1;
                        while(sector_log(sector_curr,iter+count) > 0){
                            count++;
                        }
                    }

                    double rel_index;
                    if(count == 0){
                        rel_index = iter + 0.5;
                    }
                    else{
                        rel_index = floor(iter+count/2.0);
                    }

                    RSFC[qi][qj].insert(RSFC[qi][qj].begin(),
                                        std::make_pair(sec2normVec(sector_range[sector_curr]), rel_index * param.time_step));
                    T.emplace_back(rel_index);

                    sector_next = sector_curr;
                    iter = iter - sector_log.col(iter).maxCoeff() + 1;
                }
            }
        }

        timer2.stop();
        std::cout << "RSFC complete!" << std::endl;
        std::cout << "RSFC runtime: " << timer2.elapsedSeconds() << std::endl;
    }

    octomap::point3d sec2normVec(int sector){
        octomap::point3d n;
        n.x() = 0;
        n.y() = 0;
        n.z() = 0;
        int sgn = (sector > 0) - (sector < 0);

        switch(abs(sector))
        {
            case 1:
                n.x() = sgn;
                break;
            case 2:
                n.y() = sgn;
                break;
            case 3:
                n.z() = sgn;
                break;
            default:
                ROS_ERROR("??????");
                break;
        }

        return n;
    }

    bool updateTs(){
        Timer timer3;

        T.emplace_back(0);
        T.emplace_back(makespan);

//        // Delete redundant time delay
//        for(int qi = 0; qi < mission.qn; qi++){
//            for(int qj = qi + 1; qj < mission.qn; qj++){
//                for(int ri=0; ri<RSFC[qi][qj].size(); ri++){
//                    double t = RSFC[qi][qj][ri].second;
//                    if(t-floor(t) > 0.001){
//                        std::vector<double> obsbox_i;
//                        int bi = 0;
//                        while(bi < SFC[qi].size() && SFC[qi][bi].second < floor(t)){
//                            bi++;
//                        }
//                        if(SFC[qi][bi].second == floor(t)){
//                            //bi bi+1
//                            for(int i=0; i<3; i++) {
//                                obsbox_i.emplace_back(std::max(SFC[qi][bi].first[i],
//                                                               SFC[qi][bi+1].first[i]));
//                            }
//                            for(int i=3; i<6; i++) {
//                                obsbox_i.emplace_back(std::min(SFC[qi][bi].first[i],
//                                                               SFC[qi][bi+1].first[i]));
//                            }
//                        }
//                        else{
//                            //bi
//                            obsbox_i = SFC[qi][bi].first;
//                        }
//
//                        std::vector<double> obsbox_j;
//                        int bj = 0;
//                        while(bj < SFC[qj].size() && SFC[qj][bj].second < floor(t)){
//                            bj++;
//                        }
//                        if(SFC[qj][bj].second == floor(t)){
//                            // bj bj+1
//                            for(int j=0; j<3; j++) {
//                                obsbox_j.emplace_back(std::max(SFC[qj][bj].first[j],
//                                                               SFC[qj][bj+1].first[j]));
//                            }
//                            for(int j=3; j<6; j++) {
//                                obsbox_j.emplace_back(std::min(SFC[qj][bj].first[j],
//                                                               SFC[qj][bj+1].first[j]));
//                            }
//                        }
//                        else{
//                            obsbox_j = SFC[qj][bj].first;
//                        }
//
//                        bool flag1 = true;
//                        bool flag2 = true;
//                        int sector1 = RSFC[qi][qj][ri].first;
//                        int sector2 = RSFC[qi][qj][ri+1].first;
//
//                        if(sector1 > 0 && obsbox_j[sector1+3-1] < obsbox_i[sector1-1]+mission.quad_size[qi]+mission.quad_size[qj]){ //z axis coeff ////////////////////
//                            flag1 = false;
//                        }
//                        else if(sector1 < 0 && obsbox_j[abs(sector1)-1] > obsbox_i[abs(sector1)+3-1]-mission.quad_size[qi]-mission.quad_size[qj]){
//                            flag1 = false;
//                        }
//
//                        if(sector2 > 0 && obsbox_j[sector2+3-1] < obsbox_i[sector2-1]+mission.quad_size[qi]+mission.quad_size[qj]){ //z axis coeff ////////////////////
//                            flag2 = false;
//                        }
//                        else if(sector2 < 0 && obsbox_j[abs(sector2)-1] > obsbox_i[abs(sector2)+3-1]-mission.quad_size[qi]-mission.quad_size[qj]){
//                            flag2 = false;
//                        }
//
//                        if(flag1 && flag2){
//                            T.erase(std::find(T.begin(), T.end(), t));
//                            T.emplace_back(floor(t));
//                            RSFC[qi][qj][ri].second = floor(t);
//                        }
//                    }
//                }
//            }
//        }

        // update ts_total
        std::sort(T.begin(), T.end());
        T.erase(std::unique(T.begin(), T.end()), T.end());

//        // check isolate box and update ts_each
//        for(int qi = 0; qi < mission.qn; qi++){
//            ts_each[qi].emplace_back(0);
//            ts_each[qi].emplace_back(makespan);
//
//            for(int qj = qi + 1; qj < qn; qj++){
//                for(int ti = 1; ti < ts_total.size(); ti++){
//                    int bi = 0;
//                    while(bi < obstacle_boxes[qi].size() && obstacle_boxes[qi][bi].second < ts_total[ti]){
//                        bi++;
//                    }
//                    int bj = 0;
//                    while(bj < obstacle_boxes[qj].size() && obstacle_boxes[qj][bj].second < ts_total[ti]){
//                        bj++;
//                    }
//
//                    if(isBoxInBox(obstacle_boxes[qi][bi].first, obstacle_boxes[qj][bj].first)){
//                        ts_each[qi].emplace_back(ts_total[ti-1]);
//                        ts_each[qj].emplace_back(ts_total[ti]);
//                    }
//                }
//            }
//
//            std::sort(ts_each[qi].begin(), ts_each[qi].end());
//            ts_each[qi].erase(std::unique(ts_each[qi].begin(), ts_each[qi].end()), ts_each[qi].end());
//        }

        // scaling
        for(int i = 0; i < T.size(); i++){
            T[i] = T[i] * param.time_step;
        }

        timer3.stop();
        std::cout << "Time segment runtime: " << timer3.elapsedSeconds() << std::endl;

        return true;
    }
};