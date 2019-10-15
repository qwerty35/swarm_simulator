#pragma once

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ROS
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// MATPLOTLIB-CPP
#define _USE_MATH_DEFINES
#include <cmath>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
//#include <thread>

// Submodules
#include <rbp_planner.hpp>
#include <rbp_corridor.hpp>
#include <init_traj_planner.hpp>
#include <mission.hpp>
#include <param.hpp>

class ResultPublisher {
public:
    ResultPublisher(ros::NodeHandle _nh,
                  std::shared_ptr<RBPPlanner> _RBPPlanner_obj,
                  std::shared_ptr<Corridor> _corridor_obj,
                  std::shared_ptr<InitTrajPlanner> _initTrajPlanner_obj,
                  SwarmPlanning::Mission _mission,
                  SwarmPlanning::Param _param)
            : nh(std::move(_nh)),
              RBPPlanner_obj(std::move(_RBPPlanner_obj)),
              corridor_obj(std::move(_corridor_obj)),
              initTrajPlanner_obj(std::move(_initTrajPlanner_obj)),
              mission(std::move(_mission)),
              param(std::move(_param))
    {
        qn = mission.qn;
        outdim = 3;

        M = RBPPlanner_obj->msgs_traj_info.data.size() - 2 - 1;
        T.resize(M + 1);
        for(int m = 0; m < M+1; m++){
            T[m] = RBPPlanner_obj->msgs_traj_info.data.at(m + 2);
        }

        dt = 0.1;
        t.resize(floor(T.back() / dt));
        for (int i = 0; i < t.size(); i++) {
            t[i] = i * dt;
        }

        pva.resize(qn);
        coef.resize(qn);
        quad_state.resize(qn);
        currentState.resize(qn);
        for(int qi = 0; qi < qn; qi++) {
            float rows = RBPPlanner_obj->msgs_traj_coef[qi].layout.dim.at(0).size;
            float cols = RBPPlanner_obj->msgs_traj_coef[qi].layout.dim.at(1).size;
            std::vector<double> data = RBPPlanner_obj->msgs_traj_coef[qi].data;
            coef[qi] = Eigen::Map<Eigen::MatrixXd>(data.data(), rows, cols);

            currentState[qi].resize(outdim * param.phi);
            quad_state[qi].resize(outdim * param.phi);
            for (int i = 0; i < outdim * param.phi; i++) {
                quad_state[qi][i].resize(t.size());
            }
        }

        pubs_traj_coef.resize(qn);
        pubs_traj.resize(qn);
        pubs_relBox.resize(qn);
        for(int qi = 0; qi < qn; qi++){
            std::string mav_name = "/mav" + std::to_string(qi);
            pubs_traj_coef[qi] = nh.advertise<std_msgs::Float64MultiArray>("/traj_coef" + mav_name, 1);
            pubs_traj[qi] = nh.advertise<nav_msgs::Path>("/desired_trajectory" + mav_name, 1);
            pubs_relBox[qi] = nh.advertise<visualization_msgs::MarkerArray>("/relative_box" + mav_name, 1);
        }
        pub_traj_info = nh.advertise<std_msgs::Float64MultiArray>("/traj_info", 1);
        pub_initTraj = nh.advertise<visualization_msgs::MarkerArray>("/initTraj", 1);
        pub_obsBox = nh.advertise<visualization_msgs::MarkerArray>("/obstacle_box", 1);
        pub_feasibleBox = nh.advertise<visualization_msgs::MarkerArray>("/feasible_box", 1);
        pub_colBox = nh.advertise<visualization_msgs::MarkerArray>("/collision_model", 1);
        pub_minDist = nh.advertise<geometry_msgs::Pose>("/min_dist", 1);

        msgs_traj.resize(qn);
        msgs_relBox.resize(qn);
    }

    void update(double current_time){
        update_traj(current_time);
        update_initTraj();
        update_obsBox(current_time);
        update_relBox(current_time);
        update_feasibleBox(current_time);
        update_colBox();
        update_distance_between_agents_realtime(current_time);
    }

    void publish(){
        for(int qi = 0; qi < qn; qi++){
            pubs_traj_coef[qi].publish(RBPPlanner_obj.get()->msgs_traj_coef[qi]);
            pubs_traj[qi].publish(msgs_traj[qi]);
            pubs_relBox[qi].publish(msgs_relBox[qi]);
        }
        pub_traj_info.publish(RBPPlanner_obj.get()->msgs_traj_info);
        pub_initTraj.publish(msgs_initTraj);
        pub_obsBox.publish(msgs_obsBox);
        pub_feasibleBox.publish(msgs_feasibleBox);
        pub_colBox.publish(msgs_colBox);
        pub_minDist.publish(msgs_minDist);
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

//    void plot_real_time(){
//        plot_distance_between_agents_realtime();
//    }

private:
    ros::NodeHandle nh;
    std::shared_ptr<RBPPlanner> RBPPlanner_obj;
    std::shared_ptr<Corridor> corridor_obj;
    std::shared_ptr<InitTrajPlanner> initTrajPlanner_obj;
    SwarmPlanning::Mission mission;
    SwarmPlanning::Param param;

    int qn, M, outdim;
    double global_min_dist, dt;
    tf::TransformBroadcaster br;
    std::vector<Eigen::MatrixXd> pva;
    std::vector<Eigen::MatrixXd> coef;
    std::vector<std::vector<double>> currentState;
    std::vector<double> T, t, max_dist, min_dist, collision_dist;
    std::vector<std::vector<std::vector<double>>> quad_state;
//    std::shared_ptr<plt::Plot> plot_min_dist_obj;

    // ROS publisher
    ros::Publisher pub_traj_info;
    std::vector<ros::Publisher> pubs_traj_coef;
    std::vector<ros::Publisher> pubs_traj;
    ros::Publisher pub_initTraj;
    ros::Publisher pub_obsBox;
    std::vector<ros::Publisher> pubs_relBox;
    ros::Publisher pub_feasibleBox;
    ros::Publisher pub_colBox;
    ros::Publisher pub_minDist;

    // ROS messages
    std::vector<nav_msgs::Path> msgs_traj;
    visualization_msgs::MarkerArray msgs_initTraj;
    visualization_msgs::MarkerArray msgs_obsBox;
    std::vector<visualization_msgs::MarkerArray> msgs_relBox;
    visualization_msgs::MarkerArray msgs_feasibleBox;
    visualization_msgs::MarkerArray msgs_colBox;
    geometry_msgs::Pose msgs_minDist;

    void timeMatrix(double current_time, int& index, Eigen::MatrixXd& polyder){
        double tseg = 0;
        double tcand;

        // find segment start time tseg
        for(int m = 0; m < M; m++){
            tcand = T[m];
            if(tcand < current_time){
                tseg = tcand;
                index = m;
            } else {
                break;
            }
        }
        tseg = current_time-tseg;
        polyder.resize(3, param.n+1);
        for(int i = 0; i < 3; i++){
            for(int j = 0; j < param.n+1; j++){
                if(i <= j)
                    polyder(i, param.n-j) = ((i==0)*1+(i==1)*j+(i==2)*j*(j-1)) * pow(tseg,j-i);
                else
                    polyder(i, param.n-j) = 0;
            }
        }
    }

    void update_traj(double current_time){
        if (current_time > T.back()) {
            return;
        }
        for(int qi = 0; qi < qn; qi++) {
            int index = 0;
            Eigen::MatrixXd polyder;
            timeMatrix(current_time, index, polyder);

            pva[qi] = polyder * coef[qi].block((param.n + 1) * index, 0, (param.n + 1), 3);

            msgs_traj[qi].header.frame_id = "/world";
            msgs_traj[qi].header.stamp.sec = current_time;

            geometry_msgs::PoseStamped pos_des;
            pos_des.header.frame_id = "/world";
            pos_des.pose.position.x = pva[qi](0,0);
            pos_des.pose.position.y = pva[qi](0,1);
            pos_des.pose.position.z = pva[qi](0,2);
            msgs_traj[qi].poses.emplace_back(pos_des);

            currentState[qi][0] = pva[qi](0,0);
            currentState[qi][1] = pva[qi](0,1);
            currentState[qi][2] = pva[qi](0,2);
            currentState[qi][3] = pva[qi](1,0);
            currentState[qi][4] = pva[qi](1,1);
            currentState[qi][5] = pva[qi](1,2);
            currentState[qi][6] = pva[qi](2,0);
            currentState[qi][7] = pva[qi](2,1);
            currentState[qi][8] = pva[qi](2,2);

            tf::Transform transform;
            transform.setOrigin(tf::Vector3(pva[qi](0,0), pva[qi](0,1), pva[qi](0,2)));
            tf::Quaternion q;
            q.setRPY(0,0,0);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                    "world", "/mav"+std::to_string(qi)+"/base_link"));
        }
    }

    void update_initTraj(){
        visualization_msgs::MarkerArray mk_array;
        for (int qi = 0; qi < qn; qi++) {
            for (int m = 0; m < initTrajPlanner_obj->initTraj[qi].size(); m++) {
                visualization_msgs::Marker mk;
                mk.header.frame_id = "world";
                mk.header.stamp = ros::Time::now();
                mk.ns = "mav" + std::to_string(qi);
                mk.type = visualization_msgs::Marker::CUBE;
                mk.action = visualization_msgs::Marker::ADD;

                mk.pose.orientation.x = 0.0;
                mk.pose.orientation.y = 0.0;
                mk.pose.orientation.z = 0.0;
                mk.pose.orientation.w = 1.0;

                mk.color.a = 1.0;
                mk.color.r = param.color[qi][0];
                mk.color.g = param.color[qi][1];
                mk.color.b = param.color[qi][2];

                mk.id = m;
                octomap::point3d p_init = initTrajPlanner_obj->initTraj[qi][m];
                mk.pose.position.x = p_init.x();
                mk.pose.position.y = p_init.y();
                mk.pose.position.z = p_init.z();

                mk.scale.x = 0.1;
                mk.scale.y = 0.1;
                mk.scale.z = 0.1;

                mk_array.markers.emplace_back(mk);
            }
        }
        msgs_initTraj = mk_array;
    }

    void update_obsBox(double current_time){
        visualization_msgs::MarkerArray mk_array;
        for (int qi = 0; qi < qn; qi++){
            // find current obsBox number
            int box_curr = 0;
            while(box_curr < corridor_obj->SFC[qi].size() &&
                    corridor_obj->SFC[qi][box_curr].second < current_time){
                box_curr++;
            }
            if(box_curr >= corridor_obj->SFC[qi].size()){
                box_curr = corridor_obj->SFC[qi].size() - 1;
            }

            visualization_msgs::Marker mk;
            mk.header.frame_id = "world";
            mk.ns = "mav" + std::to_string(qi);
            mk.type = visualization_msgs::Marker::CUBE;
            mk.action = visualization_msgs::Marker::ADD;

            mk.pose.orientation.x = 0.0;
            mk.pose.orientation.y = 0.0;
            mk.pose.orientation.z = 0.0;
            mk.pose.orientation.w = 1.0;

            for (int bi = 0; bi < corridor_obj->SFC[qi].size(); bi++){
                mk.id = bi;
//                mk.header.stamp = ros::Time(obstacle_boxes[qi][bi].second);
                std::vector<double> obstacle_box = corridor_obj->SFC[qi][bi].first;

                {
                    double margin = mission.quad_size[qi];
                    obstacle_box[0] -= margin;
                    obstacle_box[1] -= margin;
                    obstacle_box[2] -= margin;
                    obstacle_box[3] += margin;
                    obstacle_box[4] += margin;
                    obstacle_box[5] += margin;
                }

                mk.pose.position.x = (obstacle_box[0]+obstacle_box[3])/2.0;
                mk.pose.position.y = (obstacle_box[1]+obstacle_box[4])/2.0;
                mk.pose.position.z = (obstacle_box[2]+obstacle_box[5])/2.0;

                mk.scale.x = obstacle_box[3]-obstacle_box[0];
                mk.scale.y = obstacle_box[4]-obstacle_box[1];
                mk.scale.z = obstacle_box[5]-obstacle_box[2];

                mk.color.a = 0.2;
                mk.color.r = param.color[qi][0];
                mk.color.g = param.color[qi][1];
                mk.color.b = param.color[qi][2];

                mk_array.markers.emplace_back(mk);
            }
        }
        msgs_obsBox = mk_array;
    }

    void update_relBox(double current_time)
    {
        for(int qi = 0; qi < qn; qi++){
            visualization_msgs::MarkerArray mk_array;
            for(int qj = qi + 1; qj < qn ; qj++){
                int box_curr = 0;
                while(box_curr < corridor_obj->RSFC[qi][qj].size() &&
                        corridor_obj->RSFC[qi][qj][box_curr].second < current_time){
                    box_curr++;
                }
                if(box_curr >= corridor_obj->RSFC[qi][qj].size()){
                    box_curr = corridor_obj->RSFC[qi][qj].size() - 1;
                }

                visualization_msgs::Marker mk;
                mk.header.frame_id = "world";
                mk.ns = "mav" + std::to_string(qj);

                // inter-collision model
                double r = mission.quad_size[qi] + mission.quad_size[qj];
                double h = r * param.downwash;

                mk.type = visualization_msgs::Marker::SPHERE;
                mk.action = visualization_msgs::Marker::ADD;
                mk.id = 0;

                mk.pose.orientation.x = 0;
                mk.pose.orientation.y = 0;
                mk.pose.orientation.z = 0;
                mk.pose.orientation.w = 1.0;

                mk.pose.position.x = pva[qi](0, 0);
                mk.pose.position.y = pva[qi](0, 1);
                mk.pose.position.z = pva[qi](0, 2);

                mk.scale.x = 2 * r;
                mk.scale.y = 2 * r;
                mk.scale.z = 2 * h;

                mk.color.a = 0.5;
                mk.color.r = 1.0;
                mk.color.g = 0.0;
                mk.color.b = 0.0;

                mk_array.markers.emplace_back(mk);

                // qj position
                mk.header.frame_id = "world";
                mk.type = visualization_msgs::Marker::SPHERE;
                mk.action = visualization_msgs::Marker::ADD;
                mk.id = 1;

                mk.pose.orientation.x = 0;
                mk.pose.orientation.y = 0;
                mk.pose.orientation.z = 0;
                mk.pose.orientation.w = 1.0;

                mk.pose.position.x = pva[qj](0, 0);
                mk.pose.position.y = pva[qj](0, 1);
                mk.pose.position.z = pva[qj](0, 2);

//                mk.scale.x = mission.quad_size[qj];
//                mk.scale.y = mission.quad_size[qj];
//                mk.scale.z = mission.quad_size[qj] * param.downwash;
                mk.scale.x = 0.1;
                mk.scale.y = 0.1;
                mk.scale.z = 0.1;

                mk.color.a = 1.0;
                mk.color.r = param.color[qj][0];
                mk.color.g = param.color[qj][1];
                mk.color.b = param.color[qj][2];

                mk_array.markers.emplace_back(mk);

                // relative box
                mk.header.frame_id = "world";
                mk.header.stamp = ros::Time::now();
                mk.type = visualization_msgs::Marker::CUBE;
                mk.action = visualization_msgs::Marker::ADD;
                mk.id = qj + 2;

                mk.color.a = 0.3;
                mk.color.r = 0.0;
                mk.color.g = 1.0;
                mk.color.b = 0.0;

                mk.scale.x = 40;
                mk.scale.y = 40;
                mk.scale.z = 40;

                octomap::point3d qi_vector;
                qi_vector.x() = pva[qi](0, 0);
                qi_vector.y() = pva[qi](0, 1);
                qi_vector.z() = pva[qi](0, 2);

                octomap::point3d normal_vector = corridor_obj->RSFC[qi][qj][box_curr].first;
                Eigen::Vector3d V3d_normal_vector(normal_vector.x(), normal_vector.y(), normal_vector.z());

                double distance = r / normal_vector.norm() + mk.scale.z / 2;
//                double distance = mission.quad_size[qi] / normal_vector.norm();

                mk.pose.position.x = pva[qi](0, 0) + normal_vector.normalized().x() * distance;
                mk.pose.position.y = pva[qi](0, 1) + normal_vector.normalized().y() * distance;
                mk.pose.position.z = pva[qi](0, 2) + normal_vector.normalized().z() * distance;

                Eigen::Vector3d z_0 = Eigen::Vector3d::UnitZ();
                Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z_0, V3d_normal_vector);

                mk.pose.orientation.x = q.x();
                mk.pose.orientation.y = q.y();
                mk.pose.orientation.z = q.z();
                mk.pose.orientation.w = q.w();

                mk_array.markers.emplace_back(mk);
            }
            msgs_relBox[qi] = mk_array;
        }
    }

    void update_feasibleBox(double current_time)
    {
        visualization_msgs::MarkerArray mk_array;
        for(int qj = 0; qj < qn; qj++){
            // current position
            visualization_msgs::Marker mk;
            mk.header.frame_id = "world";
            mk.header.stamp = ros::Time::now();
            mk.ns = "mav" + std::to_string(qj);
            mk.action = visualization_msgs::Marker::ADD;

            // inter-collision model
            for(int qi = 0; qi < qn; qi++) {
                mk.type = visualization_msgs::Marker::SPHERE;
                mk.id = qi;
                mk.pose.position.x = pva[qi](0, 0);
                mk.pose.position.y = pva[qi](0, 1);
                mk.pose.position.z = pva[qi](0, 2);

                mk.scale.x = 2 * mission.quad_size[qi];
                mk.scale.y = 2 * mission.quad_size[qi];
                mk.scale.z = 2 * mission.quad_size[qi] * param.downwash;

                mk.color.a = 0.5;
                mk.color.r = param.color[qi][0];
                mk.color.g = param.color[qi][1];
                mk.color.b = param.color[qi][2];

                mk_array.markers.emplace_back(mk);
            }

            // feasible box
            for(int qi = 0; qi < qn; qi++){
                mk.type = visualization_msgs::Marker::CUBE;
                mk.id = qn + qi;
                mk.color.a = 0.3;
                mk.color.r = param.color[qi][0];
                mk.color.g = param.color[qi][1];
                mk.color.b = param.color[qi][2];

                mk.scale.x = 40;
                mk.scale.y = 40;
                mk.scale.z = 40;

                double r = mission.quad_size[qi];
                double h = r * param.downwash;

                octomap::point3d normal_vector;
                int box_curr = 0;
                if(qi < qj) { // RSFC
                    while (box_curr < corridor_obj->RSFC[qi][qj].size() &&
                            corridor_obj->RSFC[qi][qj][box_curr].second < current_time) {
                        box_curr++;
                    }
                    if (box_curr >= corridor_obj->RSFC[qi][qj].size()) {
                        box_curr = corridor_obj->RSFC[qi][qj].size() - 1;
                    }

                    normal_vector = corridor_obj->RSFC[qi][qj][box_curr].first;
                }
                else if(qi > qj){ // RSFC
                    while (box_curr < corridor_obj->RSFC[qj][qi].size() &&
                            corridor_obj->RSFC[qj][qi][box_curr].second < current_time) {
                        box_curr++;
                    }
                    if (box_curr >= corridor_obj->RSFC[qj][qi].size()) {
                        box_curr = corridor_obj->RSFC[qj][qi].size() - 1;
                    }

                    normal_vector = -corridor_obj->RSFC[qj][qi][box_curr].first;
                }
                else{ // SFC
                    while(box_curr < corridor_obj->SFC[qi].size() &&
                            corridor_obj->SFC[qi][box_curr].second < current_time){
                        box_curr++;
                    }
                    if(box_curr >= corridor_obj->SFC[qi].size()){
                        box_curr = corridor_obj->SFC[qi].size() - 1;
                    }
                    std::vector<double> obstacle_box = corridor_obj->SFC[qi][box_curr].first;
                    for(int iter = 0; iter < 6; iter++){
                        double margin = mission.quad_size[qi];
                        if(iter == 2 || iter == 5)
                            margin *= param.downwash;
                        if(iter < 3)
                            obstacle_box[iter] = obstacle_box[iter] - margin;
                        else
                            obstacle_box[iter] = obstacle_box[iter] + margin;
                    }

                    mk.type = visualization_msgs::Marker::LINE_LIST;
                    mk.pose.position.x = 0;
                    mk.pose.position.y = 0;
                    mk.pose.position.z = 0;

                    mk.pose.orientation.x = 0;
                    mk.pose.orientation.y = 0;
                    mk.pose.orientation.z = 0;
                    mk.pose.orientation.w = 1.0;

                    mk.scale.x = 0.05;
//                    mk.scale.y = obstacle_box[4]-obstacle_box[1];
//                    mk.scale.z = obstacle_box[5]-obstacle_box[2];

                    mk.color.a = 1;
                    mk.color.r = param.color[qi][0];
                    mk.color.g = param.color[qi][1];
                    mk.color.b = param.color[qi][2];

                    geometry_msgs::Point pt_i, pt_j;
                    std::vector<int> index = {0,1,2,
                                              3,4,2,
                                              3,1,5,
                                              0,4,5};
                    int offset = 0;
                    for(int iter = 0; iter < 4; iter++) {
                        pt_i.x = obstacle_box[index[offset + 0]];
                        pt_i.y = obstacle_box[index[offset + 1]];
                        pt_i.z = obstacle_box[index[offset + 2]];

                        for (int i = 0; i < 3; i++) {
                            pt_j.x = obstacle_box[(index[offset + 0] + 3 * (i==0)) % 6];
                            pt_j.y = obstacle_box[(index[offset + 1] + 3 * (i==1)) % 6];
                            pt_j.z = obstacle_box[(index[offset + 2] + 3 * (i==2)) % 6];

                            mk.points.emplace_back(pt_i);
                            mk.points.emplace_back(pt_j);
                        }
                        offset += 3;
                    }
                    mk_array.markers.emplace_back(mk);
                    mk.points.clear();

                    continue;
                }

                double distance = r / normal_vector.norm() - mk.scale.z / 2;
                mk.pose.position.x = pva[qi](0, 0) + normal_vector.normalized().x() * distance;
                mk.pose.position.y = pva[qi](0, 1) + normal_vector.normalized().y() * distance;
                mk.pose.position.z = pva[qi](0, 2) + normal_vector.normalized().z() * distance;

                Eigen::Vector3d V3d_normal_vector(normal_vector.x(), normal_vector.y(), normal_vector.z());
                Eigen::Vector3d z_0 = Eigen::Vector3d::UnitZ();
                Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z_0, V3d_normal_vector);

                mk.pose.orientation.x = q.x();
                mk.pose.orientation.y = q.y();
                mk.pose.orientation.z = q.z();
                mk.pose.orientation.w = q.w();

                mk_array.markers.emplace_back(mk);
            }
        }
        msgs_feasibleBox = mk_array;
    }

    // obstacle-collision model
    void update_colBox(){
        visualization_msgs::MarkerArray mk_array;
        for (int qi = 0; qi < qn; qi++) {
            visualization_msgs::Marker mk;
            mk.header.frame_id = "world";
            mk.ns = "colBox";
            mk.type = visualization_msgs::Marker::SPHERE;
            mk.action = visualization_msgs::Marker::ADD;

            mk.pose.orientation.x = 0;
            mk.pose.orientation.y = 0;
            mk.pose.orientation.z = 0;
            mk.pose.orientation.w = 1.0;

            mk.id = qi;
            mk.pose.position.x = pva[qi](0, 0);
            mk.pose.position.y = pva[qi](0, 1);
            mk.pose.position.z = pva[qi](0, 2);

            mk.scale.x = 2 * mission.quad_size[qi];
            mk.scale.y = 2 * mission.quad_size[qi];
            mk.scale.z = 2 * mission.quad_size[qi] * param.downwash;

            mk.color.a = 0.7;
            mk.color.r = param.color[qi][0];
            mk.color.g = param.color[qi][1];
            mk.color.b = param.color[qi][2];

//            if(param.gn == 0 && qi < param.group){
//                mk.color.a = 0.7;
//                mk.color.r = 0;
//                mk.color.g = 0;
//                mk.color.b = 0;
//            }
//            else if(param.gn == 1 && qi >= param.group){
//                mk.color.a = 0.7;
//                mk.color.r = 0;
//                mk.color.g = 0;
//                mk.color.b = 0;
//            }
//            else{
////                mk.color.a = 0;
//            }

            mk_array.markers.emplace_back(mk);
        }
        msgs_colBox = mk_array;
    }

    void update_quad_state() {
        for (int qi = 0; qi < qn; qi++) {
            for (int j = 0; j < t.size(); j++) {
                int index = 0;
                Eigen::MatrixXd state, polyder;
                timeMatrix(t[j], index, polyder);
                state = polyder * coef[qi].block((param.n + 1) * index, 0, (param.n + 1), 3);

                for (int i = 0; i < outdim * param.phi; i++) {
                    quad_state[qi][i][j] = state(i / outdim, i % outdim);
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
        plt::figure_size(1280, 960);

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
        plt::figure_size(1280, 720);

        std::vector<double> collision_dist;
        collision_dist.resize(t.size());
        for(int i = 0; i < t.size(); i++){
            collision_dist[i] = 2 * mission.quad_size[0]; //TODO: heterogeous case
        }

        plt::plot(t, collision_dist);
//        plt::plot(t, max_dist);
        plt::plot(t, min_dist);

        plt::title("Ellipsoidal Distance between Quadrotor");

        plt::show(false);
    }

//    void plot_distance_between_agents_realtime(){
//        collision_dist.resize(t.size());
//        for(int i = 0; i < t.size(); i++){
//            collision_dist[i] = 2 * mission.quad_size[0]; //TODO: heterogeous case
//        }
//
//        plt::figure(2);
//        plt::title("Ellipsoidal Distance between Quadrotor");
//        plt::figure_size(1280, 960);
//        plt::ylim(0, 5);
//        plt::named_plot("collision constraint", t, collision_dist);
//        plot_min_dist_obj.reset(new plt::Plot());
//        plt::axis("equal");
//        plt::legend();
//
//        for(int index = 0; index < t.size(); index++) {
//            std::vector<double> t_partial(t.begin(), t.begin() + index);
//            std::vector<double> min_dist_partial(min_dist.begin(), min_dist.begin() + index);
//            plot_min_dist_obj.get()->update(t_partial, min_dist_partial);
//            plt::pause(0.001);
//        }
//    }

    void update_distance_between_agents_realtime(double current_time){
        int index = floor(current_time / dt);
        double current_min_dist, alpha;

        if(index >= t.size()){
            index = t.size() - 1;
            current_min_dist = min_dist[index];
        }
        else{
            alpha = (current_time - index * dt) / dt;
            current_min_dist = (1-alpha) * min_dist[index] + alpha * min_dist[index];
        }

        msgs_minDist.position.x = current_min_dist;
        msgs_minDist.position.y = 2 * mission.quad_size[0];
    }
};