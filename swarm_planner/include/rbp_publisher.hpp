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

// Submodules
#include <mission.hpp>
#include <param.hpp>

namespace SwarmPlanning {
    class RBPPublisher {
    public:
        RBPPublisher(ros::NodeHandle _nh,
                     SwarmPlanning::PlanResult _planResult,
                     SwarmPlanning::Mission _mission,
                     SwarmPlanning::Param _param)
                : nh(std::move(_nh)),
                  planResult(std::move(_planResult)),
                  mission(std::move(_mission)),
                  param(std::move(_param)) {
            qn = mission.qn;
            outdim = 3;

            T = planResult.T;
            M.resize(qn);
            for(int qi = 0; qi < qn; qi++){
                M[qi] = T[qi].size() - 1;
            }

            dt = 0.1;
            t.resize(floor(T[0].back() / dt)); // All T[qi].back = makespan
            for (int i = 0; i < t.size(); i++) {
                t[i] = i * dt;
            }

            coef.resize(qn);
            quad_state.resize(qn);
            currentState.resize(qn);
            for (int qi = 0; qi < qn; qi++) {
                if(planResult.state >= OPTIMIZATION) {
                    coef[qi] = planResult.coef[qi];
                }
                currentState[qi].resize(outdim * param.phi);
                quad_state[qi].resize(outdim * param.phi);
                for (int i = 0; i < outdim * param.phi; i++) {
                    quad_state[qi][i].resize(t.size());
                }
            }
            if(planResult.state >= OPTIMIZATION) {
                generateROSMsg();
            }

            pubs_traj_coef.resize(qn);
            pubs_initTraj_vis.resize(qn);
            pubs_traj.resize(qn);
            pubs_relBox.resize(qn);
            for (int qi = 0; qi < qn; qi++) {
                std::string mav_name = "/mav" + std::to_string(qi);
                pubs_traj_coef[qi] = nh.advertise<std_msgs::Float64MultiArray>("/traj_coef" + mav_name, 1);
                pubs_traj[qi] = nh.advertise<nav_msgs::Path>("/desired_trajectory" + mav_name, 1);
                pubs_initTraj_vis[qi] = nh.advertise<nav_msgs::Path>("/initTraj" + mav_name, 1);
                pubs_relBox[qi] = nh.advertise<visualization_msgs::MarkerArray>("/relative_box" + mav_name, 1);
            }
//            pub_traj_info = nh.advertise<std_msgs::Float64MultiArray>("/traj_info", 1);
            pub_initTraj = nh.advertise<visualization_msgs::MarkerArray>("/initTraj", 1);
            pub_obsBox = nh.advertise<visualization_msgs::MarkerArray>("/obstacle_box", 1);
            pub_feasibleBox = nh.advertise<visualization_msgs::MarkerArray>("/feasible_box", 1);
            pub_colBox = nh.advertise<visualization_msgs::MarkerArray>("/collision_model", 1);
//            pub_minDist = nh.advertise<geometry_msgs::Pose>("/min_dist", 1);

            msgs_traj.resize(qn);
            msgs_relBox.resize(qn);
        }

        void update(double current_time) {
            if(planResult.state >= INITTRAJ){
                update_initTraj(current_time);
                update_colBox();
            }
            if(planResult.state >= SFC){
                update_obsBox(current_time);
            }
            if(planResult.state >= RSFC){
                update_relBox(current_time);
            }
            if(planResult.state >= OPTIMIZATION) {
                update_traj(current_time);
                update_feasibleBox(current_time);
            }
        }

        void publish() {
            if(planResult.state >= INITTRAJ){
                pub_colBox.publish(msgs_colBox);
                pub_initTraj.publish(msgs_initTraj);
                for (int qi = 0; qi < qn; qi++) {
                    pubs_initTraj_vis[qi].publish(msgs_initTraj_vis[qi]);
                }
            }
            if(planResult.state >= SFC){
                pub_obsBox.publish(msgs_obsBox);
            }
            if(planResult.state >= RSFC){
                for (int qi = 0; qi < qn; qi++) {
                    pubs_relBox[qi].publish(msgs_relBox[qi]);
                }
            }

            if(planResult.state >= OPTIMIZATION) {
                for (int qi = 0; qi < qn; qi++) {
                    pubs_traj_coef[qi].publish(msgs_traj_coef[qi]);
                    pubs_traj[qi].publish(msgs_traj[qi]);
                }
                pub_feasibleBox.publish(msgs_feasibleBox);
            }
//            pub_traj_info.publish(planResult.msgs_traj_info);
//            pub_minDist.publish(msgs_minDist);
        }

        void plot(bool log) {
            update_quad_state();
            update_safety_margin_ratio();

            if(log) {
                plot_safety_margin_ratio();
                if(planResult.state >= OPTIMIZATION) {
                    plot_quad_dynamics();
                }
            }

            ROS_INFO_STREAM("Global minimum safety margin ratio: " << safety_margin_ratio);
            ROS_INFO_STREAM("Total flight distance: " << trajectory_length_sum());
        }

    private:
        ros::NodeHandle nh;
        SwarmPlanning::PlanResult planResult;
        SwarmPlanning::Mission mission;
        SwarmPlanning::Param param;

        T_t T;
        int qn, outdim;
        double safety_margin_ratio, dt;
        tf::TransformBroadcaster br;
        std::vector<int> M;
        std::vector<Eigen::MatrixXd> coef;
        std::vector<std::vector<double>> currentState;
        std::vector<double> t, max_ratio, min_ratio;
        std::vector<std::vector<std::vector<double>>> quad_state;
//    std::shared_ptr<plt::Plot> plot_min_dist_obj;

        // ROS publisher
//        ros::Publisher pub_traj_info;
        std::vector<ros::Publisher> pubs_traj_coef;
        std::vector<ros::Publisher> pubs_traj;
        std::vector<ros::Publisher> pubs_initTraj_vis;
        ros::Publisher pub_initTraj;
        ros::Publisher pub_obsBox;
        std::vector<ros::Publisher> pubs_relBox;
        ros::Publisher pub_feasibleBox;
        ros::Publisher pub_colBox;
//        ros::Publisher pub_minDist;

        // ROS messages
        std::vector<std_msgs::Float64MultiArray> msgs_traj_coef;
        std::vector<nav_msgs::Path> msgs_traj;
        std::vector<nav_msgs::Path> msgs_initTraj_vis;
        visualization_msgs::MarkerArray msgs_initTraj;
        visualization_msgs::MarkerArray msgs_obsBox;
        std::vector<visualization_msgs::MarkerArray> msgs_relBox;
        visualization_msgs::MarkerArray msgs_feasibleBox;
        visualization_msgs::MarkerArray msgs_colBox;
//        geometry_msgs::Pose msgs_minDist;

        // generate ros message to transfer planning result
        void generateROSMsg() {
            msgs_traj_coef.resize(qn);
            for (int qi = 0; qi < qn; qi++) {
                std_msgs::MultiArrayDimension rows;
                rows.size = M[qi] * (param.n + 1);
                msgs_traj_coef[qi].layout.dim.emplace_back(rows);

                std_msgs::MultiArrayDimension cols;
                cols.size = outdim;
                msgs_traj_coef[qi].layout.dim.emplace_back(cols);

                std::vector<double> coef_temp(planResult.coef[qi].data(),
                                              planResult.coef[qi].data() + planResult.coef[qi].size());
                msgs_traj_coef[qi].data.insert(msgs_traj_coef[qi].data.end(), coef_temp.begin(), coef_temp.end());
            }
        }

        void update_traj(double current_time) {
            for (int qi = 0; qi < qn; qi++) {
                if (current_time > T[qi].back()) {
                    continue;
                }
                Eigen::MatrixXd currentState_matrix = planResult.currentState(param, qi, current_time);
                currentState[qi][0] = currentState_matrix(0, 0); //x
                currentState[qi][1] = currentState_matrix(0, 1); //y
                currentState[qi][2] = currentState_matrix(0, 2); //z
                currentState[qi][3] = currentState_matrix(1, 0); //vx
                currentState[qi][4] = currentState_matrix(1, 1); //vy
                currentState[qi][5] = currentState_matrix(1, 2); //vz
                currentState[qi][6] = currentState_matrix(2, 0); //ax
                currentState[qi][7] = currentState_matrix(2, 1); //ay
                currentState[qi][8] = currentState_matrix(2, 2); //az

                msgs_traj[qi].header.frame_id = "/world";
                msgs_traj[qi].header.stamp.sec = current_time;
                geometry_msgs::PoseStamped pos_des;
                pos_des.header.frame_id = "/world";
                pos_des.pose.position.x = currentState[qi][0];
                pos_des.pose.position.y = currentState[qi][1];
                pos_des.pose.position.z = currentState[qi][2];
                msgs_traj[qi].poses.emplace_back(pos_des);

                tf::Transform transform;
                transform.setOrigin(tf::Vector3(currentState[qi][0], currentState[qi][1], currentState[qi][2]));
                tf::Quaternion q;
                q.setRPY(0, 0, atan2(currentState[qi][4], currentState[qi][3]));
                transform.setRotation(q);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                 "world", "/mav" + std::to_string(qi) + "/base_link"));
            }
        }

        void update_initTraj(double current_time) {
            //update discrete initial trajectory
            visualization_msgs::MarkerArray mk_array;
            for (int qi = 0; qi < qn; qi++) {
                for (int m = 0; m < planResult.initTraj[qi].size(); m++) {
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
                    octomap::point3d p_init = planResult.initTraj[qi][m];
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

            //update initial trajectory with linear interpolation
            double t_, t_seg, alpha;
            if (current_time > T[0].back()) {
                return;
            }
            msgs_initTraj_vis.resize(qn);
            for (int qi = 0; qi < qn; qi++) {
                msgs_initTraj_vis[qi].header.frame_id = "/world";
                msgs_initTraj_vis[qi].header.stamp.sec = current_time;

                int m = planResult.findSegmentIdx(qi, current_time);
                t_ = current_time - T[qi][m];
                t_seg = T[qi][m + 1] - T[qi][m];
                alpha = t_/t_seg;

                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "/world";
                pose.pose.position.x = (1-alpha) * planResult.initTraj[qi][m].x() + alpha * planResult.initTraj[qi][m+1].x();
                pose.pose.position.y = (1-alpha) * planResult.initTraj[qi][m].y() + alpha * planResult.initTraj[qi][m+1].y();
                pose.pose.position.z = (1-alpha) * planResult.initTraj[qi][m].z() + alpha * planResult.initTraj[qi][m+1].z();
                msgs_initTraj_vis[qi].poses.emplace_back(pose);

                if(planResult.state < OPTIMIZATION) {
                    currentState[qi][0] = pose.pose.position.x;
                    currentState[qi][1] = pose.pose.position.y;
                    currentState[qi][2] = pose.pose.position.z;
                }
            }

        }

        void update_obsBox(double current_time) {
            visualization_msgs::MarkerArray mk_array;
            for (int qi = 0; qi < qn; qi++) {
                // find current obsBox number
                int box_curr = 0;
                while (box_curr < planResult.SFC[qi].size() &&
                        planResult.SFC[qi][box_curr].end_time < current_time) {
                    box_curr++;
                }
                if (box_curr >= planResult.SFC[qi].size()) {
                    box_curr = planResult.SFC[qi].size() - 1;
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

                for (int bi = 0; bi < planResult.SFC[qi].size(); bi++) {
                    mk.id = bi;
//                mk.header.stamp = ros::Time(obstacle_boxes[qi][bi].second);
                    std::vector<double> obstacle_box = planResult.SFC[qi][bi].box;

                    {
                        double margin = mission.quad_collision_model[qi][qi].r;
                        obstacle_box[0] -= margin;
                        obstacle_box[1] -= margin;
                        obstacle_box[2] -= margin;
                        obstacle_box[3] += margin;
                        obstacle_box[4] += margin;
                        obstacle_box[5] += margin;
                    }

                    mk.pose.position.x = (obstacle_box[0] + obstacle_box[3]) / 2.0;
                    mk.pose.position.y = (obstacle_box[1] + obstacle_box[4]) / 2.0;
                    mk.pose.position.z = (obstacle_box[2] + obstacle_box[5]) / 2.0;

                    mk.scale.x = obstacle_box[3] - obstacle_box[0];
                    mk.scale.y = obstacle_box[4] - obstacle_box[1];
                    mk.scale.z = obstacle_box[5] - obstacle_box[2];

                    mk.color.a = 0.2;
                    mk.color.r = param.color[qi][0];
                    mk.color.g = param.color[qi][1];
                    mk.color.b = param.color[qi][2];

                    mk_array.markers.emplace_back(mk);
                }
            }
            msgs_obsBox = mk_array;
        }

        void update_relBox(double current_time) {
            for (int qi = 0; qi < qn; qi++) {
                visualization_msgs::MarkerArray mk_array;
                for (int qj = qi + 1; qj < qn; qj++) {
                    int box_curr = 0;
                    while (box_curr < planResult.RSFC[qi][qj].size() &&
                           planResult.RSFC[qi][qj][box_curr].end_time < current_time) {
                        box_curr++;
                    }
                    if (box_curr >= planResult.RSFC[qi][qj].size()) {
                        box_curr = planResult.RSFC[qi][qj].size() - 1;
                    }

                    visualization_msgs::Marker mk;
                    mk.header.frame_id = "world";
                    mk.ns = "mav" + std::to_string(qj);

                    // inter-collision model
                    double r = mission.quad_collision_model[qi][qj].r;
                    double a = mission.quad_collision_model[qi][qj].a;
                    double b = mission.quad_collision_model[qi][qj].b;

                    mk.type = visualization_msgs::Marker::CUBE;
                    mk.action = visualization_msgs::Marker::ADD;
                    mk.id = 0;

                    mk.pose.orientation.x = 0;
                    mk.pose.orientation.y = 0;
                    mk.pose.orientation.z = 0;
                    mk.pose.orientation.w = 1.0;

                    mk.pose.position.x = currentState[qi][0];
                    mk.pose.position.y = currentState[qi][1];
                    mk.pose.position.z = currentState[qi][2] + (a - b)/2;

                    mk.scale.x = 2 * r;
                    mk.scale.y = 2 * r;
                    mk.scale.z = a + b;

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

                    mk.pose.position.x = currentState[qj][0];
                    mk.pose.position.y = currentState[qj][1];
                    mk.pose.position.z = currentState[qj][2];

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

                    double box_scale = 40;
                    mk.scale.x = box_scale;
                    mk.scale.y = box_scale;
                    mk.scale.z = box_scale;

                    octomap::point3d qi_vector;
                    qi_vector.x() = currentState[qi][0];
                    qi_vector.y() = currentState[qi][1];
                    qi_vector.z() = currentState[qi][2];

                    octomap::point3d normal_vector = planResult.RSFC[qi][qj][box_curr].normal_vector;
                    double distance = planResult.RSFC[qi][qj][box_curr].d + box_scale / 2;
                    Eigen::Vector3d V3d_normal_vector(normal_vector.x(), normal_vector.y(), normal_vector.z());

//                    double distance = r / normal_vector.norm() + mk.scale.z / 2;
//                double distance = mission.quad_size[qi] / normal_vector.norm();

                    mk.pose.position.x = currentState[qi][0] + normal_vector.normalized().x() * distance;
                    mk.pose.position.y = currentState[qi][1] + normal_vector.normalized().y() * distance;
                    mk.pose.position.z = currentState[qi][2] + normal_vector.normalized().z() * distance;

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

        void update_feasibleBox(double current_time) {
            visualization_msgs::MarkerArray mk_array;
            for (int qj = 0; qj < qn; qj++) {
                // current position
                visualization_msgs::Marker mk;
                mk.header.frame_id = "world";
                mk.header.stamp = ros::Time::now();
                mk.ns = "mav" + std::to_string(qj);
                mk.action = visualization_msgs::Marker::ADD;

                // inter-collision model
                for (int qi = 0; qi < qn; qi++) {
                    mk.id = qi;

                    double r, a, b;
                    r = mission.quad_collision_model[qi][qj].r;
                    a = mission.quad_collision_model[qi][qj].a;
                    b = mission.quad_collision_model[qi][qj].b;
                    if(qi == qj){
                        mk.type = visualization_msgs::Marker::SPHERE;
                        mk.pose.position.x = currentState[qi][0];
                        mk.pose.position.y = currentState[qi][1];
                        mk.pose.position.z = currentState[qi][2];
                        mk.scale.x = 0.1;
                        mk.scale.y = 0.1;
                        mk.scale.z = 0.1;
                    }
                    else {
                        mk.type = visualization_msgs::Marker::CUBE;
                        mk.pose.position.x = currentState[qi][0];
                        mk.pose.position.y = currentState[qi][1];
                        mk.pose.position.z = currentState[qi][2] + (a - b) / 2;
                        mk.scale.x = 2 * r;
                        mk.scale.y = 2 * r;
                        mk.scale.z = a + b;
                    }
                    mk.color.a = 0.5;
                    mk.color.r = param.color[qi][0];
                    mk.color.g = param.color[qi][1];
                    mk.color.b = param.color[qi][2];

                    mk_array.markers.emplace_back(mk);
                }

                // feasible box
                for (int qi = 0; qi < qn; qi++) {
                    mk.type = visualization_msgs::Marker::CUBE;
                    mk.id = qn + qi;
                    mk.color.a = 0.3;
                    mk.color.r = param.color[qi][0];
                    mk.color.g = param.color[qi][1];
                    mk.color.b = param.color[qi][2];

                    double box_scale = 40;
                    mk.scale.x = box_scale;
                    mk.scale.y = box_scale;
                    mk.scale.z = box_scale;

                    octomap::point3d normal_vector;
                    double distance;
                    int box_curr = 0;
                    if (qi < qj) { // RSFC
                        box_curr = planResult.findRSFCIdx(qi, qj, current_time);
                        normal_vector = planResult.RSFC[qi][qj][box_curr].normal_vector;
                        distance = planResult.RSFC[qi][qj][box_curr].d - box_scale / 2;
                    } else if (qi > qj) { // RSFC
                        box_curr = planResult.findRSFCIdx(qj, qi, current_time);
                        normal_vector = -planResult.RSFC[qj][qi][box_curr].normal_vector;
                        distance = planResult.RSFC[qj][qi][box_curr].d - box_scale / 2;
                    } else { // SFC
                        box_curr = planResult.findSFCIdx(qi, current_time);
                        std::vector<double> obstacle_box = planResult.SFC[qi][box_curr].box;
                        
                        mk.type = visualization_msgs::Marker::LINE_LIST;
                        mk.pose.position.x = 0;
                        mk.pose.position.y = 0;
                        mk.pose.position.z = 0;

                        mk.pose.orientation.x = 0;
                        mk.pose.orientation.y = 0;
                        mk.pose.orientation.z = 0;
                        mk.pose.orientation.w = 1.0;

                        mk.scale.x = 0.05;

                        mk.color.a = 1;
                        mk.color.r = param.color[qi][0];
                        mk.color.g = param.color[qi][1];
                        mk.color.b = param.color[qi][2];

                        geometry_msgs::Point pt_i, pt_j;
                        std::vector<int> index = {0, 1, 2,
                                                  3, 4, 2,
                                                  3, 1, 5,
                                                  0, 4, 5};
                        int offset = 0;
                        for (int iter = 0; iter < 4; iter++) {
                            pt_i.x = obstacle_box[index[offset + 0]];
                            pt_i.y = obstacle_box[index[offset + 1]];
                            pt_i.z = obstacle_box[index[offset + 2]];

                            for (int i = 0; i < 3; i++) {
                                pt_j.x = obstacle_box[(index[offset + 0] + 3 * (i == 0)) % 6];
                                pt_j.y = obstacle_box[(index[offset + 1] + 3 * (i == 1)) % 6];
                                pt_j.z = obstacle_box[(index[offset + 2] + 3 * (i == 2)) % 6];

                                mk.points.emplace_back(pt_i);
                                mk.points.emplace_back(pt_j);
                            }
                            offset += 3;
                        }
                        mk_array.markers.emplace_back(mk);
                        mk.points.clear();

                        continue;
                    }

                    mk.pose.position.x = currentState[qi][0] + normal_vector.x() * distance;
                    mk.pose.position.y = currentState[qi][1] + normal_vector.y() * distance;
                    mk.pose.position.z = currentState[qi][2] + normal_vector.z() * distance;

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

        void update_colBox() {
            visualization_msgs::MarkerArray mk_array;
            double r, a, b;
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
                mk.pose.position.x = currentState[qi][0];
                mk.pose.position.y = currentState[qi][1];
                mk.pose.position.z = currentState[qi][2];

                r = mission.quad_collision_model[qi][qi].r;
                mk.scale.x = 2 * r;
                mk.scale.y = 2 * r;
//                mk.scale.z = 2 * r * param.downwash;
                mk.scale.z = 2 * r;

                mk.color.a = 0.7;
                mk.color.r = param.color[qi][0];
                mk.color.g = param.color[qi][1];
                mk.color.b = param.color[qi][2];

                mk_array.markers.emplace_back(mk);

                //relBox
                mk.header.frame_id = "world";
                mk.ns = "relBox_mav" + std::to_string(qi);
                mk.action = visualization_msgs::Marker::ADD;

                mk.pose.orientation.x = 0;
                mk.pose.orientation.y = 0;
                mk.pose.orientation.z = 0;
                mk.pose.orientation.w = 1.0;

                for(int qj = 0; qj < qn; qj++) {
                    mk.id = qn * qi + qj;

                    if(qi == qj){
                        mk.type = visualization_msgs::Marker::SPHERE;
                        mk.pose.position.x = currentState[qj][0];
                        mk.pose.position.y = currentState[qj][1];
                        mk.pose.position.z = currentState[qj][2];

                        mk.scale.x = 0.1;
                        mk.scale.y = 0.1;
                        mk.scale.z = 0.1;
                    }
                    else {
                        mk.type = visualization_msgs::Marker::CUBE;
                        r = mission.quad_collision_model[qi][qj].r;
                        a = mission.quad_collision_model[qi][qj].a;
                        b = mission.quad_collision_model[qi][qj].b;

                        mk.pose.position.x = currentState[qj][0];
                        mk.pose.position.y = currentState[qj][1];
                        mk.pose.position.z = currentState[qj][2] + (a - b) / 2;

                        mk.scale.x = 2 * r;
                        mk.scale.y = 2 * r;
                        mk.scale.z = a + b;
                    }

                    mk.color.a = 0.7;
                    mk.color.r = param.color[qj][0];
                    mk.color.g = param.color[qj][1];
                    mk.color.b = param.color[qj][2];

                    mk_array.markers.emplace_back(mk);
                }
            }
            msgs_colBox = mk_array;
        }

        void update_quad_state() {
            for (int qi = 0; qi < qn; qi++) {
                for (int j = 0; j < t.size(); j++) {
                    if(planResult.state < OPTIMIZATION){
                        octomap::point3d p = planResult.currentPosition(param, qi, t[j]);
                        quad_state[qi][0][j] = p.x();
                        quad_state[qi][1][j] = p.y();
                        quad_state[qi][2][j] = p.z();
                    }
                    else {
                        Eigen::MatrixXd state = planResult.currentState(param, qi, t[j]);
                        for (int i = 0; i < outdim * param.phi; i++) {
                            quad_state[qi][i][j] = state(i / outdim, i % outdim);
                        }
                    }
                }
            }
        }

        double trajectory_length_sum() {
            double length_sum = 0;
            for (int qi = 0; qi < qn; qi++) {
                for (int i = 0; i < t.size() - 1; i++) {
                    length_sum += sqrt(pow(quad_state[qi][0][i + 1] - quad_state[qi][0][i], 2) +
                                       pow(quad_state[qi][1][i + 1] - quad_state[qi][1][i], 2) +
                                       pow(quad_state[qi][2][i + 1] - quad_state[qi][2][i], 2));
                }
            }
            return length_sum;
        }

        void plot_quad_dynamics() {
            plt::figure_size(1280, 960);

            std::vector<double> dynamic_limit;
            dynamic_limit.resize(t.size());

            // Plot Quad Velocity
            plt::subplot(3, 2, 1);
            for (int qi = 0; qi < qn; qi++) {
                plt::named_plot("agent" + std::to_string(qi), t, quad_state[qi][3]);
            }
            std::fill(dynamic_limit.begin(), dynamic_limit.end(), mission.max_vel[0][0]); //TODO: heterogeneous case
            plt::plot(t, dynamic_limit, "r--");
            std::fill(dynamic_limit.begin(), dynamic_limit.end(), -mission.max_vel[0][0]); //TODO: heterogeneous case
            plt::plot(t, dynamic_limit, "r--");
            plt::title("velocity -x axis");

            plt::subplot(3, 2, 3);
            for (int qi = 0; qi < qn; qi++) {
                plt::named_plot("agent" + std::to_string(qi), t, quad_state[qi][4]);
            }
            std::fill(dynamic_limit.begin(), dynamic_limit.end(), mission.max_vel[0][1]); //TODO: heterogeneous case
            plt::plot(t, dynamic_limit, "r--");
            std::fill(dynamic_limit.begin(), dynamic_limit.end(), -mission.max_vel[0][1]); //TODO: heterogeneous case
            plt::plot(t, dynamic_limit, "r--");
            plt::title("velocity -y axis");

            plt::subplot(3, 2, 5);
            for (int qi = 0; qi < qn; qi++) {
                plt::named_plot("agent" + std::to_string(qi), t, quad_state[qi][5]);
            }
            std::fill(dynamic_limit.begin(), dynamic_limit.end(), mission.max_vel[0][2]); //TODO: heterogeneous case
            plt::plot(t, dynamic_limit, "r--");
            std::fill(dynamic_limit.begin(), dynamic_limit.end(), -mission.max_vel[0][2]); //TODO: heterogeneous case
            plt::plot(t, dynamic_limit, "r--");
            plt::title("velocity -z axis");

            // Plot Quad Acceleration
            plt::subplot(3, 2, 2);
            for (int qi = 0; qi < qn; qi++) {
                plt::named_plot("agent" + std::to_string(qi), t, quad_state[qi][6]);
            }
            std::fill(dynamic_limit.begin(), dynamic_limit.end(), mission.max_acc[0][0]); //TODO: heterogeneous case
            plt::plot(t, dynamic_limit, "r--");
            std::fill(dynamic_limit.begin(), dynamic_limit.end(), -mission.max_acc[0][0]); //TODO: heterogeneous case
            plt::plot(t, dynamic_limit, "r--");
            plt::title("acceleration -x axis");

            plt::subplot(3, 2, 4);
            for (int qi = 0; qi < qn; qi++) {
                plt::named_plot("agent" + std::to_string(qi), t, quad_state[qi][7]);
            }
            std::fill(dynamic_limit.begin(), dynamic_limit.end(), mission.max_acc[0][1]); //TODO: heterogeneous case
            plt::plot(t, dynamic_limit, "r--");
            std::fill(dynamic_limit.begin(), dynamic_limit.end(), -mission.max_acc[0][1]); //TODO: heterogeneous case
            plt::plot(t, dynamic_limit, "r--");
            plt::title("acceleration -y axis");

            plt::subplot(3, 2, 6);
            for (int qi = 0; qi < qn; qi++) {
                plt::named_plot("agent" + std::to_string(qi), t, quad_state[qi][8]);
            }
            std::fill(dynamic_limit.begin(), dynamic_limit.end(), mission.max_acc[0][2]); //TODO: heterogeneous case
            plt::plot(t, dynamic_limit, "r--");
            std::fill(dynamic_limit.begin(), dynamic_limit.end(), -mission.max_acc[0][2]); //TODO: heterogeneous case
            plt::plot(t, dynamic_limit, "r--");
            plt::title("acceleration -z axis");

            plt::legend();
            plt::show(false);
        }

        void update_safety_margin_ratio() {
            double max_ratio_, min_ratio_, ratio, r, a, b, boxDist;
            max_ratio.resize(t.size());
            min_ratio.resize(t.size());

            safety_margin_ratio = SP_INFINITY;
            for (int i = 0; i < t.size(); i++) {
                max_ratio_ = 0;
                min_ratio_ = SP_INFINITY;
                for (int qi = 0; qi < qn; qi++) {
                    for (int qj = qi + 1; qj < qn; qj++) {
                        r = mission.quad_collision_model[qi][qj].r;
                        a = mission.quad_collision_model[qi][qj].a;
                        b = mission.quad_collision_model[qi][qj].b;

                        boxDist = std::max({std::abs(quad_state[qi][0][i] - quad_state[qj][0][i]),
                                            std::abs(quad_state[qi][1][i] - quad_state[qj][1][i]),
                                            std::abs(quad_state[qi][2][i] - quad_state[qj][2][i] + (b-a)/2) * 2 * r / (a + b)});

                        ratio = boxDist / r;

                        if (ratio > max_ratio_) {
                            max_ratio_ = ratio;
                        }
                        if (ratio < min_ratio_) {
                            min_ratio_ = ratio;
                        }
                        if (ratio < safety_margin_ratio) {
                            safety_margin_ratio = ratio;
                        }
                    }
                }
                max_ratio[i] = max_ratio_;
                min_ratio[i] = min_ratio_;
            }
        }

        void plot_safety_margin_ratio() {
            plt::figure(1);
            plt::figure_size(480, 270);

            double maximum_of_min_ratio = 0;
            std::vector<double> collision_ratio;
            collision_ratio.resize(t.size());
            for (int i = 0; i < t.size(); i++) {
                if(min_ratio[i] > maximum_of_min_ratio){
                    maximum_of_min_ratio = min_ratio[i];
                }
                collision_ratio[i] = 1;
            }

            plt::plot(t, collision_ratio, "r--");
//            plt::plot(t, max_ratio);
            plt::plot(t, min_ratio);

            plt::ylim<double>(0, maximum_of_min_ratio + 1);

            plt::title("Safety margin ratio between Quadrotors");

            plt::show(false);
        }
    };
}