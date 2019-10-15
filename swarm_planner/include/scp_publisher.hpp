#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "scp_planner.hpp"
#include <mission.hpp>
#include <param.hpp>

namespace SwarmPlanning {
    class SCPPublisher {
    public:
        SCPPublisher(const ros::NodeHandle &_n,
                     std::shared_ptr<SCPPlanner> _SCPPlanner_obj,
                     Mission _mission,
                     Param _param)
                : n(_n),
                  SCPPlanner_obj(_SCPPlanner_obj),
                  mission(_mission),
                  param(_param) {
            N = round(SCPPlanner_obj->msgs_traj_info.data[0]);
            K = round(SCPPlanner_obj->msgs_traj_info.data[1]);
            h = SCPPlanner_obj->msgs_traj_info.data[2];
            outdim = 3;

            traj_pubs.resize(N);
            msgs_traj.resize(N);
            for (int qi = 0; qi < N; qi++) {
                std::string mav_name = "/mav" + std::to_string(qi);
                traj_pubs[qi] = n.advertise<nav_msgs::Path>("/desired_trajectory" + mav_name, 1);
            }
            colBox_pub = n.advertise<visualization_msgs::MarkerArray>("/collision_model", 1);

            p_curr.resize(N);
            build_mapping_mtx();
            std::vector<double> data = SCPPlanner_obj->msgs_traj_input.data;
            u = Eigen::Map<Eigen::MatrixXd>(data.data(), outdim * N * K, 1);
            p = P * u + p_start;
            v = V * u;
            a = A * u;
        }

        void update(double current_time) {
            update_traj(current_time);
            update_colBox();
        }

        void publish() {
            for (int qi = 0; qi < N; qi++) {
                traj_pubs[qi].publish(msgs_traj[qi]);
            }
            colBox_pub.publish(msgs_colBox);
        }

    private:
        ros::NodeHandle n;
        std::shared_ptr<SCPPlanner> SCPPlanner_obj;
        Mission mission;
        Param param;

        int N, K, outdim;
        double h;
        tf::TransformBroadcaster br;
        Eigen::MatrixXd P, V, A, J, p_start, p_goal, u, p, v, a;
        std::vector<Eigen::MatrixXd> p_curr;

        std::vector<ros::Publisher> traj_pubs;
        ros::Publisher colBox_pub;

        std::vector<nav_msgs::Path> msgs_traj;
        visualization_msgs::MarkerArray msgs_colBox;

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

        void update_traj(double current_time) {
            int k = floor(current_time / h);
            if (k >= K - 1) {
                return;
            }

            for (int qi = 0; qi < N; qi++) {
                msgs_traj[qi].header.frame_id = "/world";
                msgs_traj[qi].header.stamp.sec = current_time;

                p_curr[qi] = Eigen::MatrixXd::Zero(outdim, 1);
                Eigen::MatrixXd p_0 = Eigen::MatrixXd::Zero(outdim, 1);
                Eigen::MatrixXd p_1 = Eigen::MatrixXd::Zero(outdim, 1);
                for (int dim = 0; dim < outdim; dim++) {
                    p_0(dim, 0) = p(dim * N * K + qi * K + k, 0);
                    p_1(dim, 0) = p(dim * N * K + qi * K + k + 1, 0);
                }
                p_curr[qi] = p_0 + (current_time - k * h) / h * (p_1 - p_0);

                geometry_msgs::PoseStamped pos_des;
                pos_des.header.frame_id = "/world";
                pos_des.pose.position.x = p_curr[qi](0, 0);
                pos_des.pose.position.y = p_curr[qi](1, 0);
                pos_des.pose.position.z = p_curr[qi](2, 0);
                msgs_traj[qi].poses.emplace_back(pos_des);

                tf::Transform transform;
                transform.setOrigin(
                        tf::Vector3(pos_des.pose.position.x, pos_des.pose.position.y, pos_des.pose.position.z));
                tf::Quaternion q;
                q.setRPY(0, 0, 0);
                transform.setRotation(q);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                      "world", "/mav" + std::to_string(qi) + "/base_link"));
            }
        }

        // obstacle-collision model
        void update_colBox() {
            visualization_msgs::MarkerArray mk_array;
            for (int qi = 0; qi < N; qi++) {
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
                mk.pose.position.x = p_curr[qi](0, 0);
                mk.pose.position.y = p_curr[qi](1, 0);
                mk.pose.position.z = p_curr[qi](2, 0);

                mk.scale.x = 2 * mission.quad_size[qi];
                mk.scale.y = 2 * mission.quad_size[qi];
                mk.scale.z = 2 * mission.quad_size[qi];

                mk.color.a = 0.7;
                mk.color.r = param.color[qi][0];
                mk.color.g = param.color[qi][1];
                mk.color.b = param.color[qi][2];

                mk_array.markers.emplace_back(mk);

                mk.ns = "initTraj";
                for (int k = 0; k < K; k++) {
                    mk.id = 1000 + qi * K + k;
                    Eigen::MatrixXd picker_i = position_picker(qi, k);
                    Eigen::MatrixXd p_i = picker_i * p;
                    mk.pose.position.x = p_i(0, 0);
                    mk.pose.position.y = p_i(1, 0);
                    mk.pose.position.z = p_i(2, 0);

                    mk.scale.x = 0.1;
                    mk.scale.y = 0.1;
                    mk.scale.z = 0.1;

                    mk.color.a = 0.5;
                    mk.color.r = param.color[qi][0];
                    mk.color.g = param.color[qi][1];
                    mk.color.b = param.color[qi][2];

                    mk_array.markers.emplace_back(mk);
                }
            }
            msgs_colBox = mk_array;
        }

        Eigen::MatrixXd position_picker(int qi, int k) {
            Eigen::MatrixXd P_pick = Eigen::MatrixXd::Zero(outdim, outdim * N * K);
            for (int dim = 0; dim < outdim; dim++) {
                P_pick(dim, dim * N * K + qi * K + k) = 1;
            }

            return P_pick;
        }
    };
}