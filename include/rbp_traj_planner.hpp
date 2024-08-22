#pragma once

// ROS
#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

// Octomap
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

// Parameters
#include <param.hpp>
#include <mission.hpp>
#include <timer.hpp>

// Submodules
#include <ecbs_planner.hpp>
#include <rbp_corridor.hpp>
#include <rbp_planner.hpp>
//#include <rbp_publisher.hpp>

//CSV
#include "csv_reader.h"

namespace SwarmPlanning {
class RBPTrajPlanner : public rclcpp::Node {
  typedef std::shared_ptr<octomap::ColorOcTree> Octree_ptr;
  typedef DynamicEDTOctomapBase<octomap::ColorOcTree> DynamicEDTColorOctomap;
  typedef std::shared_ptr<DynamicEDTColorOctomap> DynamicEDTMapPtr;
  typedef octomap_msgs::msg::Octomap OctomapMsg;
  typedef octomap::point3d point3d;
  typedef std::vector<octomap::point3d> point3ds;
  typedef visualization_msgs::msg::Marker MarkerMsg;
  typedef visualization_msgs::msg::MarkerArray MarkerArrayMsg;

private:
  PlanResult planResult;
  Param param;
  Mission mission;

  rclcpp::TimerBase::SharedPtr timer;
  Octree_ptr octree_ptr = nullptr;
  DynamicEDTMapPtr distmap_ptr = nullptr;
  OctomapMsg octomap_msg;
  std::chrono::steady_clock::time_point start_time;

  // ROS publisher
  rclcpp::Publisher<MarkerArrayMsg>::SharedPtr pub_initTraj;
  rclcpp::Publisher<MarkerArrayMsg>::SharedPtr pub_obsBox;
  rclcpp::Publisher<MarkerArrayMsg>::SharedPtr pub_feasibleBox;
  rclcpp::Publisher<MarkerArrayMsg>::SharedPtr pub_colBox;
  rclcpp::Publisher<OctomapMsg>::SharedPtr pub_map;

  // ROS messages
  MarkerArrayMsg msgs_initTraj;
  MarkerArrayMsg msgs_obsBox;
  MarkerArrayMsg msgs_feasibleBox;
  MarkerArrayMsg msgs_colBox;

  int M = 0, dim = 3, count_map_published = 0;
  double dt = 0.1;
  std::vector<Eigen::MatrixXd> pva;
  std::vector<Eigen::MatrixXd> coef;
  std::vector<std::vector<double>> currentState;
  std::vector<double> T, t, max_ratio, min_ratio;
  std::vector<std::vector<std::vector<double>>> quad_state;

  void timer_callback();

  void initialize_publisher();

  void update_octree_from_csv(const std::string &map_file_name,
                              const Param& param);

  void update(double current_time);
  void publish();

  void update_traj(double current_time);
  void update_initTraj();
  void update_obsBox(double current_time);
  void update_feasibleBox(double current_time);
  void update_colBox();

  void timeMatrix(double current_time, int &index, Eigen::MatrixXd &polyder);

public:
  RBPTrajPlanner(int id);
};
}
