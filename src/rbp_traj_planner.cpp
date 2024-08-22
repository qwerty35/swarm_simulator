#include "rbp_traj_planner.hpp"

using namespace SwarmPlanning;
using namespace std::chrono;
using namespace std::chrono_literals;

inline std::string get_extension(const std::string &path) {
  return path.substr(path.find_last_of('.') + 1);
}

inline double get_elapsed_time(steady_clock::time_point t) {
  double current_time =
      (double)duration_cast<microseconds>(steady_clock::now() - t).count() *
      0.000001;
  return current_time;
}

RBPTrajPlanner::RBPTrajPlanner(int id) : Node("rbp_traj_planner") {
  // Mission
  this->declare_parameter("mission_file_name", "mission/empty10/multi_random_10agents_1.json");
  std::string mission_file_name =
      this->get_parameter("mission_file_name").as_string();


  //test all
  mission_file_name = "mission/empty10/multi_random_10agents_" + std::to_string(id) + ".json";
  if(not mission.load(mission_file_name)){
    return;
  }

  // Param
  this->declare_parameter("param_file_name", "config/param.yaml");
  std::string param_file_name =
      this->get_parameter("param_file_name").as_string();
  if(not param.load(param_file_name)){
    return;
  }
  param.setColor(mission.qn);

  // Submodules
  std::shared_ptr<InitTrajPlanner> initTrajPlanner_ptr;
  std::shared_ptr<Corridor> corridor_ptr;
  std::shared_ptr<RBPPlanner> RBPPlanner_ptr;

  // Main Loop
  Timer timer_total;
  Timer timer_step;

  timer_total.reset();

  // Build 3D Euclidean Distance Field
  timer_step.reset();
  {
    // Read map file and convert it to octree
    octree_ptr = std::make_shared<octomap::ColorOcTree>(0.1);
    std::string mission_extension = get_extension(param.map_file_name);
    if (mission_extension == "csv") {
      update_octree_from_csv(param.map_file_name, param);
    } else {
      RCLCPP_INFO(this->get_logger(), "Invalid map_file_name");
      return;
    }

    float maxDist = 1;
    octomap::point3d world_min(param.world_x_min, param.world_y_min, param.world_z_min);
    octomap::point3d world_max(param.world_x_max, param.world_y_max, param.world_z_max);
    distmap_ptr = std::make_shared<DynamicEDTColorOctomap>(
        maxDist, octree_ptr.get(), world_min, world_max, false);
    distmap_ptr->update();

    bool success = octomap_msgs::fullMapToMsg(*octree_ptr, octomap_msg);
    if (not success) {
      RCLCPP_INFO(this->get_logger(), "Octomap message conversion failed!");
      return;
    } else {
      octomap_msg.header.frame_id = "world";
    }
  }
  timer_step.stop();
//  RCLCPP_INFO(this->get_logger(), "distmap runtime: %f", timer_step.elapsedSeconds());

  // Step 1: Plan Initial Trajectory
  timer_step.reset();
  {
    initTrajPlanner_ptr.reset(new ECBSPlanner(distmap_ptr, mission, param));
    if (!initTrajPlanner_ptr->update(param.log, &planResult)) {
      return;
    }
  }
  timer_step.stop();
//  RCLCPP_INFO(this->get_logger(), "Initial Trajectory Planner runtime: %f", timer_step.elapsedSeconds());

  // Step 2: Generate SFC, RSFC
  timer_step.reset();
  {
    corridor_ptr.reset(new Corridor(distmap_ptr, mission, param));
    if (!corridor_ptr->update(param.log, &planResult)) {
      return;
    }
  }
  timer_step.stop();
//  RCLCPP_INFO(this->get_logger(), "BoxGenerator runtime: %f", timer_step.elapsedSeconds());

  // Step 3: Formulate QP problem and solving it to generate trajectory for quadrotor swarm
  timer_step.reset();
  {
    RBPPlanner_ptr.reset(new RBPPlanner(mission, param));
    if (!RBPPlanner_ptr->update(param.log, &planResult)) {
      return;
    }
  }
  timer_step.stop();
//  RCLCPP_INFO(this->get_logger(), "SwarmPlanner runtime: %f", timer_step.elapsedSeconds());

  timer_total.stop();
  RCLCPP_INFO(this->get_logger(), "Overall runtime: %f, Total flight time: %f", timer_total.elapsedSeconds(), planResult.T.back());

  // Initilize publisher
  initialize_publisher();

  // Set start time
  start_time = steady_clock::now();

  // ROS Timer
  timer = this->create_wall_timer(
      50ms, std::bind(&RBPTrajPlanner::timer_callback, this));
}

void RBPTrajPlanner::timer_callback() {
  // Publish Swarm Trajectory
  double current_time = get_elapsed_time(start_time);
  update(current_time);
  publish();
}

void RBPTrajPlanner::initialize_publisher() {
  // ROS publisher
  pub_initTraj = this->create_publisher<MarkerArrayMsg>("initTraj", 10);
  pub_obsBox = this->create_publisher<MarkerArrayMsg>("obstacle_box", 10);
  pub_feasibleBox = this->create_publisher<MarkerArrayMsg>("feasible_box", 10);
  pub_colBox = this->create_publisher<MarkerArrayMsg>("collision_model", 10);
  pub_map = this->create_publisher<OctomapMsg>("map", 10);

  M = planResult.msgs_traj_info.data.size() - 2 - 1;
  T.resize(M + 1);
  for (int m = 0; m < M + 1; m++) {
    T[m] = planResult.msgs_traj_info.data.at(m + 2);
  }

  dt = 0.1;
  t.resize(floor(T.back() / dt));
  for (int i = 0; i < t.size(); i++) {
    t[i] = i * dt;
  }

  pva.resize(mission.qn);
  coef.resize(mission.qn);
  quad_state.resize(mission.qn);
  currentState.resize(mission.qn);
  for (int qi = 0; qi < mission.qn; qi++) {
    float rows = planResult.msgs_traj_coef[qi].layout.dim.at(0).size;
    float cols = planResult.msgs_traj_coef[qi].layout.dim.at(1).size;
    std::vector<double> data = planResult.msgs_traj_coef[qi].data;
    coef[qi] = Eigen::Map<Eigen::MatrixXd>(data.data(), rows, cols);

    currentState[qi].resize(dim * param.phi);
    quad_state[qi].resize(dim * param.phi);
    for (int i = 0; i < dim * param.phi; i++) {
      quad_state[qi][i].resize(t.size());
    }
  }
}


void RBPTrajPlanner::update_octree_from_csv(const std::string &map_file_name,
                                            const Param& param) {
  octomap::Pointcloud octomap_pointcloud;
  std::string octomap_path = param.package_path + "/" + map_file_name;
  std::ifstream obstacle_csv(octomap_path);
  for (auto &row : CSVRange(obstacle_csv)) {
    if (row.size() < 2) {
      break;
    }

    point3ds points(2); // center of mass, size
    for (int i = 0; i < 2; i++) {
      for (int k = 0; k < 3; k++) {
        std::string token = std::string(row[i * 3 + k]);
        points[i](k) = std::stof(token);
      }
    }

    point3d com = points[0];  // center of mass
    point3d size = points[1]; // size
    double res = param.box_xy_res;
    int i_start = (int)round((com.x() - 0.5 * size.x()) / res);
    int i_end = (int)round((com.x() + 0.5 * size.x()) / res);
    int j_start = (int)round((com.y() - 0.5 * size.y()) / res);
    int j_end = (int)round((com.y() + 0.5 * size.y()) / res);
    int k_start = (int)round((com.z() - 0.5 * size.z()) / res);
    int k_end = (int)round((com.z() + 0.5 * size.z()) / res);

    for (int i = i_start; i < i_end; i++) {
      for (int j = j_start; j < j_end; j++) {
        for (int k = k_start; k < k_end; k++) {
          point3d point(static_cast<float>((i + 0.5) * res),
                        static_cast<float>((j + 0.5) * res),
                        static_cast<float>((k + 0.5) * res));
          octomap_pointcloud.push_back(point);
        }
      }
    }
  }

  octree_ptr->insertPointCloud(octomap_pointcloud, point3d(0, 0, 0));
  octree_ptr->expand();

  // Set color
  // for (octomap::ColorOcTree::leaf_iterator it = octree_ptr->begin_leafs(),
  //                                          end = octree_ptr->end_leafs();
  //      it != end; ++it) {
  //   it->setColor(r, g, b);
  // }
}

void RBPTrajPlanner::update(double current_time) {
  update_traj(current_time);
  update_initTraj();
  update_obsBox(current_time);
  update_feasibleBox(current_time);
  update_colBox();
}

void RBPTrajPlanner::publish() {
  pub_initTraj->publish(msgs_initTraj);
  pub_obsBox->publish(msgs_obsBox);
  pub_feasibleBox->publish(msgs_feasibleBox);
  pub_colBox->publish(msgs_colBox);

  if (count_map_published < 100) {
    pub_map->publish(octomap_msg);
    count_map_published++;
  }
}

void RBPTrajPlanner::update_traj(double current_time) {
  if (current_time > T.back()) {
    return;
  }
  for (int qi = 0; qi < mission.qn; qi++) {
    int index = 0;
    Eigen::MatrixXd polyder;
    timeMatrix(current_time, index, polyder);

    pva[qi] = polyder * coef[qi].block((param.n + 1) * index, 0, (param.n + 1), 3);

    currentState[qi][0] = pva[qi](0, 0);
    currentState[qi][1] = pva[qi](0, 1);
    currentState[qi][2] = pva[qi](0, 2);
    currentState[qi][3] = pva[qi](1, 0);
    currentState[qi][4] = pva[qi](1, 1);
    currentState[qi][5] = pva[qi](1, 2);
    currentState[qi][6] = pva[qi](2, 0);
    currentState[qi][7] = pva[qi](2, 1);
    currentState[qi][8] = pva[qi](2, 2);
  }
}

void RBPTrajPlanner::update_initTraj() {
  MarkerArrayMsg mk_array;
  for (int qi = 0; qi < mission.qn; qi++) {
    for (int m = 0; m < planResult.initTraj[qi].size(); m++) {
      MarkerMsg mk;
      mk.header.frame_id = "world";
      mk.ns = "mav" + std::to_string(qi);
      mk.type = visualization_msgs::msg::Marker::CUBE;
      mk.action = visualization_msgs::msg::Marker::ADD;

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
}

void RBPTrajPlanner::update_obsBox(double current_time) {
  MarkerArrayMsg mk_array;
  for (int qi = 0; qi < mission.qn; qi++) {
    // find current obsBox number
    int box_curr = 0;
    while (box_curr < planResult.SFC[qi].size() &&
           planResult.SFC[qi][box_curr].second < current_time) {
      box_curr++;
    }
    if (box_curr >= planResult.SFC[qi].size()) {
      box_curr = planResult.SFC[qi].size() - 1;
    }

    MarkerMsg mk;
    mk.header.frame_id = "world";
    mk.ns = "mav" + std::to_string(qi);
    mk.type = visualization_msgs::msg::Marker::CUBE;
    mk.action = visualization_msgs::msg::Marker::ADD;

    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    for (int bi = 0; bi < planResult.SFC[qi].size(); bi++) {
      mk.id = bi;
      //                mk.header.stamp = ros::Time(obstacle_boxes[qi][bi].second);
      std::vector<double> obstacle_box = planResult.SFC[qi][bi].first;

      {
        double margin = mission.quad_size[qi];
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

void RBPTrajPlanner::update_feasibleBox(double current_time) {
  MarkerArrayMsg mk_array;
  for (int qj = 0; qj < mission.qn; qj++) {
    // current position
    MarkerMsg mk;
    mk.header.frame_id = "world";
    mk.ns = "mav" + std::to_string(qj);
    mk.action = visualization_msgs::msg::Marker::ADD;

    // inter-collision model
    for (int qi = 0; qi < mission.qn; qi++) {
      mk.type = visualization_msgs::msg::Marker::SPHERE;
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
    for (int qi = 0; qi < mission.qn; qi++) {
      mk.type = visualization_msgs::msg::Marker::CUBE;
      mk.id = mission.qn + qi;
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
      if (qi < qj) { // RSFC
        while (box_curr < planResult.RSFC[qi][qj].size() &&
               planResult.RSFC[qi][qj][box_curr].second < current_time) {
          box_curr++;
        }
        if (box_curr >= planResult.RSFC[qi][qj].size()) {
          box_curr = planResult.RSFC[qi][qj].size() - 1;
        }

        normal_vector = planResult.RSFC[qi][qj][box_curr].first;
      } else if (qi > qj) { // RSFC
        while (box_curr < planResult.RSFC[qj][qi].size() &&
               planResult.RSFC[qj][qi][box_curr].second < current_time) {
          box_curr++;
        }
        if (box_curr >= planResult.RSFC[qj][qi].size()) {
          box_curr = planResult.RSFC[qj][qi].size() - 1;
        }

        normal_vector = -planResult.RSFC[qj][qi][box_curr].first;
      } else { // SFC
        while (box_curr < planResult.SFC[qi].size() &&
               planResult.SFC[qi][box_curr].second < current_time) {
          box_curr++;
        }
        if (box_curr >= planResult.SFC[qi].size()) {
          box_curr = planResult.SFC[qi].size() - 1;
        }
        std::vector<double> obstacle_box = planResult.SFC[qi][box_curr].first;
        for (int iter = 0; iter < 6; iter++) {
          double margin = mission.quad_size[qi];
          if (iter == 2 || iter == 5)
            margin *= param.downwash;
          if (iter < 3)
            obstacle_box[iter] = obstacle_box[iter] - margin;
          else
            obstacle_box[iter] = obstacle_box[iter] + margin;
        }

        mk.type = visualization_msgs::msg::Marker::LINE_LIST;
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

        geometry_msgs::msg::Point pt_i, pt_j;
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

void RBPTrajPlanner::update_colBox() {
  visualization_msgs::msg::MarkerArray mk_array;
  for (int qi = 0; qi < mission.qn; qi++) {
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = "world";
    mk.ns = "colBox";
    mk.type = visualization_msgs::msg::Marker::SPHERE;
    mk.action = visualization_msgs::msg::Marker::ADD;

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

    mk_array.markers.emplace_back(mk);
  }
  msgs_colBox = mk_array;
}

void RBPTrajPlanner::timeMatrix(double current_time, int &index, Eigen::MatrixXd &polyder) {
  double tseg = 0;
  double tcand;

  // find segment start time tseg
  for (int m = 0; m < M; m++) {
    tcand = T[m];
    if (tcand < current_time) {
      tseg = tcand;
      index = m;
    } else {
      break;
    }
  }
  tseg = current_time - tseg;
  polyder.resize(3, param.n + 1);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < param.n + 1; j++) {
      if (i <= j)
        polyder(i, param.n - j) =
            ((i == 0) * 1 + (i == 1) * j + (i == 2) * j * (j - 1)) * pow(tseg, j - i);
      else
        polyder(i, param.n - j) = 0;
    }
  }
}

int main(int argc, const char *argv[]) {
//  rclcpp::init(argc, argv);
//  rclcpp::spin(std::make_shared<RBPTrajPlanner>());
//  rclcpp::shutdown();

  rclcpp::init(argc, argv);
  for(int id = 1; id <= 30; id++) {
    auto a = RBPTrajPlanner(id);
  }
  rclcpp::shutdown();

  return 0;
}