#pragma once
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sp_const.hpp>
#include <string>
#include "yaml-cpp/yaml.h"

namespace SwarmPlanning{
    class Param {
    public:
        bool log = false;
        std::string package_path;
        std::string map_file_name;

        double world_x_min = -5;
        double world_y_min = -5;
        double world_z_min = 0;
        double world_x_max = 5;
        double world_y_max = 5;
        double world_z_max = 2.5;

        double ecbs_w = 1.3;
        double grid_xy_res = 0.3;
        double grid_z_res = 0.6;
        double grid_margin = 0.2;

        double box_xy_res = 0.1;
        double box_z_res = 0.1;

        bool time_scale = true;
        double time_step = 1.0;
        double downwash = 2.0; // downwash coefficient
        int iteration = 1;
        bool sequential = false;
        int batch_size = 4; // the number of agents in a batch
        int batch_iter = 0; // the number of batches
        int n; // degree of polynomial
        int phi; // desired derivatives

        std::vector<std::vector<double>> color;

        bool load(const std::string& param_file_name);
        void setColor(int qn);
    };

    bool Param::load(const std::string& param_file_name) {
        YAML::Node param;
        std::string param_path;
        try {
          package_path = ament_index_cpp::get_package_share_directory("swarm_planner");
          param_path = package_path + "/" + param_file_name;
          param = YAML::LoadFile(param_path);
        } catch (...) {
          std::cout << "[Param] Fail to load param file, current_path: " << param_path << "\n";
          return false;
        }

        map_file_name = param["map_file_name"].as<std::string>();

        log = param["log"].as<bool>();

        world_x_min = param["world/x_min"].as<double>();
        world_y_min = param["world/y_min"].as<double>();
        world_z_min = param["world/z_min"].as<double>();
        world_x_max = param["world/x_max"].as<double>();
        world_y_max = param["world/y_max"].as<double>();
        world_z_max = param["world/z_max"].as<double>();

        grid_xy_res = param["grid/xy_res"].as<double>();
        grid_z_res = param["grid/z_res"].as<double>();
        grid_margin = param["grid/margin"].as<double>();
        ecbs_w = param["ecbs/w"].as<double>();

        box_xy_res = param["box/xy_res"].as<double>();
        box_z_res = param["box/z_res"].as<double>();

        time_scale = param["plan/time_scale"].as<bool>();
        time_step = param["plan/time_step"].as<double>();
        downwash = param["plan/downwash"].as<double>();
        n = param["plan/n"].as<int>();
        phi = param["plan/phi"].as<int>();

        sequential = param["plan/sequential"].as<bool>();
        batch_size = param["plan/batch_size"].as<int>();
        batch_iter = param["plan/batch_iter"].as<int>();
        iteration = param["plan/iteration"].as<int>();

        return true;
    }

    // HSV Colormap
    void Param::setColor(int qn){
        double h,f;
        int i;

        color.resize(qn);
        for(int qi = 0; qi < qn; qi++){
            color[qi].resize(3);
            h = qi * 6 / (double)qn;
            i = (int)h;
            f = h-i;

            switch(i){
                case 0:
                    color[qi][0] = 1;
                    color[qi][1] = f;
                    color[qi][2] = 0;
                    break;
                case 1:
                    color[qi][0] = 1-f;
                    color[qi][1] = 1;
                    color[qi][2] = 0;
                    break;
                case 2:
                    color[qi][0] = 0;
                    color[qi][1] = 1;
                    color[qi][2] = f;
                    break;
                case 3:
                    color[qi][0] = 0;
                    color[qi][1] = 1-f;
                    color[qi][2] = 1;
                    break;
                case 4:
                    color[qi][0]= f;
                    color[qi][1]= 0;
                    color[qi][2]= 1;
                    break;
                case 5:
                    color[qi][0]= 1;
                    color[qi][1]= 0;
                    color[qi][2]= 1-f;
                    break;
                default:
                    break;
            }
        }
    }
}
