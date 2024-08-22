#pragma once
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <fstream>
#include <string>
#include "rclcpp/rclcpp.hpp"

using namespace rapidjson;

namespace SwarmPlanning {
    class Mission {
    public:
        int qn; // the number of quadrotors
        std::vector<std::vector<double>> startState, goalState, max_vel, max_acc;
        std::vector<double> quad_size, quad_speed;

        bool load(const std::string& mission_path);
        void applyNoise(double max_noise);
    };

    bool Mission::load(const std::string& mission_file_name) {
        std::string package_path = ament_index_cpp::get_package_share_directory("swarm_planner");
        std::string mission_path = package_path + "/" + mission_file_name;
        std::ifstream ifs(mission_path);
        IStreamWrapper isw(ifs);
        Document document;
        if(document.ParseStream(isw).HasParseError()){
            std::cout << "There is no such mission file " << mission_path << "\n";
            return false;
        }

        const Value& agents = document["agents"];
        qn = agents.Size();

        startState.resize(qn);
        goalState.resize(qn);
        quad_size.resize(qn);
        quad_speed.resize(qn);
        max_vel.resize(qn);
        max_acc.resize(qn);

        for(SizeType qi = 0; qi < agents.Size(); qi++){
            // start
            std::vector<double> state(9, 0);
            const Value& start = agents[qi].GetObject()["start"];
            for(SizeType i = 0; i < start.Size(); i++){
                state[i] = start[i].GetDouble();
            }
            startState[qi] = state;

            // goal
            state.assign(9, 0);
            const Value& goal = agents[qi].GetObject()["goal"];
            for(SizeType i = 0; i < goal.Size(); i++){
                state[i] = goal[i].GetDouble();
            }
            goalState[qi] = state;

            // radius
            quad_size[qi] = 0.15;

            // speed
            quad_speed[qi] = 1.0;

            // maximum velocity, acceleration
            max_vel[qi] = {1.0, 1.0, 1.0};
            max_acc[qi] = {2.0, 2.0, 2.0};
        }

        return true;
    }

    void Mission::applyNoise(double max_noise){
        srand((unsigned int)time(nullptr));
        for(int qi = 0; qi < qn; qi++) {
            for(int k = 0; k < 3; k++){
                startState[qi][k] += rand()/(double)RAND_MAX * max_noise;
                goalState[qi][k] += rand()/(double)RAND_MAX * max_noise;
            }
        }
    }
}
