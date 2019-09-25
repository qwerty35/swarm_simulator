#pragma once
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <fstream>
#include <ros/ros.h>
#include <string>

using namespace rapidjson;

namespace SwarmPlanning {
    class Mission {
    public:
        int qn; // the number of quadrotors
        std::vector<std::vector<double>> startState, goalState, max_vel, max_acc;
        std::vector<double> quad_size, quad_speed;

        bool setMission(const ros::NodeHandle &nh);
        void applyNoise(double max_noise);
    };


    bool Mission::setMission(const ros::NodeHandle &nh) {
        std::string mission_addr;
        nh.param<std::string>("mission", mission_addr, "demo");

        std::ifstream ifs(mission_addr);
        IStreamWrapper isw(ifs);
        Document document;
        if(document.ParseStream(isw).HasParseError()){
            ROS_ERROR_STREAM("There is no such mission file " << mission_addr << "\n");
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
            // name
            std::string name = agents[qi].GetObject()["name"].GetString();

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
            quad_size[qi] = agents[qi].GetObject()["radius"].GetDouble();

            // speed
            quad_speed[qi] = agents[qi].GetObject()["speed"].GetDouble();

            // maximum velocity, acceleration
            std::vector<double> dynamic_limit(3, 0);
            Value::MemberIterator quadrotor = document["quadrotors"].FindMember(name.c_str());
            const Value& maxVel = quadrotor->value.GetObject()["max_vel"];
            for(SizeType i = 0; i < maxVel.Size(); i++){
                dynamic_limit[i] = maxVel[i].GetDouble();
            }
            max_vel[qi] = dynamic_limit;

            dynamic_limit.assign(3, 0);
            const Value& maxAcc = quadrotor->value.GetObject()["max_acc"];
            for(SizeType i = 0; i < maxAcc.Size(); i++){
                dynamic_limit[i] = maxAcc[i].GetDouble();
            }
            max_acc[qi] = dynamic_limit;
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
