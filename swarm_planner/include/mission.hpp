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
//        std::vector<double> quad_size;
        std::vector<double> quad_speed;
        std::vector<std::vector<CollisionModel_internal>> quad_collision_model;
        std::vector<int> quad_priority;

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
//        quad_size.resize(qn);
        quad_collision_model.resize(qn);
        for(int qi = 0; qi < qn; qi++){
            quad_collision_model[qi].resize(qn);
        }
        quad_priority.resize(qn);
        quad_speed.resize(qn);
        max_vel.resize(qn);
        max_acc.resize(qn);

        for(SizeType qi = 0; qi < qn; qi++){
            // name
            std::string name = agents[qi].GetObject()["name"].GetString();
            Value::MemberIterator quadrotor = document["quadrotors"].FindMember(name.c_str());

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

            // priority
            quad_priority[qi] = quadrotor->value.GetObject()["priority"].GetInt();

//            // radius
//            quad_size[qi] = agents[qi].GetObject()["radius"].GetDouble();

            // size
            const Value& size = quadrotor->value.GetObject()["size"];
            quad_collision_model[qi][qi] = CollisionModel_internal(size[0].GetDouble(), size[1].GetDouble(), size[2].GetDouble());

            // speed
//            quad_speed[qi] = agents[qi].GetObject()["speed"].GetDouble();
            quad_speed[qi] = quadrotor->value.GetObject()["speed"].GetDouble();

            // maximum velocity, acceleration
            std::vector<double> dynamic_limit(3, 0);
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

        //collision_model
        for(int qi = 0; qi < qn; qi++) {
            for(int qj = qi + 1; qj < qn; qj++) {
                quad_collision_model[qi][qj].r = quad_collision_model[qi][qi].r + quad_collision_model[qj][qj].r;
                quad_collision_model[qj][qi].r = quad_collision_model[qi][qi].r + quad_collision_model[qj][qj].r;
                if(quad_priority[qi] > quad_priority[qj]){
                    quad_collision_model[qi][qj].a = quad_collision_model[qi][qi].b + quad_collision_model[qj][qj].a;
                    quad_collision_model[qj][qi].a = quad_collision_model[qi][qi].r + quad_collision_model[qj][qj].r;
                    quad_collision_model[qi][qj].b = quad_collision_model[qi][qi].r + quad_collision_model[qj][qj].r;
                    quad_collision_model[qj][qi].b = quad_collision_model[qi][qi].b + quad_collision_model[qj][qj].a;

                }
                else if(quad_priority[qi] < quad_priority[qj]){
                    quad_collision_model[qi][qj].a = quad_collision_model[qi][qi].r + quad_collision_model[qj][qj].r;
                    quad_collision_model[qj][qi].a = quad_collision_model[qi][qi].a + quad_collision_model[qj][qj].b;
                    quad_collision_model[qi][qj].b = quad_collision_model[qi][qi].a + quad_collision_model[qj][qj].b;
                    quad_collision_model[qj][qi].b = quad_collision_model[qi][qi].r + quad_collision_model[qj][qj].r;
                }
                else{
                    quad_collision_model[qi][qj].a = quad_collision_model[qi][qi].b + quad_collision_model[qj][qj].a;
                    quad_collision_model[qj][qi].a = quad_collision_model[qi][qi].a + quad_collision_model[qj][qj].b;
                    quad_collision_model[qi][qj].b = quad_collision_model[qi][qi].a + quad_collision_model[qj][qj].b;
                    quad_collision_model[qj][qi].b = quad_collision_model[qi][qi].b + quad_collision_model[qj][qj].a;
                }
            }
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
