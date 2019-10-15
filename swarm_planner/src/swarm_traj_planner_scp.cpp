// ROS
#include <ros/ros.h>

// Useful tools
#include <param.hpp>
#include <mission.hpp>
#include <timer.hpp>

// Submodule
#include <scp_planner.hpp>
#include <scp_publisher.hpp>
#include <scp_plotter.hpp>

using namespace SwarmPlanning;

bool has_path = false;

int main(int argc, char* argv[]) {
    ROS_INFO("Swarm Trajectory Planner - Sequential Convex Programming");
    ros::init (argc, argv, "swarm_traj_planner_scp");
    ros::NodeHandle nh( "~" );
    ros::V_string args;
    ros::removeROSArgs(argc, argv, args);

    // Set Mission
    Mission mission;
    if(!mission.setMission(nh)){
        return -1;
    }
    mission.applyNoise(0.01);

    // Set ROS Parameters
    Param param;
    if(!param.setROSParam(nh)){
        return -1;
    }
    param.setColor(mission.qn);


    // Submodules
    std::shared_ptr<SCPPlanner> SCPPlanner_obj;
    std::shared_ptr<SCPPublisher> SCPPublisher_obj;
    std::shared_ptr<SCPPlotter> SCPPlotter_obj;

    // Main Loop
    ros::Rate rate(20);
    Timer timer_total;
    Timer timer_step;
    double start_time, current_time;
    while (ros::ok()) {
        if (!has_path) {
            timer_total.reset();

            // Plan Swarm Trajectory
            timer_step.reset();

            SCPPlanner_obj.reset(new SCPPlanner(mission, param));
            if(!SCPPlanner_obj.get()->update(param.log)){
                return -1;
            }

            timer_total.stop();
            ROS_INFO_STREAM("Overall runtime: " << timer_total.elapsedSeconds());

            // Initialize Trajectory Publisher
            SCPPublisher_obj.reset(new SCPPublisher(nh, SCPPlanner_obj, mission, param));

            // Plot Planning Result
            SCPPlotter_obj.reset(new SCPPlotter(SCPPlanner_obj, mission, param));
            SCPPlotter_obj->plot();

            start_time = ros::Time::now().toSec();
            has_path = true;
        }
        if(has_path) {
            // Publish Swarm Trajectory
            current_time = ros::Time::now().toSec() - start_time;
            SCPPublisher_obj.get()->update(current_time);
            SCPPublisher_obj.get()->publish();
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}