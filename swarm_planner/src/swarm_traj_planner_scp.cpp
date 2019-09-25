// ROS
#include <ros/ros.h>

// Useful tools
#include <param.hpp>
#include <mission.hpp>
#include <timer.hpp>

// Submodule
#include <SwarmPlannerSCP.hpp>
#include <TrajPublisherSCP.hpp>
#include <TrajPlotterSCP.hpp>

bool has_path = false;


int main(int argc, char* argv[]) {
    ROS_INFO("Swarm Trajectory Planner - Sequential Convex Programming");
    ros::init (argc, argv, "swarm_traj_planner_scp");
    ros::NodeHandle nh( "~" );
    ros::V_string args;
    ros::removeROSArgs(argc, argv, args);

    // Set Mission
    SwarmPlanning::Mission mission;
    if(!mission.setMission(nh)){
        return -1;
    }
    mission.applyNoise(0.01);

    // Set ROS Parameters
    SwarmPlanning::Param param;
    if(!param.setROSParam(nh)){
        return -1;
    }
    param.setColor(mission.qn);


    // Submodules
    std::shared_ptr<SwarmPlannerSCP> swarm_planner;
    std::shared_ptr<TrajPublisherSCP> traj_publisher;
    std::shared_ptr<TrajPlotterSCP> traj_plotter;

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

            swarm_planner.reset(new SwarmPlannerSCP(mission, param));
            if(!swarm_planner.get()->update(param.log)){
                return -1;
            }

            timer_total.stop();
            ROS_INFO_STREAM("Overall runtime: " << timer_total.elapsedSeconds());

            // Initialize Trajectory Publisher
            traj_publisher.reset(new TrajPublisherSCP(nh, swarm_planner, mission, param));

            // Plot Planning Result
            traj_plotter.reset(new TrajPlotterSCP(swarm_planner, mission, param));
            traj_plotter->plot();

            start_time = ros::Time::now().toSec();
            has_path = true;
        }
        if(has_path) {
            // Publish Swarm Trajectory
            current_time = ros::Time::now().toSec() - start_time;
            traj_publisher.get()->update(current_time);
            traj_publisher.get()->publish();
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}