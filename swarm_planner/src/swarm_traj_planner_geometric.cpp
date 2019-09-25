// ROS
#include <ros/ros.h>

// Octomap
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

// Useful tools
#include <param.hpp>
#include <mission.hpp>
#include <timer.hpp>

// Submodule
#include <ECBSPlanner.hpp>
#include <SIPPPlanner.hpp>
#include <BoxGeneratorGeometric.hpp>
#include <SwarmPlannerGeometric.hpp>
#include <TrajPublisherGeometric.hpp>
#include <TrajPlotter.hpp>

bool has_octomap = false;
bool has_path = false;
std::shared_ptr<octomap::OcTree> octree_obj;

void octomapCallback(const octomap_msgs::Octomap& octomap_msg)
{
    if(has_octomap)
        return;

    octree_obj.reset(dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(octomap_msg)));

    has_octomap = true;
}

int main(int argc, char* argv[]) {
    ROS_INFO("Swarm Trajectory Planner");
    ros::init (argc, argv, "swarm_traj_planner_geometric");
    ros::NodeHandle nh( "~" );
    ros::Subscriber octomap_sub = nh.subscribe( "/octomap_full", 1, octomapCallback );

    // Set Mission
    SwarmPlanning::Mission mission;
    if(!mission.setMission(nh)){
        return -1;
    }

    // Set ROS Parameters
    SwarmPlanning::Param param;
    if(!param.setROSParam(nh)){
        return -1;
    }
    param.setColor(mission.qn);

    // Submodules
    std::shared_ptr<DynamicEDTOctomap> distmap_obj;
    std::shared_ptr<InitTrajPlanner> init_traj_planner;
    std::shared_ptr<BoxGenerator> box_generator;
    std::shared_ptr<SwarmPlanner> swarm_planner;
    std::shared_ptr<TrajPublisher> traj_publisher;
    std::shared_ptr<TrajPlotter> traj_plotter;

    // Main Loop
    ros::Rate rate(20);
    Timer timer_total;
    Timer timer_step;
    double start_time, current_time;
    while (ros::ok()) {
        if (has_octomap && !has_path) {
            timer_total.reset();

            // Build 3D Euclidean Distance Field
            timer_step.reset();

            float maxDist = 1;
            octomap::point3d min_point3d(param.world_x_min, param.world_y_min, param.world_z_min);
            octomap::point3d max_point3d(param.world_x_max, param.world_y_max, param.world_z_max);

            distmap_obj.reset(new DynamicEDTOctomap(maxDist, octree_obj.get(), min_point3d, max_point3d, false));
            distmap_obj.get()->update();

            timer_step.stop();
            ROS_INFO_STREAM("distmap runtime: " << timer_step.elapsedSeconds());

            // Step 1: Plan Initial Trajectory
            timer_step.reset();

            if(param.initTraj_planner_type == SP_IPT_ECBS)
                init_traj_planner.reset(new ECBSPlanner(distmap_obj, mission, param));
            else if(param.initTraj_planner_type == SP_IPT_SIPP)
                init_traj_planner.reset(new SIPPPlanner(distmap_obj, mission, param));
            else{
                ROS_ERROR("Invalid Initial Trajectory Planner");
                return -1;
            }

            if(!init_traj_planner.get()->update(param.log)){
                return -1;
            }

            timer_step.stop();
            ROS_INFO_STREAM("Initial Trajectory Planner runtime: " << timer_step.elapsedSeconds());

            // Step 2: Generate SFC, RSFC
            timer_step.reset();

            box_generator.reset(new BoxGenerator(init_traj_planner, distmap_obj, mission, param));
            if(!box_generator.get()->update(param.log)){
                return -1;
            }

            timer_step.stop();
            ROS_INFO_STREAM("BoxGenerator runtime: " << timer_step.elapsedSeconds());

            // Step 3: Plan Swarm Trajectory
            timer_step.reset();

            swarm_planner.reset(new SwarmPlanner(box_generator, init_traj_planner, mission, param));
            if(!swarm_planner.get()->update(param.log)){
                return -1;
            }

            timer_step.stop();
            ROS_INFO_STREAM("SwarmPlanner runtime: " << timer_step.elapsedSeconds());

            timer_total.stop();
            ROS_INFO_STREAM("Overall runtime: " << timer_total.elapsedSeconds());

            // Initialize Trajectory Publisher
            traj_publisher.reset(new TrajPublisher(nh, swarm_planner, box_generator, init_traj_planner, mission, param));

            // Plot Planning Result
            traj_plotter.reset(new TrajPlotter(swarm_planner, box_generator, init_traj_planner, mission, param));
            traj_plotter->plot(param.log);

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