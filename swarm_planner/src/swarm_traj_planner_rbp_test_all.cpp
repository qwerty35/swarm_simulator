// Configuration
#include <param.hpp>
#include <mission.hpp>
#include <timer.hpp>

// Octomap
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

// Submodule
#include <ECBSPlanner.hpp>
#include <SIPPPlanner.hpp>
#include <BoxGeneratorGeometric.hpp>
#include <SwarmPlannerGeometric.hpp>
#include <TrajPublisherGeometric.hpp>
#include <TrajPlotter.hpp>

int main(int argc, char* argv[]) {
    ROS_INFO("Swarm Trajectory Planner");
    ros::init (argc, argv, "swarm_traj_planner_rbp_test_all");
    ros::NodeHandle nh( "~" );

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

    std::shared_ptr<octomap::OcTree> octree_obj;


    // Submodules
    std::shared_ptr<DynamicEDTOctomap> distmap_obj;
    std::shared_ptr<InitTrajPlanner> init_traj_planner;
    std::shared_ptr<BoxGenerator> box_generator;
    std::shared_ptr<SwarmPlanner> swarm_planner;
    std::shared_ptr<TrajPublisher> traj_publisher;
    std::shared_ptr<TrajPlotter> traj_plotter;

    // Main Loop
    Timer timer_total;
    Timer timer_step;

    for(int i = 1; i <= 50; i++) {
        ROS_INFO_STREAM("Map: map" << i << ".bt");
        octree_obj.reset(new octomap::OcTree(param.path + "/worlds/map" + std::to_string(i) + ".bt"));

        timer_total.reset();

        // Build 3D Euclidean Distance Field
        timer_step.reset();

        float maxDist = 1;
        octomap::point3d min_point3d(param.world_x_min, param.world_y_min, param.world_z_min);
        octomap::point3d max_point3d(param.world_x_max, param.world_y_max, param.world_z_max);

        distmap_obj.reset(new DynamicEDTOctomap(maxDist, octree_obj.get(), min_point3d, max_point3d, false));
        distmap_obj.get()->update();

        timer_step.stop();
        ROS_INFO_STREAM("Euclidean Distmap runtime: " << timer_step.elapsedSeconds());

        // Step 1: Plan Initial Trajectory
        timer_step.reset();

        if (param.initTraj_planner_type == SP_IPT_ECBS)
            init_traj_planner.reset(new ECBSPlanner(distmap_obj, mission, param));
        else if (param.initTraj_planner_type == SP_IPT_SIPP)
            init_traj_planner.reset(new SIPPPlanner(distmap_obj, mission, param));
        else {
            ROS_ERROR("Invalid Initial Trajectory Planner");
            return -1;
        }

        if (!init_traj_planner.get()->update(param.log)) {
            return -1;
        }

        timer_step.stop();
        ROS_INFO_STREAM("Initial Trajectory Planner runtime: " << timer_step.elapsedSeconds());

        // Step 2: Generate SFC, RSFC
        timer_step.reset();

        box_generator.reset(new BoxGenerator(init_traj_planner, distmap_obj, mission, param));
        if (!box_generator.get()->update(param.log)) {
            return -1;
        }

        timer_step.stop();
        ROS_INFO_STREAM("BoxGenerator runtime: " << timer_step.elapsedSeconds());

        // Step 3: Plan Swarm Trajectory
        timer_step.reset();

        swarm_planner.reset(new SwarmPlanner(box_generator, init_traj_planner, mission, param));
        if (!swarm_planner.get()->update(param.log)) {
            return -1;
        }

        timer_step.stop();
        ROS_INFO_STREAM("SwarmPlanner runtime: " << timer_step.elapsedSeconds());

        timer_total.stop();
        ROS_INFO_STREAM("Overall runtime: " << timer_total.elapsedSeconds());

        // Plot Planning Result
        traj_plotter.reset(new TrajPlotter(swarm_planner, box_generator, init_traj_planner, mission, param));
        traj_plotter->plot(false);
    }

    return 0;
}