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
#include <ecbs_planner.hpp>
#include <rbp_corridor.hpp>
#include <rbp_planner.hpp>

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
    std::shared_ptr<InitTrajPlanner> initTrajPlanner_obj;
    std::shared_ptr<Corridor> corridor_obj;
    std::shared_ptr<RBPPlanner> RBPPlanner_obj;

    // Main Loop
    Timer timer_total;
    Timer timer_step;

    for(int i = 1; i <= 50; i++) {
        ROS_INFO_STREAM("Map: map" << i << ".bt");
        octree_obj.reset(new octomap::OcTree(param.package_path + "/worlds/map" + std::to_string(i) + ".bt"));

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
        {
            initTrajPlanner_obj.reset(new ECBSPlanner(distmap_obj, mission, param));
            if (!initTrajPlanner_obj.get()->update(param.log)) {
                return -1;
            }
        }
        timer_step.stop();
        ROS_INFO_STREAM("Initial Trajectory Planner runtime: " << timer_step.elapsedSeconds());

        // Step 2: Generate SFC, RSFC
        timer_step.reset();
        {
            corridor_obj.reset(new Corridor(initTrajPlanner_obj, distmap_obj, mission, param));
            if (!corridor_obj.get()->update(param.log)) {
                return -1;
            }
        }
        timer_step.stop();
        ROS_INFO_STREAM("BoxGenerator runtime: " << timer_step.elapsedSeconds());

        // Step 3: Formulate QP problem and solving it to generate trajectory for quadrotor swarm
        timer_step.reset();
        {
            RBPPlanner_obj.reset(new RBPPlanner(corridor_obj, initTrajPlanner_obj, mission, param));
            if (!RBPPlanner_obj.get()->update(param.log)) {
                return -1;
            }
        }
        timer_step.stop();
        ROS_INFO_STREAM("SwarmPlanner runtime: " << timer_step.elapsedSeconds());

        timer_total.stop();
        ROS_INFO_STREAM("Overall runtime: " << timer_total.elapsedSeconds());
    }

    return 0;
}