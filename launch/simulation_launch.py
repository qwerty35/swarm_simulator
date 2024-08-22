from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from time import gmtime, strftime
import os
import yaml

def generate_launch_description():
    mission_file_name = "mission/empty10/multi_random_10agents_1.json"
    param_file_name = "config/param.yaml"

    ld = LaunchDescription()
    traj_planner_node = Node(
        package="swarm_planner",
        namespace="",
        executable="rbp_traj_planner",
        output="screen",
        name="rbp_traj_planner",
        parameters=[
            {"mission_file_name": mission_file_name},
            {"param_file_name": param_file_name}
        ]
    )
    ld.add_action(traj_planner_node)

    # package_path = str(os.path.realpath(get_package_share_directory('swarm_planner')))
    # rviz_path = package_path + '/config/rviz_config_sim.rviz'
    # rviz_node = Node(
    #     package='rviz2',
    #     namespace='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_path],
    # )
    # ld.add_action(rviz_node)

    return ld
