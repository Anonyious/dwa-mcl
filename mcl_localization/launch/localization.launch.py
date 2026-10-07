"""Bring up MCL with a map server, the static laser transform, and RViz.

Replaces the ROS 1 test.launch. Two notable changes beyond the API port:

* nav2_map_server's map_server is a lifecycle node, so it needs a lifecycle
  manager to be configured and activated. Without one it starts up and then
  never publishes anything.
* An earlier launch file never referenced bag/test.bag at all, and the node
  worked around the bag's old timestamps by republishing every scan on /scan2
  with a fresh stamp. That hack is gone: play the bag with --clock and set
  use_sim_time instead.

Example:
    ros2 launch mcl_localization localization.launch.py use_sim_time:=true
    ros2 bag play <bag> --clock
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('mcl_localization')
    default_map = os.path.join(share, 'map', 'map.yaml')
    default_params = os.path.join(share, 'config', 'mcl.yaml')
    default_rviz = os.path.join(share, 'config', 'display.rviz')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')
    initialize_globally = LaunchConfiguration('initialize_globally')

    args = [
        DeclareLaunchArgument('map', default_value=default_map,
                              description='Map YAML to serve.'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='MCL parameter file.'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Set true when replaying a bag with --clock.'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz.'),
        DeclareLaunchArgument(
            'initialize_globally', default_value='false',
            description='Start from a uniform cloud instead of waiting for '
                        '/initialpose. Unreliable on symmetric maps.'),
    ]

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml, 'use_sim_time': use_sim_time}],
    )

    # map_server is a lifecycle node; this drives it to 'active'.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    mcl = Node(
        package='mcl_localization',
        executable='mcl_localization_node',
        name='mcl_localization',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'initialize_globally': PythonExpression(
                    ["'", initialize_globally, "'.lower() in ('true','1')"]),
            },
        ],
    )

    # base_link -> base_laser. The node reads this offset from TF rather than
    # duplicating it as a constant, so it only needs to be right here.
    # Argument order: x y z yaw pitch roll parent child.
    static_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser',
        output='screen',
        arguments=['--x', '0.75', '--y', '-0.23', '--z', '0.0',
                   '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'base_laser'],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        args + [map_server, lifecycle_manager, static_laser_tf, mcl, rviz])
