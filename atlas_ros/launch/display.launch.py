import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
import sys

from ament_index_python import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("atlas_ros")
    atlas_model_path = os.path.join(pkg_share, "resource/atlas/atlas.urdf")
    cube_model_path = os.path.join(pkg_share, "resource/ground/boston_box.urdf")
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/atlas_config.rviz')

    atlas_robot_state_publisher_node = launch_ros.actions.Node(
        name="atlas_robot_state_publisher",
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', atlas_model_path])}],
        remappings=[("robot_description", "atlas_robot_description")]
    )
    cube_robot_state_publisher_node = launch_ros.actions.Node(
        name="cube_robot_state_publisher",
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', cube_model_path])}],
        remappings=[("robot_description", "cube_robot_description")]
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        atlas_robot_state_publisher_node,
        cube_robot_state_publisher_node,
        rviz_node
    ])