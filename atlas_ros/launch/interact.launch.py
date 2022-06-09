import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
import sys

from ament_index_python import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("atlas_ros")
    atlas_model_path = os.path.join(pkg_share, "resource/atlas/atlas.urdf")
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/atlas_config_interaction.rviz')

    atlas_robot_joint_publisher_node = launch_ros.actions.Node(
        name="atlas_robot_joint_publisher",
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', atlas_model_path])}],
        remappings=[("robot_description", "atlas_robot_description")]
    )
    atlas_robot_joint_publisher_gui_node = launch_ros.actions.Node(
        name="atlas_robot_joint_publisher_gui",
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'robot_description': Command(['xacro ', atlas_model_path])}],
        remappings=[("robot_description", "atlas_robot_description")]
    )
    atlas_robot_state_publisher_node = launch_ros.actions.Node(
        name="atlas_robot_state_publisher",
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', atlas_model_path])}],
        remappings=[("robot_description", "atlas_robot_description")]
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
        atlas_robot_joint_publisher_gui_node,
        # atlas_robot_joint_publisher_node,
        rviz_node
    ])