import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
import sys

from ament_index_python import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("atlas_ros")
    default_model_path = os.path.join(pkg_share, "resource/atlas/atlas.urdf")
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/atlas_config.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        robot_state_publisher_node,
        rviz_node
    ])