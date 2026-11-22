import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import launch

def generate_launch_description():

    kinematic_model_node = Node(
        package='kinematic_model',
        executable='kinematic_model',
        output='screen',
        name='kinematics_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]
    )

    eight_trajectory_node = Node(
        package='eight_trajectory',
        executable='eight_trajectory',
        output='screen',
        name='eight_trajectory_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]
    )


    # create and return launch description object
    return LaunchDescription(
        [
            eight_trajectory_node,
            kinematic_model_node
        ]
    )