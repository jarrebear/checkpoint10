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

    wheel_velocities_node = Node(
        package='wheel_velocities_publisher',
        executable='wheel_velocities_publisher',
        output='screen',
        name='wheel_velocities_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]
    )


    # create and return launch description object
    return LaunchDescription(
        [
            wheel_velocities_node,
            kinematic_model_node
        ]
    )