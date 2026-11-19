"""Gazebo (headless) + gz_ros2_control + the generated controller.

ros2 launch bringup/gz.launch.py urdf:=<test urdf> controller:=franka_controller
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def setup(context):
    urdf_path = LaunchConfiguration('urdf').perform(context)
    controller = LaunchConfiguration('controller').perform(context)
    with open(urdf_path) as f:
        description = f.read()
    generated = os.path.join(get_package_share_directory(controller), 'config', controller + '.yaml')
    gz_sim = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_sim),
                                 launch_arguments={'gz_args': '-r -s -v 1 empty.sdf'}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': True}]),
        Node(package='ros_gz_sim', executable='create', arguments=['-topic', 'robot_description', '-name', 'robot']),
        Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster']),
        Node(package='controller_manager', executable='spawner',
             arguments=[controller, '-p', generated, '-p', os.path.join(HERE, 'gz_overrides.yaml')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf'),
        DeclareLaunchArgument('controller', default_value='franka_controller'),
        OpaqueFunction(function=setup),
    ])
