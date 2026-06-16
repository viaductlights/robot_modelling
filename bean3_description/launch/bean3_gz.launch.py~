import os
import xacro

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    #define ros2 packages
    package_name = 'bean3_description'
    robot_package = get_package_share_directory(package_name)
#    gz_package = get_package_share_directory('ros_gz_sim')
    
    #GZ environment variable for remapping directory paths
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.path.join(os.path.join(get_package_prefix(package_name), "share"))

    #define urdf
    urdf_file_name = 'bean2.urdf'
    robot_description_file = os.path.join(robot_package, 'urdf', urdf_file_name)
    robot_description_config = xacro.process_file(
        robot_description_file
    )
    
    robot_description = {'robot_description': robot_description_config.toxml()}

    # robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description,{'use_sim_tim':True}],
    )

    # launch gazebo
    #gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(gz_package, 'launch', 'gz_sim.launch.py')),launch_arguments={'gz_args': '-r spacecraft.sdf','use_sim_time':'True'}.items(),)

    # spawn robot in gazebo
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        parameters=[{'name': 'bean2',
                     'topic': 'robot_description',
                     'use_sim_time':True}],
        output='screen',
    )

    # joint state broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time':True}],
    )

    # gazebo - ros2 bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # clock gz -> ros2
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # joint states gz -> ros2
            '/world/empty/model/bean2/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/empty/model/bean2/joint_state', 'joint_states'),
        ],
        parameters=[{'use_sim_time':True}],
        output='screen'
    )

    return LaunchDescription([robot_state_publisher,spawn,bridge,joint_state_broadcaster])

