import os
import xacro

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # define ros2 packages
    package_name = 'bean3_description'
    robot_package = get_package_share_directory(package_name)
    gz_package = get_package_share_directory('ros_gz_sim')
    default_rviz_config_path = PathJoinSubstitution([robot_package, 'rviz', 'rviz_basic_settings.rviz'])

    # define urdf
    urdf_file_name = 'bean2.urdf'
    robot_description_file = os.path.join(robot_package, 'urdf', urdf_file_name)
    robot_description_config = xacro.process_file(
        robot_description_file
    )
    
    # GZ environment variable for remapping directory paths
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.path.join(os.path.join(get_package_prefix(package_name), "share"))

    # declare launch arguments
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=robot_description_file,
        description='Absolute path to robot URDF file'
    )

    rviz_config_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file'
    )

   
    robot_description = {'robot_description': robot_description_config.toxml()}

    # robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description,{'use_sim_time':True}],
    )

    # Joint state publisher GUI node (runs when gui:=true)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # Regular joint state publisher (runs when gui:=false)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui')),
        arguments=[LaunchConfiguration('model')]
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

    #rviz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    ld = LaunchDescription()
    ld.add_action(gui_arg)
    ld.add_action(model_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn)
    ld.add_action(bridge)
    ld.add_action(joint_state_broadcaster)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz_node)

    return ld
