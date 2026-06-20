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

# testing file for use in conjunction with iss_simulation bean3_gz_test.launch.py
# does not launch gz sim 

def generate_launch_description():
    # define ros2 packages
    package_name = 'bean3_description' # for urdf files
    robot_package = get_package_share_directory(package_name)
    sim_package = get_package_share_directory('iss_simulation')
    gz_package = get_package_share_directory('ros_gz_sim')
    moveit_package = get_package_share_directory('bean2_moveit')
    default_rviz_config_path = PathJoinSubstitution([robot_package, 'rviz', 'renamed_motion_planning.rviz'])
    ros2_controllers_yaml = os.path.join(moveit_package, 'config', 'ros2_controllers.yaml')

    # define urdf.xacro
    urdf_file = os.path.join(moveit_package, 'config', 'bean2.urdf.xacro')
    robot_description_config = xacro.process_file(
            urdf_file,
            mappings={'initial_positions_file':
                      os.path.join(moveit_package, 'config', 'initial_positions.yaml')}
    )
   
    # define sdrf
    srdf_file = os.path.join(moveit_package, 'config', 'bean2.srdf')
    with open(srdf_file, 'r') as f:
        semantic_content = f.read()
    robot_description_semantic = {'robot_description_semantic': semantic_content}

    # GZ environment variable for remapping directory paths
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.path.join(os.path.join(get_package_prefix(package_name), "share"))

    # rviz argument
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
        namespace='bean',
        output='both',
        parameters=[robot_description,{'use_sim_time':True}],
    )

    # spawn robot in gazebo
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        parameters=[{'name': 'bean2',
                     'topic': 'bean/robot_description',
                     'use_sim_time':True}],
        output='screen',
    )

    # gazebo - ros2 bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # clock gz -> ros2
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        parameters=[{'use_sim_time':True}],
        output='screen'
    )

    # joint state broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/bean/controller_manager'],
        parameters=[{'use_sim_time':True}],
    )
    
    # robot controller
    robot_controller = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['robot_controller', '--param-file', ros2_controllers_yaml,
                       '--controller-manager', '/bean/controller_manager'],
            parameters=[{'use_sim_time': True}],
    )

    #rviz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{
            'use_sim_time': True},
            robot_description,
            robot_description_semantic],
    )

    ld = LaunchDescription()
    ld.add_action(rviz_config_arg)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn)
    ld.add_action(bridge)
    ld.add_action(joint_state_broadcaster)
    ld.add_action(robot_controller)
    ld.add_action(rviz_node)

    return ld
