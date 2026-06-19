import os
import xacro

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    #define ros2 packages
    bean_package_name = 'bean3_description'
    bean_robot_package = get_package_share_directory(bean_package_name)
    gz_package = get_package_share_directory('ros_gz_sim')

    nemo_package_name = 'nemo1_description'
    nemo_robot_package = get_package_share_directory(nemo_package_name)
    
    #GZ environment variable for remapping directory paths
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.path.join(os.path.join(get_package_prefix(bean_package_name), "share")) + ':' + os.path.join(os.path.join(get_package_prefix(nemo_package_name), "share"))

    #define bean urdf
    bean_urdf_file_name = 'bean2.urdf'
    bean_robot_description_file = os.path.join(bean_robot_package, 'urdf', bean_urdf_file_name)
    bean_robot_description_config = xacro.process_file(
        bean_robot_description_file
    )
    
    bean_robot_description = {'robot_description': bean_robot_description_config.toxml()}

    #define nemo urdf
    nemo_urdf_file_name = 'nemo1.urdf'
    nemo_robot_description_file = os.path.join(nemo_robot_package, 'urdf', nemo_urdf_file_name)
    nemo_robot_description_config = xacro.process_file(
        nemo_robot_description_file
    )
    
    nemo_robot_description = {'robot_description': nemo_robot_description_config.toxml()}

    #bean robot state publisher
    bean_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace= "bean",
        output='both',
        parameters=[bean_robot_description,{'use_sim_time':True}],
    )

    #nemo robot state publisher
    nemo_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace= "nemo",
        output='both',
        parameters=[nemo_robot_description,{'use_sim_time':True}],
    )

    bean_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'bean2',
            '-topic', 'bean/robot_description',
#            '-x', '109.0',
#            '-y', '1.43',
#            '-z', '-61',
#            '-relative_to', 'static_table::table_top',
        ],
        output='screen',
    )

    nemo_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'nemo1',
            '-topic', 'nemo/robot_description',
            '-x', '1.0',
            '-y', '0.0',
            '-z', '0.05',
            '-relative_to', 'static_table::table_top',
        ],
        output='screen',
    )
    nemo_spawn_after_bean = RegisterEventHandler(
        OnProcessExit(
            target_action=bean_spawn,
            on_exit=[nemo_spawn],
        )
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
            # bean joint states gz -> ros2
            '/world/orbit/model/bean2/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            # nemo joint states gz -> ros2
            '/world/orbit/model/nemo1/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            
        ],
        remappings=[
            ('/world/orbit/model/bean2/joint_state', 'bean/joint_states'),
            ('/world/orbit/model/nemo1/joint_state', 'nemo/joint_states')
        ],
        parameters=[{'use_sim_time':True}],
        output='screen'
    )

    return LaunchDescription([
        bean_robot_state_publisher,
        nemo_robot_state_publisher,
        bean_spawn,
        nemo_spawn_after_bean,
        bridge,
        joint_state_broadcaster
        ])

