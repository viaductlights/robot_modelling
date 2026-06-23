import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    sim_share = get_package_share_directory('iss_simulation')
    gz_package = get_package_share_directory('ros_gz_sim')
    
    # 1. Gazebo Simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_package, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r ' + os.path.join(sim_share, 'worlds', 'orbit.sdf'), 'use_sim_time': 'True'}.items(),
    )

    # 2. Include existing robot bringup (keeping logic clean)
    robots_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'robots_gz_moveit_bringup.launch.py'))
    )

    # 3. Your Task Node (with delay to ensure MoveIt and Controllers are fully ready)
    task_node = Node(
        package='task_coordinator',
        executable='sim_task',
        name='sim_task_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Delayed start for the task node
    delayed_task = TimerAction(
        period=15.0, # Adjust based on your system load
        actions=[task_node]
    )

    return LaunchDescription([
        gazebo,
        robots_bringup,
        delayed_task
    ])
