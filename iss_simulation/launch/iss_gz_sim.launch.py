import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get package share directories
    robots_desc_share = get_package_share_directory('iss_simulation')
    gz_package = get_package_share_directory('ros_gz_sim')
    sim_share = get_package_share_directory('iss_simulation')

    robots_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(robots_desc_share, 'launch', 'robots_gz_bringup.launch.py')
        ])
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_package, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r ' + os.path.join(sim_share, 'worlds', 'orbit.sdf'),
            'use_sim_time':'True'
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(robots_description)
    ld.add_action(gazebo) 
    return ld
