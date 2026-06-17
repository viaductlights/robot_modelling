import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    sim_share = get_package_share_directory('iss_simulation')
    bean3_desc_share = get_package_share_directory('bean3_description')
    gz_package = get_package_share_directory('ros_gz_sim')
    moveit_share = get_package_share_directory('bean2_moveit')

    append_sim_models = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(sim_share, 'models') 
    )

    append_sim_worlds = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(sim_share, 'worlds')
    )

    append_bean_share = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=bean3_desc_share
    )

    bean3_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bean3_desc_share, 'launch', 'bean3_bringup.launch.py')
        ])
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_package, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r ' + os.path.join(sim_share, 'worlds', 'testworld.sdf'),
            'use_sim_time': 'True'
        }.items(),
    )
    
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True'
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(append_sim_models)
    ld.add_action(append_sim_worlds)
    ld.add_action(append_bean_share)
    ld.add_action(bean3_description)
    ld.add_action(gazebo)
    ld.add_action(move_group)
    return ld
