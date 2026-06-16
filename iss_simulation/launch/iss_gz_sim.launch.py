import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    sim_share = get_package_share_directory('iss_simulation')
    bean3_desc_share = get_package_share_directory('bean3_description')
    gz_package = get_package_share_directory('ros_gz_sim')

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
        value=os.path.join(bean3_desc_share)
    )

    #bean3 description
    bean3_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('bean3_description'), 'launch', 'bean3_gz.launch.py')
            ])
        )

    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(gz_package, 'launch', 'gz_sim.launch.py')),launch_arguments={'gz_args': '-r spacecraft.sdf','use_sim_time':'True'}.items(),)

    return LaunchDescription([
        append_sim_models,
        append_sim_worlds,
        bean3_description,
        gazebo
    ])

