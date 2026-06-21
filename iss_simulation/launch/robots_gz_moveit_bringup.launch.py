import os
import xacro

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # --------------------
    # define ros2 packages and paths
    # --------------------
    bean_package_name = 'bean3_description'
    bean_robot_package = get_package_share_directory(bean_package_name)
    bean_moveit_package = get_package_share_directory('bean2_moveit')
    bean_ros2_controllers_yaml = os.path.join(bean_moveit_package, 'config', 'ros2_controllers.yaml')

    gz_package = get_package_share_directory('ros_gz_sim')
    sim_package = get_package_share_directory('iss_simulation')
    default_rviz_config_path = PathJoinSubstitution([sim_package, 'rviz', 'rviz_motion_planning.rviz'])
    
    nemo_package_name = 'nemo1_description'
    nemo_robot_package = get_package_share_directory(nemo_package_name)
    nemo_moveit_package = get_package_share_directory('nemo1_moveit')
    nemo_ros2_controllers_yaml = os.path.join(nemo_moveit_package, 'config', 'ros2_controllers.yaml')

   
    # GZ environment variable for remapping directory paths
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.path.join(os.path.join(get_package_prefix(bean_package_name), "share")) + ':' + os.path.join(os.path.join(get_package_prefix(nemo_package_name), "share"))

    # --------------------
    # robot description
    # --------------------

    # define bean urdf
    bean_urdf_file = os.path.join(bean_moveit_package, 'config', 'bean2.urdf.xacro')
    bean_robot_description_config = xacro.process_file(
            bean_urdf_file,
            mappings={'initial_positions_file':
                      os.path.join(bean_moveit_package, 'config', 'initial_positions.yaml')}
    )
    bean_robot_description = {'robot_description': bean_robot_description_config.toxml()}

    # define nemo urdf
    nemo_urdf_file = os.path.join(nemo_moveit_package, 'config', 'nemo1.urdf.xacro')
    nemo_robot_description_config = xacro.process_file(
            nemo_urdf_file,
            mappings={'initial_positions_file':
                      os.path.join(nemo_moveit_package, 'config', 'initial_positions.yaml')}
    )
    nemo_robot_description = {'robot_description': nemo_robot_description_config.toxml()}

    # rviz2 is a single, unnamespaced node shared by both MotionPlanning
    # displays - 'robot_description' can only hold one value on it, so
    # each robot needs a distinctly-named copy here. Each display's
    # "Robot Description" field in the .rviz file must be set to match
    # these exact names (e.g. "bean_robot_description" for bean's
    # display); MoveIt derives the SRDF parameter name automatically by
    # appending "_semantic" to whatever that field says.
    bean_robot_description_for_rviz = {'bean_robot_description': bean_robot_description_config.toxml()}
    nemo_robot_description_for_rviz = {'nemo_robot_description': nemo_robot_description_config.toxml()}

    bean_srdf_file = os.path.join(bean_moveit_package, 'config', 'bean2.srdf')
    with open(bean_srdf_file, 'r') as f:
        bean_robot_description_semantic = {'bean_robot_description_semantic': f.read()}

    nemo_srdf_file = os.path.join(nemo_moveit_package, 'config', 'nemo1.srdf')
    with open(nemo_srdf_file, 'r') as f:
        nemo_robot_description_semantic = {'nemo_robot_description_semantic': f.read()}

    # --------------------
    # moveit configs
    # --------------------
    bean_moveit_config = MoveItConfigsBuilder('bean2', package_name='bean2_moveit').to_moveit_configs()
    nemo_moveit_config = MoveItConfigsBuilder('nemo1', package_name='nemo1_moveit').to_moveit_configs()

    # Same collision as robot_description/robot_description_semantic:
    # rviz2 is one shared unnamespaced node, so each robot's kinematics
    # config needs its own distinct key too, matching the "Robot
    # Description" field set on each MotionPlanning display
    # (bean_robot_description -> RobotModelLoader looks for
    # bean_robot_description_kinematics automatically).
    bean_robot_description_kinematics_for_rviz = {
        'bean_robot_description_kinematics':
            bean_moveit_config.robot_description_kinematics['robot_description_kinematics']
    }
    nemo_robot_description_kinematics_for_rviz = {
        'nemo_robot_description_kinematics':
            nemo_moveit_config.robot_description_kinematics['robot_description_kinematics']
    }

    # --------------------
    # launch args
    # --------------------
 
    # rviz argument
    rviz_config_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file'
    )

    # --------------------
    # robot state publishers 
    # --------------------
 
    # bean robot state publisher
    bean_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace= "bean",
        output='both',
        parameters=[bean_robot_description,{'use_sim_time':True}],
    )

    # nemo robot state publisher
    nemo_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace= "nemo",
        output='both',
        parameters=[nemo_robot_description,{'use_sim_time':True}],
    )
    
    # --------------------
    # gz spawn
    # --------------------
 
    # bean
    bean_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        parameters=[{
            'name': 'bean2',
            'topic': 'bean/robot_description',
            'use_sim_time':True}],
        output='screen',
    )

    # nemo
    nemo_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        parameters=[{
            'name': 'nemo1',
            'topic': 'nemo/robot_description',
            'use_sim_time':True}],
        output='screen',
    )

    nemo_spawn_after_bean = RegisterEventHandler(
        OnProcessExit(
            target_action=bean_spawn,
            on_exit=[nemo_spawn],
        )
    )

    # --------------------
    # gz ros2 bridge
    # --------------------
 
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

    # --------------------
    # controller spawners 
    # --------------------
 
    # bean joint state broadcaster
    bean_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/bean/controller_manager'],
        parameters=[{'use_sim_time':True}],
    )

    # nemo joint state broadcaster
    nemo_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/nemo/controller_manager'],
        parameters=[{'use_sim_time':True}],
    )

    # bean robot controller
    bean_robot_controller = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['robot_controller', '--param-file', bean_ros2_controllers_yaml, '--controller-manager', '/bean/controller_manager'],
            parameters=[{'use_sim_time':True}],
    )

    # nemo robot controller
    nemo_robot_controller = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['robot_controller', '--param-file', nemo_ros2_controllers_yaml, '--controller-manager', '/nemo/controller_manager'],
            parameters=[{'use_sim_time':True}],
    )

    # --------------------
    # move group
    # --------------------

    bean_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace='bean',
        output='screen',
        parameters=[
            bean_moveit_config.to_dict(),
            bean_robot_description,
            {'use_sim_time': True},
        ],
    )

    nemo_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace='nemo',
        output='screen',
        parameters=[
            nemo_moveit_config.to_dict(),
            nemo_robot_description,
            {'use_sim_time': True},
        ],
    )

    # -------------------
    # tf
    # -------------------

    # Removed: bean_tf / nemo_tf static_transform_publishers.
    # robot_state_publisher already publishes world->bean_base_link and
    # world->nemo_base_link on /tf_static (the real ISS-mount offsets
    # baked into each urdf's fixedFrameToWorld joint). These manual
    # identity-transform publishers were claiming the same edge on the
    # same global /tf_static topic with a different value, which is a
    # direct conflict, not a fallback.
    
    # --------------------
    # rviz
    # --------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[
            {'use_sim_time': True},
            bean_robot_description_for_rviz,
            bean_robot_description_semantic,
            bean_robot_description_kinematics_for_rviz,
            nemo_robot_description_for_rviz,
            nemo_robot_description_semantic,
            nemo_robot_description_kinematics_for_rviz,
        ],
    )

    # --------------------
    # sequencing
    # --------------------

    nemo_spawn_after_bean = RegisterEventHandler(
        OnProcessExit(target_action=bean_spawn, on_exit=[nemo_spawn])
    )

    # Both robots' controller/move_group startup is chained off nemo_spawn's
    # exit specifically - not bean_spawn's. nemo_spawn only fires after
    # bean_spawn exits, so it's the LAST entity-creation event in the
    # chain. Previously bean_followups fired on bean_spawn's exit, the
    # same event that triggers nemo_spawn - meaning bean's controllers
    # tried to activate at the exact moment nemo's (heavy) entity
    # creation was starting in the same Gazebo process, which can stall
    # the physics step gz_ros2_control's controller_manager update()
    # rides on, intermittently blowing past controller_manager's 5s
    # switch-controller window. Waiting until both entities exist before
    # starting any controller activation removes that contention.
    all_followups = RegisterEventHandler(
        OnProcessExit(
            target_action=nemo_spawn,
            on_exit=[
                bean_joint_state_broadcaster, bean_robot_controller, bean_move_group_node,
                nemo_joint_state_broadcaster, nemo_robot_controller, nemo_move_group_node,
            ],
        )
    )

    return LaunchDescription([
        rviz_config_arg,
        bean_robot_state_publisher,
        nemo_robot_state_publisher,
        bridge,
        bean_spawn,
        nemo_spawn_after_bean,
        all_followups,
        rviz_node
        ])
