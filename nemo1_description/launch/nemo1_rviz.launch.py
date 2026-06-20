from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the package share directory
    roboarm_models_share = FindPackageShare('nemo1_description')
    
    # Set default paths
    default_model_path = PathJoinSubstitution([roboarm_models_share, 'urdf', 'nemo1.urdf'])
    default_rviz_config_path = PathJoinSubstitution([roboarm_models_share, 'rviz', 'nemo_renamed.rviz'])
    
    # Declare launch arguments
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )
    
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot URDF file'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file'
    )
    
    # Robot state publisher node (always runs)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[LaunchConfiguration('model')]
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
    
    # Rviz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add actions
    ld.add_action(gui_arg)
    ld.add_action(model_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz_node)
    
    return ld

