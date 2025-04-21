from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    path_arg = DeclareLaunchArgument(
        'path_name',
        default_value='square_path_time',
        description='Nombre de la trayectoria en params.yaml'
    )

    params_file = PathJoinSubstitution([
        FindPackageShare('puzzlebot_control'),
        'config',
        'params.yaml'
    ])

    segmented_generator_node = Node(
        package='puzzlebot_control',
        executable='segmented_path_generator',
        name='segmented_path_generator_node',
        emulate_tty=True,
        parameters=[
            params_file,
            {'selected_path': LaunchConfiguration('path_name'),
             'params_file': params_file}
        ],
        output='screen'
    )

    fsm_ctrl_node = Node(
        package='puzzlebot_control',
        executable='fsm_open_loop_path_ctrl',
        name='fsm_open_loop_path_ctrl_node',
        emulate_tty=True,
        output='screen'
    )

    return LaunchDescription([
        path_arg,
        segmented_generator_node,
        fsm_ctrl_node
    ])
