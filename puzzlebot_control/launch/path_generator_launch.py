from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Argument to select the path
    path_arg = DeclareLaunchArgument(
        'path_name',
        default_value='square_path_speed',
        description='Name of the path (e.g., figure_8_path_time, square_path_speed)'
    )

    # 2. Path to the YAML parameters file
    params_file = PathJoinSubstitution([
        FindPackageShare('puzzlebot_control'),  # Replace with your package
        'config',
        'params.yaml'
    ])

    # 3. Node that loads the selected path
    path_generator_node = Node(
        package='puzzlebot_control',  # Replace with your package
        executable='path_generator',
        name='path_generator_node',
        parameters=[params_file, {'selected_path': LaunchConfiguration('path_name')}],
        output='screen'
    )

    return LaunchDescription([
        path_arg,
        path_generator_node
    ])