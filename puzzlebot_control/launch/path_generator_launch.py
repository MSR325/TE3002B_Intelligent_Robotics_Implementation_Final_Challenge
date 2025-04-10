from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Argument to select the path
    path_arg = DeclareLaunchArgument(
        'path_name',
        default_value='square_path_time',
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
        emulate_tty=True,
        parameters=[params_file, {'selected_path': LaunchConfiguration('path_name')}],
        output='screen'
    )

    # 4. Open loop path controller that receives path points and converts them to velocity commands
    open_loop_path_ctrl_node = Node(
        package='puzzlebot_control',
        executable='open_loop_path_ctrl',
        name='open_loop_path_ctrl_node',
        emulate_tty=True,
        parameters=[{
                    'linear_tolerance': 0.05,  # meters
                    'angular_tolerance': 0.1,  # radians
                    'max_linear_vel': 0.5,    # m/s
                    'max_angular_vel': 1.0  # rad/s
        }],
        output='screen'
    )

    return LaunchDescription([
        path_arg,
        path_generator_node,
        open_loop_path_ctrl_node
    ])