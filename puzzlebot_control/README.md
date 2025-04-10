## Activity 1: Moving a Puzzlebot

Build the program using "colcon build":

colcon build
source install/setup.bash

Connect to the Puzzlebot via Wi-Fi.
If using Gazebo, run the Gazebo simulator, start the simulation as follows:

ros2 launch puzzlebot_gazebo gazebo_example_launch.py

Open another terminal and run the activity:

ros2 run puzzlebot_control open_loop_ctrl

The robot should move forward, rotate 180 degrees, and come back to the initial position.

## Mini Challenge 1

The user can control which path is selected through the **current_path** parameter in the ROS2 parameter system.

1. **Parameter Declaration**: The **PathGeneratorNode** declares a parameter called **current_path** during initialization.

2. **Default Selection**: The default path is set to **'square_path_time'** if no user selection is provided.

3. **Parameter Retrieval**: The node retrieves the user's selection.

4. **User Control Methods**: Users can select a different path in several ways:

    - **Launch File**: The **path_generator_launch.py** launch file defines the parameter.

    - **Command Line**: Override the parameter when launching the node:

    ros2 run puzzlebot_control path_generator_node --ros-args -p current_path:=triangle_path_time

    **Parameter Service**: Change the parameter during runtime:

    ros2 param set /path_generator_node current_path figure_8_path_speed

    **Parameter Client API**: Programmatically set the parameter from another node using the ROS2 parameter client.

5. **Parameter Hierarchy**: ROS2 uses the following hierarchy to determine the final value of parameters.

    - **Default Value**: The value specified in **declare_parameters()**, 'square_path_time' in this case.

    - **YAML File**: Values loaded from the /config/params.yaml file override defaults.
    If the file contains **'current_path'** then the path name assigned to it will be used.

    - **Launch File**: Parameters set in launch files override YAML files.

    - **Command Line**: Parameters set via command line override all previous sources.

    To test other values aside from the default one, use all other methods. The file is loaded by the ROS2 parameter system, which happens by running the launch file or via the command line. This way, the parameters become available to the node and it accesses the parameters through the ROS2 parameter API.

