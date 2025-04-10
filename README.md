# TE3002B Intelligent Robotics Implementation Final Challenge

## Mini-Challenge 2: Open Loop Controller

Our **Mini-Challenge 2** solutions are stored in the **'puzzlebot_control'** ROS2 package directory.

Inside, we created a node with a robust open loop controller implemented as a state machine called **square_path_ctrl** 
that enables a Puzzlebot differential drive robot to drive a square path of a side length of 2 meters. 

Additionally, another node was created named **path_generator** which generates different paths according to a user.
The paths are defined in the **/config/params.yaml** YAML file by different points, velocities or times.
For each point, the node estimates the linear and rotational speeds in case a time is given by the user in the file or
estimate the time in case the velocities are provided. It also lets the user know if the point is reachable according
to the dynamical behavior of the mobile robot and the parameters of the user. The node then passes the information to
an **open_loop_path_ctrl** node via the **/pose** topic so it can send a velocity to the Puzzlebot via the **cmd_vel** topic.

### Path Selection

To select a path for the robot to make, please view the YAML file in **/config/params.yaml** and locate your
path of interest. If the path ends with **'_time** it has defined times for each point. Meanwhile, if it ends with
**'_speed** it has defined speeds for each point. After selecting a path, substitute the value that
**default_value** has inside the **path_arg** argument for path selection in the **path_generator_launch.py** launch file:

``` 
def generate_launch_description():
    # 1. Argument to select the path
    path_arg = DeclareLaunchArgument(
        'path_name',
        default_value='square_path_speed', # Value to replace for path desired
        description='Name of the path (e.g., figure_8_path_time, square_path_speed)'
    )
``` 

To run the **path_generator** node and the **open_loop_path_ctrl** controller node, run the following command after running **colcon_build** and sourcing with
**source install/setup.bash**:

```
ros2 launch puzzlebot_control path_generator_launch.py
``` 
