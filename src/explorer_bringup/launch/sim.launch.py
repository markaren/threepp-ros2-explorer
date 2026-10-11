"""The simulated robot on its own: the world, and the safety reflex in front of its motors.

    ros2 launch explorer_bringup sim.launch.py

Nothing drives the robot yet. Drive it yourself from a second terminal:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_nav

(The remapping sends your key presses through the safety monitor, like the planner's
commands. Try to ram a wall.)

A launch file is a Python program that describes which nodes to start and with which
parameters. `ros2 launch` runs it, starts the processes, and stops them all on Ctrl-C.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # The parameter file is installed with this package. FindPackageShare resolves to
    # install/share/explorer_bringup, wherever the workspace happens to be.
    parameters = PathJoinSubstitution([FindPackageShare('explorer_bringup'), 'config', 'explorer.yaml'])

    # A launch argument is a value the user can set on the command line:
    #   ros2 launch explorer_bringup sim.launch.py headless:=true
    # LaunchConfiguration('headless') is a placeholder for that value. It arrives as
    # text, so ParameterValue says which type the node's parameter expects.
    headless = ParameterValue(LaunchConfiguration('headless'), value_type=bool)
    screenshot_dir = LaunchConfiguration('screenshot_dir')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run the simulator without a visible window'),
        DeclareLaunchArgument('screenshot_dir', default_value='.',
                              description='Where the screenshot services write their PNG files'),

        Node(
            package='explorer_sim',
            executable='sim',
            output='screen',
            # Later entries override earlier ones: the file first, then the arguments.
            parameters=[parameters, {'headless': headless, 'screenshot_dir': screenshot_dir}],
        ),
        Node(
            package='explorer_safety',
            executable='safety_monitor',
            output='screen',
            parameters=[parameters],
        ),
    ])
