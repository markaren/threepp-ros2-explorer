"""The whole demo: simulator, safety reflex, mapping, exploration, cone spotting, dashboard.

    ros2 launch explorer_bringup demo.launch.py

Arguments (name:=value on the command line):

    auto_explore   true   start exploring without waiting for the operator to press E
    dashboard      true   open the operator's 3D window (the robot's belief)
    vision         true   run the cone spotter
    rviz           false  also open RViz with a ready-made configuration
    headless       false  no visible windows (used for automated tests)

`ros2 launch explorer_bringup demo.launch.py --show-args` lists them.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('explorer_bringup')
    parameters = PathJoinSubstitution([bringup, 'config', 'explorer.yaml'])
    rviz_config = PathJoinSubstitution([bringup, 'rviz', 'explorer.rviz'])

    headless = LaunchConfiguration('headless')
    screenshot_dir = LaunchConfiguration('screenshot_dir')

    return LaunchDescription([
        DeclareLaunchArgument('auto_explore', default_value='true',
                              description='Start exploring without waiting for the operator'),
        DeclareLaunchArgument('dashboard', default_value='true',
                              description="Open the operator's 3D window"),
        DeclareLaunchArgument('vision', default_value='true',
                              description='Run the cone spotter'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Also open RViz'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run without visible windows'),
        DeclareLaunchArgument('screenshot_dir', default_value='.',
                              description='Where the screenshot services write their PNG files'),

        # The robot itself is described once, in sim.launch.py. Including it here
        # instead of repeating it means there is one place to change it.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, 'launch', 'sim.launch.py'])),
            launch_arguments={'headless': headless, 'screenshot_dir': screenshot_dir}.items(),
        ),

        # ---- the robot's brain (Python) ---------------------------------------------
        Node(package='explorer_nav', executable='mapper', output='screen', parameters=[parameters]),
        Node(package='explorer_nav', executable='explorer', output='screen', parameters=[parameters]),
        Node(package='explorer_vision', executable='cone_spotter', output='screen', parameters=[parameters],
             condition=IfCondition(LaunchConfiguration('vision'))),

        # ---- the operator's side -----------------------------------------------------
        Node(
            package='explorer_dashboard',
            executable='dashboard',
            output='screen',
            parameters=[parameters, {
                'headless': ParameterValue(headless, value_type=bool),
                'auto_explore': ParameterValue(LaunchConfiguration('auto_explore'), value_type=bool),
                'screenshot_dir': screenshot_dir,
            }],
            condition=IfCondition(LaunchConfiguration('dashboard')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
