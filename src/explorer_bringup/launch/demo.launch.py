"""The whole demo: simulator, safety reflex, mapping, exploration, cone spotting, dashboard.

    ros2 launch explorer_bringup demo.launch.py

Arguments (name:=value on the command line):

    auto_explore   true   start exploring without waiting for the operator to press E
    mapper         simple   who builds the map: simple (our own mapper_node.py) or
                            slam_toolbox (the SLAM package, which also corrects the odometry)
    navigation     simple   who plans the path and drives along it: simple (our own
                            planner.py and pure_pursuit.py, inside the explorer) or nav2
                            (the Nav2 stack; the explorer then only picks the frontier)
    dashboard      true   open the operator's 3D window (the robot's belief)
    vision         true   run the cone spotter
    rviz           false  also open RViz with a ready-made layout. With navigation:=nav2
                          the layout also shows Nav2's costmaps, footprint and carrot
    effects        true   ambient occlusion, bloom, floor reflections (false: faster)
    view           overview   the simulator's camera at start: overview, follow, chase
    headless       false  no visible windows (used for automated tests)
    screenshot_dir .      where the screenshot services write their PNG files

`ros2 launch explorer_bringup demo.launch.py --show-args` lists them.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (EqualsSubstitution, IfElseSubstitution, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('explorer_bringup')
    parameters = PathJoinSubstitution([bringup, 'config', 'explorer.yaml'])
    slam_parameters = PathJoinSubstitution([bringup, 'config', 'slam_toolbox.yaml'])
    nav2_parameters = PathJoinSubstitution([bringup, 'config', 'nav2.yaml'])

    headless = LaunchConfiguration('headless')
    screenshot_dir = LaunchConfiguration('screenshot_dir')
    mapper = LaunchConfiguration('mapper')
    with_nav2 = EqualsSubstitution(LaunchConfiguration('navigation'), 'nav2')
    # RViz shows topics, and Nav2 brings topics of its own. So there are two layouts,
    # and the one that fits what is running is opened.
    rviz_config = PathJoinSubstitution([bringup, 'rviz', IfElseSubstitution(
        with_nav2, if_value='explorer_nav2.rviz', else_value='explorer.rviz')])

    return LaunchDescription([
        DeclareLaunchArgument('auto_explore', default_value='true',
                              description='Start exploring without waiting for the operator'),
        DeclareLaunchArgument('mapper', default_value='simple', choices=['simple', 'slam_toolbox'],
                              description='Who builds the map: our own mapper, or slam_toolbox'),
        DeclareLaunchArgument('navigation', default_value='simple', choices=['simple', 'nav2'],
                              description='Who plans and drives: our own explorer, or Nav2'),
        DeclareLaunchArgument('dashboard', default_value='true',
                              description="Open the operator's 3D window"),
        DeclareLaunchArgument('vision', default_value='true',
                              description='Run the cone spotter'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Also open RViz'),
        DeclareLaunchArgument('effects', default_value='true',
                              description='Ambient occlusion, bloom and floor reflections in the simulator'),
        DeclareLaunchArgument('view', default_value='overview',
                              description="The simulator's camera at start: overview, follow or chase"),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run without visible windows'),
        DeclareLaunchArgument('screenshot_dir', default_value='.',
                              description='Where the screenshot services write their PNG files'),

        # The robot itself is described once, in sim.launch.py. Including it here
        # instead of repeating it means there is one place to change it.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, 'launch', 'sim.launch.py'])),
            launch_arguments={
                'headless': headless,
                'effects': LaunchConfiguration('effects'),
                'view': LaunchConfiguration('view'),
                'screenshot_dir': screenshot_dir,
            }.items(),
        ),

        # ---- the robot's brain (Python) ---------------------------------------------
        # Who builds the map is a choice. Both candidates read `scan` and tf, and both
        # publish `map` and the transform map -> odom, so nobody downstream can tell them
        # apart by the interface. Only one of the two is started.
        Node(package='explorer_nav', executable='mapper', output='screen', parameters=[parameters],
             condition=IfCondition(EqualsSubstitution(mapper, 'simple'))),
        # slam_toolbox brings its own launch file: it is a lifecycle node, which has to be
        # configured and activated before it works, and its launch file does that. We
        # include it and hand it our parameter file.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])),
            launch_arguments={
                'slam_params_file': slam_parameters,
                'use_sim_time': 'false',     # the simulator stamps with the computer's clock
            }.items(),
            condition=IfCondition(EqualsSubstitution(mapper, 'slam_toolbox')),
        ),
        # The explorer always decides WHERE to go. With navigation:=nav2 it leaves the HOW
        # to Nav2: it sends each target as a goal of Nav2's navigate_to_pose action
        # instead of planning a path and following it (nav2_driver.py).
        Node(package='explorer_nav', executable='explorer', output='screen',
             parameters=[parameters, {'use_nav2': ParameterValue(with_nav2, value_type=bool)}]),

        # ---- Nav2, when it plans and drives ------------------------------------------
        # Four servers and the manager that brings them to life, see config/nav2.yaml.
        # Three remappings fit them into this graph:
        #   cmd_vel -> cmd_vel_nav   Nav2's commands pass through our safety monitor, like
        #                            the explorer's own. Nothing reaches the motors unseen.
        #   goal_pose -> elsewhere   Nav2 would also drive to every goal_pose it hears. In
        #                            this demo the explorer is the only one who gives Nav2
        #                            goals, so the dashboard's G key still goes through it.
        GroupAction(condition=IfCondition(with_nav2), actions=[
            Node(package='nav2_planner', executable='planner_server', name='planner_server',
                 output='screen', parameters=[nav2_parameters]),
            Node(package='nav2_controller', executable='controller_server', name='controller_server',
                 output='screen', parameters=[nav2_parameters],
                 remappings=[('cmd_vel', 'cmd_vel_nav')]),
            Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
                 output='screen', parameters=[nav2_parameters],
                 remappings=[('cmd_vel', 'cmd_vel_nav')]),
            Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
                 output='screen', parameters=[nav2_parameters],
                 remappings=[('goal_pose', 'nav2_goal_pose')]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_navigation', output='screen',
                 parameters=[nav2_parameters]),
        ]),
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
