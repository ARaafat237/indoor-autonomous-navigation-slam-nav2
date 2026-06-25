import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    gp_nav2_dir = get_package_share_directory('gp_nav2')

    default_map = '/root/maps/current_nav_map.yaml'
    default_params = os.path.join(gp_nav2_dir, 'config', 'gp_nav2_params.yaml')
    default_nav_to_pose_bt_xml = os.path.join(
        gp_nav2_dir,
        'behavior_trees',
        'gp_navigate_to_pose.xml',
    )
    default_supervisor_params = os.path.join(
        gp_nav2_dir,
        'config',
        'gp_mission_supervisor.yaml',
    )
    default_supervisor_bt_xml = os.path.join(
        gp_nav2_dir,
        'behavior_trees',
        'gp_navigate_to_pose_supervised.xml',
    )

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    nav_to_pose_bt_xml = LaunchConfiguration('nav_to_pose_bt_xml')
    supervisor_params_file = LaunchConfiguration('supervisor_params_file')
    supervisor_bt_xml = LaunchConfiguration('supervisor_bt_xml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    use_supervisor = LaunchConfiguration('use_supervisor')
    mission_goal_topic = LaunchConfiguration('mission_goal_topic')
    rviz_goal_topic = LaunchConfiguration('rviz_goal_topic')
    log_level = LaunchConfiguration('log_level')

    # Keep this launch clean and explicit:
    # - map_server + AMCL are included for saved-map navigation
    # - behavior_server is used for ROS 2 Humble, not old recoveries_server
    # - velocity_smoother publishes directly to /cmd_vel
    # - collision_monitor is intentionally not launched during NAV2 tuning
    # - waypoint_follower and smoother_server are intentionally not launched for simple point-to-point testing

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml,
        'default_nav_to_pose_bt_xml': nav_to_pose_bt_xml,
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    lifecycle_nodes = [
        'map_server',
        'amcl',
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
    ]
    
    mission_supervisor_v2 = Node(
        package='gp_nav2',
        executable='gp_mission_supervisor_v2.py',
        name='gp_mission_supervisor',
        condition=IfCondition(use_supervisor),
        output='screen',
        parameters=[
            supervisor_params_file,
            {
                'use_sim_time': use_sim_time,
                'nav_behavior_tree': supervisor_bt_xml,
                'mission_goal_topic': mission_goal_topic,
                'rviz_goal_topic': rviz_goal_topic,
            },
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    plain_goal_relay = Node(
        package='gp_nav2',
        executable='gp_goal_pose_relay.py',
        name='gp_plain_goal_relay',
        condition=UnlessCondition(use_supervisor),
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_frame': 'map',
            'mission_goal_topic': mission_goal_topic,
            'rviz_goal_topic': rviz_goal_topic,
            'nav_action_name': '/navigate_to_pose',
            'nav_behavior_tree': nav_to_pose_bt_xml,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to saved map YAML'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to Nav2 params file'
        ),

        DeclareLaunchArgument(
            'nav_to_pose_bt_xml',
            default_value=default_nav_to_pose_bt_xml,
            description='Default plain Nav2 NavigateToPose behavior tree'
        ),

        DeclareLaunchArgument(
            'supervisor_params_file',
            default_value=default_supervisor_params,
            description='Full path to gp mission supervisor v2 params file'
        ),

        DeclareLaunchArgument(
            'supervisor_bt_xml',
            default_value=default_supervisor_bt_xml,
            description='Full path to the supervised NavigateToPose behavior tree'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically configure and activate Nav2 lifecycle nodes'
        ),

        DeclareLaunchArgument(
            'use_respawn',
            default_value='false',
            description='Respawn crashed Nav2 nodes. Keep false while tuning/debugging.'
        ),

        DeclareLaunchArgument(
            'use_supervisor',
            default_value='true',
            description='Launch gp_mission_supervisor_v2 with Nav2'
        ),

        DeclareLaunchArgument(
            'mission_goal_topic',
            default_value='/mission_goal',
            description='Mission goal topic consumed by gp_mission_supervisor_v2'
        ),

        DeclareLaunchArgument(
            'rviz_goal_topic',
            default_value='/goal_pose',
            description='RViz goal topic consumed by gp_mission_supervisor_v2'
        ),

        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level'
        ),

        # ---------- Map + localization ----------
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='log',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='log',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
        ),

        # ---------- Planner + controller ----------
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='log',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            remappings=[
                # Raw controller output goes to the velocity smoother, not directly to motors.
                ('cmd_vel', 'cmd_vel_nav'),
            ],
            arguments=['--ros-args', '--log-level', log_level],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='log',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
        ),

        # ROS 2 Humble uses nav2_behaviors/behavior_server.
        # Do not use old nav2_recoveries/recoveries_server here.
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='log',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='log',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
        ),

        # ---------- Command smoothing ----------
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='log',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            remappings=[
                # Input from controller_server.
                ('cmd_vel', 'cmd_vel_nav'),

                # Output directly to gp_velocity_manager on /cmd_vel.
                ('cmd_vel_smoothed', 'cmd_vel'),
            ],
            arguments=['--ros-args', '--log-level', log_level],
        ),

        # ---------- Lifecycle manager ----------
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='log',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
                'bond_timeout': 4.0,
                'attempt_respawn_reconnection': True,
            }],
            arguments=['--ros-args', '--log-level', log_level],
        ),
        plain_goal_relay,
        mission_supervisor_v2,
    ])
