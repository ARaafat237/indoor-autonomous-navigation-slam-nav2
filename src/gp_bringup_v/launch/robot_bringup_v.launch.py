import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def include_launch(package_name, *path_parts):
    launch_path = os.path.join(
        get_package_share_directory(package_name),
        *path_parts,
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
    )


def generate_launch_description():
    robot_description_file = PathJoinSubstitution([
        FindPackageShare('gp_description'),
        'urdf',
        'gp_robot.urdf.xacro',
    ])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            robot_description_file,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='log',
        parameters=[{
            'robot_description': robot_description,
        }],
    )

    velocity_manager = Node(
        package='gp_velocity_manager',
        executable='velocity_manager',
        name='gp_velocity_manager',
        output='log',
        parameters=[{
            'input_topic': '/cmd_vel',
            'output_topic': '/cmd_vel_motor',

            # Phase 4.3: slow only the final motor command near live scan
            # obstacles. Nav2 can still plan confidently; the motor layer
            # becomes cautious in narrow passages.
            # Scan-based speed scaling. Physics-derived ladder accounting for
            # LIDAR-to-bumper offset (14cm: LIDAR at x=0.04, bumper at x=0.18),
            # 200ms reaction time, and 0.40 m/s² decel. Each tier corresponds
            # to a real stopping distance — robot can come to rest before
            # bumper contact at every commanded speed. Hard stop at 20cm scan
            # = 6cm bumper clearance is the physical floor and only fires
            # when something appears that Nav2's planning didn't anticipate
            # (dynamic obstacle, low-profile object below costmap, etc.).
            'scan_speed_scaling_enable': True,
            'scan_topic': '/scan',
            'scan_speed_scaling_stale_sec': 0.75,
            'scan_speed_scaling_sector_deg': 70.0,
            'scan_speed_scaling_open_clearance_m': 0.40,
            'scan_speed_scaling_medium_clearance_m': 0.30,
            'scan_speed_scaling_tight_clearance_m': 0.25,
            'scan_speed_scaling_danger_clearance_m': 0.22,
            'scan_speed_scaling_stop_clearance_m': 0.20,
            'scan_speed_scaling_medium_scale': 0.65,
            'scan_speed_scaling_tight_scale': 0.35,
            'scan_speed_scaling_danger_scale': 0.15,
            'scan_speed_scaling_scale_angular': False,
            'scan_speed_scaling_min_angular_scale': 0.70,
            'scan_speed_scaling_stop_forward': True,

            # Final command limits before the motor driver
            'max_linear_m_s': 0.32,
            'max_angular_rad_s': 0.70,

            # C++ ramp/decision layer.
            # Restored to original baseline. 5 s ramp starved the
            # stall watchdog in scan-scaled corridors. RTFA owns the
            # rotation→forward smoothness so the chassis-wide accel
            # cap doesn't need to also throttle it.
            'max_linear_accel_m_s2': 0.35,
            'max_angular_accel_rad_s2': 2.0,

            # Ignore tiny joystick/Nav2 noise
            'linear_deadband_m_s': 0.003,
            'angular_deadband_rad_s': 0.03,

            # Nav2-safe pure-spin handling:
            # preserve the angular velocity requested by Nav2 instead of
            # forcing a larger spin that DWB did not plan for.
            'pure_spin_v_threshold': 0.03,
            'min_pure_spin_enable': False,
            'min_pure_spin_angular_rad_s': 0.35,

            # Keep startup boost off for Nav2. If low-speed manual teleop needs
            # a kick later, enable it in a separate manual profile.
            'pure_spin_start_boost_enable': False,
            'pure_spin_start_boost_angular_rad_s': 0.45,
            'pure_spin_start_boost_duration_s': 0.10,

            # When a turn command changes into straight forward motion, pause the
            # forward ramp briefly until yaw command has unwound. This prevents the
            # base from carrying turn bias into the first centimeters of forward
            # travel during controller/manual tests and Nav2 path following.
            'turn_settle_enable': True,
            'turn_settle_forward_v_threshold_m_s': 0.025,
            'turn_settle_start_w_threshold_rad_s': 0.12,
            'turn_settle_target_w_threshold_rad_s': 0.25,
            'turn_settle_done_w_threshold_rad_s': 0.05,
            'turn_settle_linear_cap_m_s': 0.02,
            'turn_settle_min_hold_s': 0.15,
            'turn_settle_timeout_s': 0.30,

            # Separate case from normal arc settling:
            # pure rotate in place leaves caster/support wheels misaligned. The
            # next forward command gets a short crawl with slight opposite yaw so
            # the support wheel can realign before normal forward speed resumes.
            # Superseded by rotation_to_forward_assist (RTFA). Kept disabled
            # so the two state machines do not stack opposite ratio profiles.
            'post_spin_realign_enable': False,
            'post_spin_realign_recent_window_s': 5.0,
            'post_spin_realign_forward_v_threshold_m_s': 0.03,
            'post_spin_realign_spin_w_threshold_rad_s': 0.12,
            'post_spin_realign_target_w_threshold_rad_s': 0.25,
            'post_spin_realign_linear_cap_m_s': 0.02,
            'post_spin_realign_support_start_ratio': 1.40,
            'post_spin_realign_support_end_ratio': 0.0,
            'post_spin_realign_support_duration_s': 1.0,
            'post_spin_realign_overshoot_kill_enable': True,
            'post_spin_realign_overshoot_kill_w_rad_s': 0.10,
            'post_spin_realign_overshoot_kill_min_v_m_s': 0.01,

            # Rotation-to-Forward Assist (RTFA): on the rising-edge transition
            # from in-place rotation to forward motion, ramp vx from a low
            # floor toward commanded over x_ramp_duration, and inject growing
            # counter-yaw (same sign as last spin, opposite to caster drift)
            # over z_assist_duration. Catches the drift window that
            # yaw_stabilization cannot — it gates on min_forward_v and is
            # disarmed during the sub-threshold startup moment.
            'rotation_to_forward_assist_enable': True,
            'rotation_to_forward_assist_spin_w_threshold_rad_s': 0.12,
            'rotation_to_forward_assist_idle_v_threshold_m_s': 0.02,
            'rotation_to_forward_assist_fire_v_threshold_m_s': 0.02,
            'rotation_to_forward_assist_arm_min_duration_s': 0.10,
            'rotation_to_forward_assist_arm_recent_window_s': 5.0,
            'rotation_to_forward_assist_x_ramp_duration_s': 2.0,
            'rotation_to_forward_assist_x_ramp_start_m_s': 0.06,
            'rotation_to_forward_assist_z_assist_duration_s': 1.5,
            'rotation_to_forward_assist_z_assist_start_ratio': 0.4,
            'rotation_to_forward_assist_z_assist_end_ratio': 0.8,
            'rotation_to_forward_assist_z_assist_max_rad_s': 0.25,
            'rotation_to_forward_assist_z_inject_max_nav_w_rad_s': 0.08,
            'rotation_to_forward_assist_abort_nav_w_rad_s': 0.20,
            'rotation_to_forward_assist_overshoot_kill_w_rad_s': 0.10,

            # Watchdog (M1.B: longer timeout absorbs TF hiccups,
            # immediate_stop off so the ramp + turn_settle + post_spin_realign
            # state machines keep their context when ticks are missed)
            'cmd_timeout_s': 0.6,
            'output_rate_hz': 50.0,
            'publish_keepalive_s': 0.20,
            'publish_linear_epsilon_m_s': 0.001,
            'publish_angular_epsilon_rad_s': 0.005,
            'immediate_stop': False,

            # M1.A: dedicated topic for gp_mission_supervisor. While the
            # supervisor is publishing here, /cmd_vel is ignored.
            'supervisor_input_topic': '/cmd_vel_supervisor',
            'supervisor_priority_enable': True,
            'supervisor_priority_timeout_s': 0.20,

            # Closed-loop yaw rate stabilization. Reads actual yaw from EKF
            # /odometry/filtered and corrects commanded yaw rate so chassis
            # actually follows intent despite caster drag and friction.
            'odom_input_topic': '/odometry/filtered',
            'yaw_stabilization_enable': True,
            'yaw_stabilization_kp': 1.5,
            'yaw_stabilization_max_correction_rad_s': 0.30,
            'yaw_stabilization_odom_stale_sec': 0.30,
            'yaw_stabilization_log_threshold_rad_s': 0.15,
            # Speed gate — disable yaw stab below this; prevents noise
            # amplification and start-from-rest instability.
            'yaw_stabilization_min_forward_v_m_s': 0.03,

            # Soft start — cap linear velocity briefly after every
            # zero→forward transition. Lets the caster swivel during the
            # gentle ramp instead of being fought by yaw stabilization.
            'soft_start_enable': True,
            'soft_start_duration_s': 0.6,
            'soft_start_linear_cap_m_s': 0.05,
            'soft_start_idle_threshold_m_s': 0.02,

            'debug': False,
        }],
    )

    motor_serial_driver = Node(
        package='motor_serial_driver',
        executable='motor_serial_driver',
        name='motor_serial_driver',
        output='log',

        # Important:
        # The motor driver must NOT listen directly to /cmd_vel anymore.
        # It listens to the cleaned command from gp_velocity_manager.
        remappings=[
            ('/cmd_vel', '/cmd_vel_motor'),
        ],

        parameters=[{
            'port': '/dev/my_motor',
            'baudrate': 115200,
            'wheel_base': 0.14,

            # Keep Python limits higher than C++ manager to avoid double clipping
            'max_linear_m_s': 0.60,
            'max_angular_rad_s': 1.50,
            'max_speed_mm_s': 700.0,

            # Channel calibration. Keep explicit defaults so sign/mapping tests
            # can be changed from launch without touching the driver code.
            'left_cmd_sign': 1.0,
            'right_cmd_sign': 1.0,
            'left_feedback_sign': 1.0,
            'right_feedback_sign': 1.0,
            'swap_command_wheels': False,
            'swap_feedback_wheels': False,

            # Keep the Python command path transparent for Nav2. Motor minimums
            # can make a gentle DWB turn execute as an aggressive spin.
            'pure_spin_angular_scale': 1.0,
            'command_deadband_mm_s': 4.0,
            'min_effective_speed_mm_s': 0.0,
            'min_turn_wheel_speed_mm_s': 0.0,
            'min_spin_wheel_speed_mm_s': 0.0,
            'turn_min_w_threshold_rad_s': 0.10,
            'left_forward_min_speed_mm_s': 0.0,
            'left_reverse_min_speed_mm_s': 0.0,
            'right_forward_min_speed_mm_s': 0.0,
            'right_reverse_min_speed_mm_s': 0.0,

            # Nav2 baseline: preserve DWB's requested curvature. Aggressive
            # wheel-side arc scaling was turning small planner corrections into
            # large physical yaw commands.
            'left_forward_cmd_scale': 1.0,
            'right_forward_cmd_scale': 1.0,
            'left_reverse_cmd_scale': 1.0,
            'right_reverse_cmd_scale': 1.0,
            'forward_turn_outer_wheel_scale': 1.0,
            'forward_turn_inner_wheel_scale': 1.0,
            'forward_left_turn_outer_wheel_scale': 1.0,
            'forward_left_turn_inner_wheel_scale': 1.0,
            'forward_right_turn_outer_wheel_scale': 1.0,
            'forward_right_turn_inner_wheel_scale': 1.0,
            'forward_turn_preserve_linear_speed': True,

            # Keep pure-spin assistance inside Nav2's planned angular range.
            # These minimums apply only to pure spin, not to forward arcs.
            'pure_spin_left_cmd_scale': 1.0,
            'pure_spin_right_cmd_scale': 1.0,
            # M1.E: lowered minimums so low-rate spin commands (0.20 rad/s ~ 14 mm/s)
            # don't get snapped to higher wheel speeds. Raise back if motor stalls.
            'pure_spin_left_min_wheel_speed_mm_s': 12.0,
            'pure_spin_right_min_wheel_speed_mm_s': 15.0,
            'max_linear_accel_m_s2': 100.0,
            'max_angular_accel_rad_s2': 100.0,

            # Keep the feedback cleanup from the fixed driver
            'straight_cmd_w_epsilon': 0.05,
            'straight_feedback_w_deadband': 0.12,
            'straight_feedback_force_zero': True,
            'straight_feedback_force_zero_min_v_m_s': 0.02,
            'feedback_smoothing_alpha': 0.50,

            'cmd_timeout': 0.5,
            'command_rate': 30.0,
            'feedback_rate': 15.0,
            'speed_command_epsilon_mm_s': 1.0,
            'speed_command_keepalive_sec': 0.10,

            'debug_commands': False,
            'debug_feedback': False,
        }],
    )

    differential_odom = Node(
        package='differential_odom',
        executable='differential_odom_node',
        name='differential_odom_node',
        output='log',
        parameters=[{
            # EKF should be the only odom -> base_footprint TF publisher.
            'publish_tf': False,
        }],
    )

    imu_driver = Node(
        package='imu_ros2_device',
        executable='ybimu_driver',
        name='ybimu_node',
        output='log',
        parameters=[{
            'frame_id': 'imu_link',
            'gyro_x_sign': 1.0,
            'gyro_y_sign': 1.0,
            'gyro_z_sign': 1.0,
            'gyro_scale': 1.0,
            'accel_scale': 1.0,
            'orientation_covariance_diagonal': [0.25, 0.25, 0.50],
            'angular_velocity_covariance_diagonal': [0.04, 0.04, 0.05],
            'linear_acceleration_covariance_diagonal': [0.20, 0.20, 0.40],
        }],
    )

    lidar_driver = include_launch(
        'rplidar_ros',
        'launch',
        'rplidar_a1_launch.py',
    )

    ekf = include_launch(
        'gp_localization',
        'launch',
        'ekf_2wd.launch.py',
    )

    return LaunchDescription([
        robot_state_publisher,
        velocity_manager,
        motor_serial_driver,
        differential_odom,
        imu_driver,
        lidar_driver,
        ekf,
    ])
