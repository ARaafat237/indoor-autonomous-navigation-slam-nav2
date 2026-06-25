#!/usr/bin/env python3

import math
from enum import Enum
from typing import Optional, Sequence, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

try:
    from gp_nav2.gp_route_memory_manager import (
        GpRouteMemoryManager,
        RouteMemoryConfig,
    )
except ImportError:
    from gp_route_memory_manager import (
        GpRouteMemoryManager,
        RouteMemoryConfig,
    )


class MissionState(Enum):
    IDLE = "IDLE"
    START_MISSION = "START_MISSION"
    NAVIGATING_TO_GOAL = "NAVIGATING_TO_GOAL"
    REPLAN_SAME_GOAL = "REPLAN_SAME_GOAL"
    REVERSE_ESCAPE_PRECHECK = "REVERSE_ESCAPE_PRECHECK"
    REVERSE_ESCAPE_ACTIVE = "REVERSE_ESCAPE_ACTIVE"
    WAIT_AFTER_REVERSE = "WAIT_AFTER_REVERSE"
    RETRY_GOAL_AFTER_REVERSE = "RETRY_GOAL_AFTER_REVERSE"
    GOAL_PROBE_ACTIVE = "GOAL_PROBE_ACTIVE"
    WAIT_AFTER_GOAL_PROBE = "WAIT_AFTER_GOAL_PROBE"
    RETRY_GOAL_AFTER_GOAL_PROBE = "RETRY_GOAL_AFTER_GOAL_PROBE"
    CONTROLLER_PATIENCE_ACTIVE = "CONTROLLER_PATIENCE_ACTIVE"
    WAIT_AFTER_CONTROLLER_PATIENCE = "WAIT_AFTER_CONTROLLER_PATIENCE"
    RETRY_GOAL_AFTER_CONTROLLER_PATIENCE = "RETRY_GOAL_AFTER_CONTROLLER_PATIENCE"
    SELECT_SIDE_ESCAPE = "SELECT_SIDE_ESCAPE"
    NAVIGATING_TO_SIDE_ESCAPE = "NAVIGATING_TO_SIDE_ESCAPE"
    DIRECT_SIDE_ESCAPE_ACTIVE = "DIRECT_SIDE_ESCAPE_ACTIVE"
    WAIT_AFTER_DIRECT_SIDE_ESCAPE = "WAIT_AFTER_DIRECT_SIDE_ESCAPE"
    RETRY_GOAL_FROM_SIDE_ESCAPE = "RETRY_GOAL_FROM_SIDE_ESCAPE"
    SELECT_BREADCRUMB = "SELECT_BREADCRUMB"
    NAVIGATING_TO_BREADCRUMB = "NAVIGATING_TO_BREADCRUMB"
    RETRY_GOAL_FROM_BREADCRUMB = "RETRY_GOAL_FROM_BREADCRUMB"
    PREPARE_RETURN_TO_BASE = "PREPARE_RETURN_TO_BASE"
    REVERSE_ESCAPE_FOR_BASE = "REVERSE_ESCAPE_FOR_BASE"
    NAVIGATING_TO_BASE = "NAVIGATING_TO_BASE"
    SUCCEEDED = "SUCCEEDED"
    RETURNED_TO_BASE = "RETURNED_TO_BASE"
    WAIT_FOR_LOCALIZATION_STABILITY = "WAIT_FOR_LOCALIZATION_STABILITY"
    FAILED = "FAILED"


class NavPurpose:
    GOAL = "goal"
    REPLAN = "replan"
    RETRY_AFTER_CONTROLLER_PATIENCE = "retry_after_controller_patience"
    RETRY_AFTER_REVERSE = "retry_after_reverse"
    RETRY_AFTER_GOAL_PROBE = "retry_after_goal_probe"
    SIDE_ESCAPE = "side_escape"
    RETRY_FROM_SIDE_ESCAPE = "retry_from_side_escape"
    BREADCRUMB = "breadcrumb"
    RETRY_FROM_BREADCRUMB = "retry_from_breadcrumb"
    BASE = "base"


class ReverseContext:
    GOAL = "goal"
    BASE = "base"


class ReversePhase:
    IDLE = "idle"
    ZERO_BEFORE = "zero_before"
    ACTIVE = "active"
    ZERO_AFTER = "zero_after"
    WAIT_AFTER = "wait_after"


class DirectSideEscapePhase:
    IDLE = "idle"
    ACTIVE = "active"
    ZERO_AFTER = "zero_after"
    WAIT_AFTER = "wait_after"


class GoalProbePhase:
    IDLE = "idle"
    ACTIVE = "active"
    ZERO_AFTER = "zero_after"
    WAIT_AFTER = "wait_after"


class ControllerPatiencePhase:
    IDLE = "idle"
    ACTIVE = "active"
    ZERO_AFTER = "zero_after"
    WAIT_AFTER = "wait_after"


class GpMissionSupervisorV2(Node):
    """
    Mission supervisor above Nav2.

    Nav2 still owns planning and path following. This node owns mission recovery:
    replan once, reverse directly through /cmd_vel when safe, step sideways out
    of the local trap, retry the original goal, retreat to route-memory
    breadcrumbs, then return to base as the final fallback.
    """

    def __init__(self):
        super().__init__("gp_mission_supervisor")

        self._declare_parameters()
        self._read_parameters()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action_name)

        self.route_memory = GpRouteMemoryManager(
            RouteMemoryConfig(
                breadcrumb_distance=self.breadcrumb_distance,
                max_breadcrumbs=self.max_breadcrumbs,
                min_retreat_distance=self.min_retreat_distance,
                preferred_retreat_distance=self.preferred_retreat_distance,
                max_retreat_distance=self.max_retreat_distance,
                failed_zone_radius=self.failed_zone_radius,
                failed_zone_lifetime_sec=self.failed_zone_lifetime_sec,
                same_failure_distance=self.same_failure_distance,
                marker_frame=self.map_frame,
            )
        )

        self.mission_goal_sub = self.create_subscription(
            PoseStamped,
            self.mission_goal_topic,
            lambda msg: self._on_new_goal(msg, "mission_goal"),
            10,
        )
        self.rviz_goal_sub = self.create_subscription(
            PoseStamped,
            self.rviz_goal_topic,
            lambda msg: self._on_new_goal(msg, "rviz_goal"),
            10,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._on_scan,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._on_odom,
            20,
        )

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, self.mission_status_topic, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.mission_marker_topic,
            10,
        )

        self.state = MissionState.IDLE
        self.mission_id = 0
        self.goal_sequence_id = 0

        self.base_pose: Optional[PoseStamped] = None
        self.original_goal: Optional[PoseStamped] = None

        self.active_goal_handle = None
        self.active_goal_mission_id: Optional[int] = None
        self.active_goal_sequence_id: Optional[int] = None
        self.active_goal_purpose: Optional[str] = None
        self.active_goal_pose: Optional[PoseStamped] = None
        self.active_goal_start_time_sec: Optional[float] = None
        self.cancel_in_progress = False
        self.active_goal_cancel_reason: Optional[str] = None

        self.same_goal_replans = 0
        self.reverse_escapes_this_stuck_event = 0
        self.reverse_escapes_this_mission = 0
        self.goal_retries_after_reverse = 0
        self.phase5_goal_retries_this_stuck_event = 0
        self.goal_probe_attempts_this_stuck_event = 0
        self.goal_probe_attempts_this_mission = 0
        self.controller_patience_attempts_this_stuck_event = 0
        self.controller_patience_attempts_this_mission = 0
        self.controller_patience_retry_fallback = "reverse"
        self.side_escape_attempts_this_stuck_event = 0
        self.direct_side_escape_attempts_this_stuck_event = 0
        self.breadcrumb_attempts = 0
        self.total_recovery_cycles = 0
        self.failed_retreat_poses = []
        self.failed_escape_poses = []
        self.last_failed_pose: Optional[PoseStamped] = None
        self.last_goal_preemption_sec = -math.inf
        self.goal_corridor_clear_ticks = 0
        self.last_goal_retry_failure_pose: Optional[PoseStamped] = None
        self.last_goal_retry_failure_sec = -math.inf

        self.stall_reference_pose: Optional[PoseStamped] = None
        self.stall_reference_time_sec: Optional[float] = None
        self.stall_grace_until_sec: Optional[float] = None
        self.last_failure_reason_was_stall = False

        self.latest_scan: Optional[LaserScan] = None
        self.latest_scan_time_sec: Optional[float] = None
        self.latest_odom: Optional[Odometry] = None
        self.latest_odom_time_sec: Optional[float] = None

        self.reverse_phase = ReversePhase.IDLE
        self.reverse_context: Optional[str] = None
        self.reverse_mission_id: Optional[int] = None
        self.reverse_phase_start_sec: Optional[float] = None
        self.reverse_active_start_sec: Optional[float] = None
        self.reverse_start_odom_xy: Optional[Tuple[float, float]] = None
        self.reverse_start_tf_pose: Optional[PoseStamped] = None
        self.reverse_allowed_duration = 0.0
        self.reverse_allowed_distance = 0.0
        self.reverse_stop_reason = ""

        self.direct_side_phase = DirectSideEscapePhase.IDLE
        self.direct_side_mission_id: Optional[int] = None
        self.direct_side_phase_start_sec: Optional[float] = None
        self.direct_side_active_start_sec: Optional[float] = None
        self.direct_side_command = Twist()
        self.direct_side_heading_rad = 0.0
        self.direct_side_stop_reason = ""

        self.goal_probe_phase = GoalProbePhase.IDLE
        self.goal_probe_mission_id: Optional[int] = None
        self.goal_probe_phase_start_sec: Optional[float] = None
        self.goal_probe_active_start_sec: Optional[float] = None
        self.goal_probe_command = Twist()
        self.goal_probe_heading_rad = 0.0
        self.goal_probe_stop_reason = ""

        self.controller_patience_phase = ControllerPatiencePhase.IDLE
        self.controller_patience_mission_id: Optional[int] = None
        self.controller_patience_phase_start_sec: Optional[float] = None
        self.controller_patience_active_start_sec: Optional[float] = None
        self.controller_patience_command = Twist()
        self.controller_patience_heading_rad = 0.0
        self.controller_patience_stop_reason = ""

        self.localization_unstable_until_sec = 0.0
        self.localization_stable_samples = 0
        self.localization_resume_purpose: Optional[str] = None
        self.last_localization_map_pose: Optional[PoseStamped] = None
        self.last_localization_odom_xy: Optional[Tuple[float, float]] = None
        self.last_localization_sample_sec: Optional[float] = None

        self.last_marker_publish_sec = -math.inf

        timer_period = 1.0 / max(self.supervisor_loop_rate, 1.0)
        recovery_cmd_period = 1.0 / max(self.reverse_publish_rate, 1.0)
        self.timer = self.create_timer(timer_period, self._on_timer)
        self.recovery_cmd_timer = self.create_timer(
            recovery_cmd_period,
            self._on_recovery_cmd_timer,
        )

        self._publish_status("IDLE")
        self.get_logger().info("gp_mission_supervisor_v2 is ready.")
        self.get_logger().info(f"Mission goal topic: {self.mission_goal_topic}")
        self.get_logger().info(f"RViz goal topic: {self.rviz_goal_topic}")
        self.get_logger().info(f"Reverse escape publishes to: {self.cmd_vel_topic}")

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    def _declare_parameters(self):
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")

        self.declare_parameter("mission_goal_topic", "/mission_goal")
        self.declare_parameter("rviz_goal_topic", "/goal_pose")
        self.declare_parameter("mission_status_topic", "/mission_status")
        self.declare_parameter("mission_marker_topic", "/gp_mission_markers")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("nav_action_name", "/navigate_to_pose")
        self.declare_parameter("nav_behavior_tree", "")

        self.declare_parameter("goal_timeout_sec", 120.0)
        self.declare_parameter("breadcrumb_timeout_sec", 45.0)
        self.declare_parameter("side_escape_goal_timeout_sec", 20.0)
        self.declare_parameter("return_to_base_timeout_sec", 120.0)
        self.declare_parameter("supervisor_loop_rate", 5.0)

        self.declare_parameter("goal_retry_preemption_enabled", True)
        self.declare_parameter("goal_retry_preemption_min_active_sec", 0.45)
        self.declare_parameter("goal_retry_preemption_cooldown_sec", 2.5)
        self.declare_parameter("goal_retry_preemption_clearance", 0.32)
        self.declare_parameter("goal_retry_preemption_front_clearance", 0.28)
        self.declare_parameter("goal_retry_preemption_sector_degrees", 40.0)
        self.declare_parameter("goal_retry_preemption_max_heading_error_degrees", 60.0)
        self.declare_parameter("goal_retry_preemption_required_stable_ticks", 2)
        self.declare_parameter("goal_retry_failure_cooldown_sec", 8.0)
        self.declare_parameter("goal_retry_failure_radius", 0.40)

        self.declare_parameter("stall_watchdog_enabled", True)
        self.declare_parameter("stall_watchdog_min_displacement_m", 0.10)
        self.declare_parameter("stall_watchdog_time_window_sec", 6.0)
        self.declare_parameter("stall_watchdog_grace_period_sec", 2.0)
        self.declare_parameter("stall_watchdog_min_goal_distance_m", 0.40)

        self.declare_parameter("phase5_live_recovery_enabled", True)
        self.declare_parameter("phase5_log_recovery_scene", True)
        self.declare_parameter("phase5_front_open_clearance", 0.28)
        self.declare_parameter("phase5_side_open_clearance", 0.24)
        self.declare_parameter("phase5_blocked_clearance", 0.20)
        self.declare_parameter("phase5_side_probe_heading_deg", 45.0)
        self.declare_parameter("phase5_goal_side_heading_deg", 18.0)
        self.declare_parameter("phase5_goal_retry_ignores_failure_cooldown", True)
        self.declare_parameter("phase5_max_goal_retries_per_stuck_event", 1)
        self.declare_parameter("phase5_relax_failed_zone_on_open_scan", True)
        self.declare_parameter("phase5_failed_zone_relax_margin", 0.10)

        self.declare_parameter("goal_probe_enabled", True)
        self.declare_parameter("goal_probe_forward_speed", 0.08)
        self.declare_parameter("goal_probe_angular_speed", 0.24)
        self.declare_parameter("goal_probe_duration_sec", 1.50)
        self.declare_parameter("goal_probe_zero_cmd_after_sec", 0.10)
        self.declare_parameter("goal_probe_wait_after_sec", 0.15)
        self.declare_parameter("goal_probe_front_clearance", 0.26)
        self.declare_parameter("goal_probe_side_clearance", 0.22)
        self.declare_parameter("goal_probe_hard_stop_clearance", 0.20)
        self.declare_parameter("goal_probe_heading_deg", 22.0)
        self.declare_parameter("goal_probe_straight_heading_deg", 15.0)
        self.declare_parameter("goal_probe_max_goal_heading_deg", 110.0)
        self.declare_parameter("goal_probe_max_attempts_per_stuck_event", 3)
        self.declare_parameter("goal_probe_max_attempts_per_mission", 6)

        self.declare_parameter("reverse_escape_enabled", True)
        self.declare_parameter("reverse_speed", -0.08)
        self.declare_parameter("reverse_max_duration", 4.0)
        self.declare_parameter("reverse_target_distance", 0.32)
        self.declare_parameter("reverse_absolute_max_distance", 0.38)
        self.declare_parameter("reverse_publish_rate", 20.0)
        self.declare_parameter("reverse_zero_cmd_before_sec", 0.3)
        self.declare_parameter("reverse_zero_cmd_after_sec", 0.25)
        self.declare_parameter("post_reverse_costmap_wait_sec", 0.30)

        self.declare_parameter("rear_check_enabled", True)
        self.declare_parameter("rear_sector_center_degrees", 180.0)
        self.declare_parameter("rear_sector_degrees", 70.0)
        self.declare_parameter("rear_clearance_full_reverse", 0.55)
        self.declare_parameter("rear_clearance_short_reverse", 0.35)
        self.declare_parameter("rear_clearance_hard_stop", 0.24)
        self.declare_parameter("allow_reverse_without_scan", False)
        self.declare_parameter("scan_stale_sec", 1.0)

        self.declare_parameter("controller_patience_enabled", True)
        self.declare_parameter("controller_patience_forward_speed", 0.05)
        self.declare_parameter("controller_patience_angular_speed", 0.22)
        self.declare_parameter("controller_patience_duration_sec", 1.4)
        self.declare_parameter("controller_patience_zero_cmd_after_sec", 0.20)
        self.declare_parameter("controller_patience_wait_after_sec", 0.30)
        self.declare_parameter("controller_patience_clearance_required", 0.26)
        self.declare_parameter("controller_patience_hard_stop_clearance", 0.20)
        self.declare_parameter("controller_patience_heading_deg", 18.0)
        self.declare_parameter("controller_patience_max_attempts_per_stuck_event", 1)
        self.declare_parameter("controller_patience_max_attempts_per_mission", 3)
        self.declare_parameter(
            "controller_patience_candidate_order",
            ["forward", "forward_left", "forward_right"],
        )

        self.declare_parameter("side_escape_enabled", True)
        self.declare_parameter("side_escape_distance", 0.25)
        self.declare_parameter("side_escape_forward_bias", 0.05)
        self.declare_parameter("side_escape_yaw_offset_deg", 15.0)
        self.declare_parameter("side_escape_clearance_required", 0.35)
        self.declare_parameter("side_escape_max_attempts_per_event", 1)
        self.declare_parameter(
            "side_escape_candidate_order",
            ["left", "right", "back_left", "back_right"],
        )
        self.declare_parameter("direct_side_escape_enabled", True)
        self.declare_parameter("direct_side_escape_speed", 0.06)
        self.declare_parameter("direct_side_escape_angular_speed", 0.28)
        self.declare_parameter("direct_side_escape_duration_sec", 1.2)
        self.declare_parameter("direct_side_escape_zero_cmd_after_sec", 0.20)
        self.declare_parameter("direct_side_escape_wait_after_sec", 0.30)
        self.declare_parameter("direct_side_escape_clearance_required", 0.26)
        self.declare_parameter("direct_side_escape_hard_stop_clearance", 0.20)
        self.declare_parameter("direct_side_escape_max_attempts_per_event", 1)
        self.declare_parameter("failed_escape_goal_radius", 0.35)

        self.declare_parameter("breadcrumb_distance", 0.25)
        self.declare_parameter("max_breadcrumbs", 40)
        self.declare_parameter("min_retreat_distance", 0.45)
        self.declare_parameter("preferred_retreat_distance", 0.70)
        self.declare_parameter("max_retreat_distance", 1.50)
        self.declare_parameter("record_breadcrumbs_during_retreat", False)

        self.declare_parameter("failed_zone_radius", 0.35)
        self.declare_parameter("failed_zone_lifetime_sec", 180.0)
        self.declare_parameter("same_failure_distance", 0.35)

        self.declare_parameter("max_same_goal_replans_before_reverse", 1)
        self.declare_parameter("max_reverse_escapes_per_stuck_event", 1)
        self.declare_parameter("max_reverse_escapes_per_mission", 3)
        self.declare_parameter("max_goal_retries_after_reverse", 1)
        self.declare_parameter("max_breadcrumb_attempts", 2)
        self.declare_parameter("max_total_recovery_cycles", 12)

        self.declare_parameter("current_pose_timeout_sec", 0.10)
        self.declare_parameter("odom_stale_sec", 1.0)
        self.declare_parameter("localization_guard_enabled", True)
        self.declare_parameter("localization_jump_distance", 0.35)
        self.declare_parameter("localization_odom_disagreement", 0.25)
        self.declare_parameter("localization_check_min_dt", 0.20)
        self.declare_parameter("localization_settle_sec", 1.5)
        self.declare_parameter("localization_stable_sample_count", 5)
        self.declare_parameter("marker_publish_period_sec", 1.0)
        self.declare_parameter("publish_debug_markers", False)

    def _read_parameters(self):
        self.map_frame = self._param("map_frame")
        self.base_frame = self._param("base_frame")

        self.mission_goal_topic = self._param("mission_goal_topic")
        self.rviz_goal_topic = self._param("rviz_goal_topic")
        self.mission_status_topic = self._param("mission_status_topic")
        self.mission_marker_topic = self._param("mission_marker_topic")
        self.cmd_vel_topic = self._param("cmd_vel_topic")
        self.scan_topic = self._param("scan_topic")
        self.odom_topic = self._param("odom_topic")
        self.nav_action_name = self._param("nav_action_name")
        self.nav_behavior_tree = self._param("nav_behavior_tree")

        self.goal_timeout_sec = self._float_param("goal_timeout_sec")
        self.breadcrumb_timeout_sec = self._float_param("breadcrumb_timeout_sec")
        self.side_escape_goal_timeout_sec = self._float_param(
            "side_escape_goal_timeout_sec"
        )
        self.return_to_base_timeout_sec = self._float_param(
            "return_to_base_timeout_sec"
        )
        self.supervisor_loop_rate = self._float_param("supervisor_loop_rate")

        self.goal_retry_preemption_enabled = self._bool_param(
            "goal_retry_preemption_enabled"
        )
        self.goal_retry_preemption_min_active_sec = self._float_param(
            "goal_retry_preemption_min_active_sec"
        )
        self.goal_retry_preemption_cooldown_sec = self._float_param(
            "goal_retry_preemption_cooldown_sec"
        )
        self.goal_retry_preemption_clearance = self._float_param(
            "goal_retry_preemption_clearance"
        )
        self.goal_retry_preemption_front_clearance = self._float_param(
            "goal_retry_preemption_front_clearance"
        )
        self.goal_retry_preemption_sector_degrees = self._float_param(
            "goal_retry_preemption_sector_degrees"
        )
        self.goal_retry_preemption_max_heading_error_degrees = self._float_param(
            "goal_retry_preemption_max_heading_error_degrees"
        )
        self.goal_retry_preemption_required_stable_ticks = self._int_param(
            "goal_retry_preemption_required_stable_ticks"
        )
        self.goal_retry_failure_cooldown_sec = self._float_param(
            "goal_retry_failure_cooldown_sec"
        )
        self.goal_retry_failure_radius = self._float_param(
            "goal_retry_failure_radius"
        )

        self.stall_watchdog_enabled = self._bool_param("stall_watchdog_enabled")
        self.stall_watchdog_min_displacement_m = self._float_param(
            "stall_watchdog_min_displacement_m"
        )
        self.stall_watchdog_time_window_sec = self._float_param(
            "stall_watchdog_time_window_sec"
        )
        self.stall_watchdog_grace_period_sec = self._float_param(
            "stall_watchdog_grace_period_sec"
        )
        self.stall_watchdog_min_goal_distance_m = self._float_param(
            "stall_watchdog_min_goal_distance_m"
        )

        self.phase5_live_recovery_enabled = self._bool_param(
            "phase5_live_recovery_enabled"
        )
        self.phase5_log_recovery_scene = self._bool_param("phase5_log_recovery_scene")
        self.phase5_front_open_clearance = self._float_param(
            "phase5_front_open_clearance"
        )
        self.phase5_side_open_clearance = self._float_param(
            "phase5_side_open_clearance"
        )
        self.phase5_blocked_clearance = self._float_param(
            "phase5_blocked_clearance"
        )
        self.phase5_side_probe_heading_deg = self._float_param(
            "phase5_side_probe_heading_deg"
        )
        self.phase5_goal_side_heading_deg = self._float_param(
            "phase5_goal_side_heading_deg"
        )
        self.phase5_goal_retry_ignores_failure_cooldown = self._bool_param(
            "phase5_goal_retry_ignores_failure_cooldown"
        )
        self.phase5_max_goal_retries_per_stuck_event = self._int_param(
            "phase5_max_goal_retries_per_stuck_event"
        )
        self.phase5_relax_failed_zone_on_open_scan = self._bool_param(
            "phase5_relax_failed_zone_on_open_scan"
        )
        self.phase5_failed_zone_relax_margin = self._float_param(
            "phase5_failed_zone_relax_margin"
        )

        self.goal_probe_enabled = self._bool_param("goal_probe_enabled")
        self.goal_probe_forward_speed = self._float_param("goal_probe_forward_speed")
        self.goal_probe_angular_speed = self._float_param("goal_probe_angular_speed")
        self.goal_probe_duration_sec = self._float_param("goal_probe_duration_sec")
        self.goal_probe_zero_cmd_after_sec = self._float_param(
            "goal_probe_zero_cmd_after_sec"
        )
        self.goal_probe_wait_after_sec = self._float_param(
            "goal_probe_wait_after_sec"
        )
        self.goal_probe_front_clearance = self._float_param(
            "goal_probe_front_clearance"
        )
        self.goal_probe_side_clearance = self._float_param(
            "goal_probe_side_clearance"
        )
        self.goal_probe_hard_stop_clearance = self._float_param(
            "goal_probe_hard_stop_clearance"
        )
        self.goal_probe_heading_deg = self._float_param("goal_probe_heading_deg")
        self.goal_probe_straight_heading_deg = self._float_param(
            "goal_probe_straight_heading_deg"
        )
        self.goal_probe_max_goal_heading_deg = self._float_param(
            "goal_probe_max_goal_heading_deg"
        )
        self.goal_probe_max_attempts_per_stuck_event = self._int_param(
            "goal_probe_max_attempts_per_stuck_event"
        )
        self.goal_probe_max_attempts_per_mission = self._int_param(
            "goal_probe_max_attempts_per_mission"
        )

        self.reverse_escape_enabled = self._bool_param("reverse_escape_enabled")
        self.reverse_speed = -abs(self._float_param("reverse_speed"))
        self.reverse_max_duration = self._float_param("reverse_max_duration")
        self.reverse_target_distance = self._float_param("reverse_target_distance")
        self.reverse_absolute_max_distance = self._float_param(
            "reverse_absolute_max_distance"
        )
        self.reverse_publish_rate = self._float_param("reverse_publish_rate")
        self.reverse_zero_cmd_before_sec = self._float_param(
            "reverse_zero_cmd_before_sec"
        )
        self.reverse_zero_cmd_after_sec = self._float_param(
            "reverse_zero_cmd_after_sec"
        )
        self.post_reverse_costmap_wait_sec = self._float_param(
            "post_reverse_costmap_wait_sec"
        )

        self.rear_check_enabled = self._bool_param("rear_check_enabled")
        self.rear_sector_center_degrees = self._float_param(
            "rear_sector_center_degrees"
        )
        self.rear_sector_degrees = self._float_param("rear_sector_degrees")
        self.rear_clearance_full_reverse = self._float_param(
            "rear_clearance_full_reverse"
        )
        self.rear_clearance_short_reverse = self._float_param(
            "rear_clearance_short_reverse"
        )
        self.rear_clearance_hard_stop = self._float_param("rear_clearance_hard_stop")
        self.allow_reverse_without_scan = self._bool_param(
            "allow_reverse_without_scan"
        )
        self.scan_stale_sec = self._float_param("scan_stale_sec")

        self.controller_patience_enabled = self._bool_param(
            "controller_patience_enabled"
        )
        self.controller_patience_forward_speed = self._float_param(
            "controller_patience_forward_speed"
        )
        self.controller_patience_angular_speed = self._float_param(
            "controller_patience_angular_speed"
        )
        self.controller_patience_duration_sec = self._float_param(
            "controller_patience_duration_sec"
        )
        self.controller_patience_zero_cmd_after_sec = self._float_param(
            "controller_patience_zero_cmd_after_sec"
        )
        self.controller_patience_wait_after_sec = self._float_param(
            "controller_patience_wait_after_sec"
        )
        self.controller_patience_clearance_required = self._float_param(
            "controller_patience_clearance_required"
        )
        self.controller_patience_hard_stop_clearance = self._float_param(
            "controller_patience_hard_stop_clearance"
        )
        self.controller_patience_heading_deg = self._float_param(
            "controller_patience_heading_deg"
        )
        self.controller_patience_max_attempts_per_stuck_event = self._int_param(
            "controller_patience_max_attempts_per_stuck_event"
        )
        self.controller_patience_max_attempts_per_mission = self._int_param(
            "controller_patience_max_attempts_per_mission"
        )
        self.controller_patience_candidate_order = self._string_list_param(
            "controller_patience_candidate_order"
        )

        self.side_escape_enabled = self._bool_param("side_escape_enabled")
        self.side_escape_distance = self._float_param("side_escape_distance")
        self.side_escape_forward_bias = self._float_param("side_escape_forward_bias")
        self.side_escape_yaw_offset_deg = self._float_param(
            "side_escape_yaw_offset_deg"
        )
        self.side_escape_clearance_required = self._float_param(
            "side_escape_clearance_required"
        )
        self.side_escape_max_attempts_per_event = self._int_param(
            "side_escape_max_attempts_per_event"
        )
        self.side_escape_candidate_order = self._string_list_param(
            "side_escape_candidate_order"
        )
        self.direct_side_escape_enabled = self._bool_param(
            "direct_side_escape_enabled"
        )
        self.direct_side_escape_speed = self._float_param(
            "direct_side_escape_speed"
        )
        self.direct_side_escape_angular_speed = self._float_param(
            "direct_side_escape_angular_speed"
        )
        self.direct_side_escape_duration_sec = self._float_param(
            "direct_side_escape_duration_sec"
        )
        self.direct_side_escape_zero_cmd_after_sec = self._float_param(
            "direct_side_escape_zero_cmd_after_sec"
        )
        self.direct_side_escape_wait_after_sec = self._float_param(
            "direct_side_escape_wait_after_sec"
        )
        self.direct_side_escape_clearance_required = self._float_param(
            "direct_side_escape_clearance_required"
        )
        self.direct_side_escape_hard_stop_clearance = self._float_param(
            "direct_side_escape_hard_stop_clearance"
        )
        self.direct_side_escape_max_attempts_per_event = self._int_param(
            "direct_side_escape_max_attempts_per_event"
        )
        self.failed_escape_goal_radius = self._float_param(
            "failed_escape_goal_radius"
        )

        self.breadcrumb_distance = self._float_param("breadcrumb_distance")
        self.max_breadcrumbs = self._int_param("max_breadcrumbs")
        self.min_retreat_distance = self._float_param("min_retreat_distance")
        self.preferred_retreat_distance = self._float_param(
            "preferred_retreat_distance"
        )
        self.max_retreat_distance = self._float_param("max_retreat_distance")
        self.record_breadcrumbs_during_retreat = self._bool_param(
            "record_breadcrumbs_during_retreat"
        )

        self.failed_zone_radius = self._float_param("failed_zone_radius")
        self.failed_zone_lifetime_sec = self._float_param("failed_zone_lifetime_sec")
        self.same_failure_distance = self._float_param("same_failure_distance")

        self.max_same_goal_replans_before_reverse = self._int_param(
            "max_same_goal_replans_before_reverse"
        )
        self.max_reverse_escapes_per_stuck_event = self._int_param(
            "max_reverse_escapes_per_stuck_event"
        )
        self.max_reverse_escapes_per_mission = self._int_param(
            "max_reverse_escapes_per_mission"
        )
        self.max_goal_retries_after_reverse = self._int_param(
            "max_goal_retries_after_reverse"
        )
        self.max_breadcrumb_attempts = self._int_param("max_breadcrumb_attempts")
        self.max_total_recovery_cycles = self._int_param(
            "max_total_recovery_cycles"
        )

        self.current_pose_timeout_sec = self._float_param("current_pose_timeout_sec")
        self.odom_stale_sec = self._float_param("odom_stale_sec")
        self.localization_guard_enabled = self._bool_param(
            "localization_guard_enabled"
        )
        self.localization_jump_distance = self._float_param(
            "localization_jump_distance"
        )
        self.localization_odom_disagreement = self._float_param(
            "localization_odom_disagreement"
        )
        self.localization_check_min_dt = self._float_param(
            "localization_check_min_dt"
        )
        self.localization_settle_sec = self._float_param("localization_settle_sec")
        self.localization_stable_sample_count = self._int_param(
            "localization_stable_sample_count"
        )
        self.marker_publish_period_sec = self._float_param(
            "marker_publish_period_sec"
        )
        self.publish_debug_markers = self._bool_param("publish_debug_markers")

    def _param(self, name: str):
        return self.get_parameter(name).value

    def _float_param(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _int_param(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _bool_param(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    def _float_list_param(self, name: str):
        value = self.get_parameter(name).value
        return [float(item) for item in value]

    def _string_list_param(self, name: str):
        value = self.get_parameter(name).value
        return [str(item) for item in value]

    # ------------------------------------------------------------------
    # ROS input
    # ------------------------------------------------------------------

    def _on_new_goal(self, goal_msg: PoseStamped, source: str):
        goal = self._copy_pose(goal_msg)

        if goal.header.frame_id == "":
            goal.header.frame_id = self.map_frame

        if goal.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f"Ignoring {source} in frame '{goal.header.frame_id}'. "
                f"Expected '{self.map_frame}'."
            )
            self._publish_status("GOAL_REJECTED_WRONG_FRAME")
            return

        self.get_logger().warn(f"New {source} received. Preempting current mission.")
        self._start_clean_mission(goal)

    def _on_scan(self, msg: LaserScan):
        self.latest_scan = msg
        self.latest_scan_time_sec = self._now_sec()

    def _on_odom(self, msg: Odometry):
        self.latest_odom = msg
        self.latest_odom_time_sec = self._now_sec()

    # ------------------------------------------------------------------
    # Mission lifecycle
    # ------------------------------------------------------------------

    def _start_clean_mission(self, goal: PoseStamped):
        self.mission_id += 1
        self.goal_sequence_id += 1

        self._cancel_active_nav_goal("PREEMPTED")
        self._stop_reverse_escape()
        self._stop_goal_probe()
        self._stop_controller_patience()
        self._stop_direct_side_escape()
        self._publish_zero_cmd()

        current_pose = self._get_current_pose()
        if current_pose is None:
            self._set_state(MissionState.FAILED, "FAILED_NO_CURRENT_POSE")
            self.get_logger().error("Cannot start mission: current robot pose is unavailable.")
            return

        self.base_pose = self._copy_pose(current_pose)
        self.original_goal = self._copy_pose(goal)

        self.route_memory.reset()
        self.route_memory.add_breadcrumb_if_needed(current_pose, force=True)

        self.same_goal_replans = 0
        self.reverse_escapes_this_stuck_event = 0
        self.reverse_escapes_this_mission = 0
        self.goal_retries_after_reverse = 0
        self.phase5_goal_retries_this_stuck_event = 0
        self.goal_probe_attempts_this_stuck_event = 0
        self.goal_probe_attempts_this_mission = 0
        self.controller_patience_attempts_this_stuck_event = 0
        self.controller_patience_attempts_this_mission = 0
        self.controller_patience_retry_fallback = "reverse"
        self.side_escape_attempts_this_stuck_event = 0
        self.direct_side_escape_attempts_this_stuck_event = 0
        self.breadcrumb_attempts = 0
        self.total_recovery_cycles = 0
        self.failed_retreat_poses = []
        self.failed_escape_poses = []
        self.last_failed_pose = None
        self.last_goal_preemption_sec = -math.inf
        self.goal_corridor_clear_ticks = 0
        self.last_goal_retry_failure_pose = None
        self.last_goal_retry_failure_sec = -math.inf
        self.last_failure_reason_was_stall = False
        self._reset_stall_watchdog()

        self.active_goal_handle = None
        self.active_goal_mission_id = None
        self.active_goal_sequence_id = None
        self.active_goal_purpose = None
        self.active_goal_pose = None
        self.active_goal_start_time_sec = None
        self.cancel_in_progress = False
        self.active_goal_cancel_reason = None
        self.localization_unstable_until_sec = 0.0
        self.localization_stable_samples = 0
        self.localization_resume_purpose = None
        self.last_localization_map_pose = self._copy_pose(current_pose)
        self.last_localization_odom_xy = self._current_odom_xy_if_fresh(self._now_sec())
        self.last_localization_sample_sec = self._now_sec()

        self._set_state(MissionState.START_MISSION, "MISSION_STARTED")
        self._send_original_goal(NavPurpose.GOAL, MissionState.NAVIGATING_TO_GOAL)

    def _send_original_goal(self, purpose: str, state: MissionState):
        if self.original_goal is None:
            self._set_state(MissionState.FAILED, "FAILED_NO_ORIGINAL_GOAL")
            return

        self._set_state(state, f"SENDING_{purpose.upper()}")
        self._send_nav_goal(self.original_goal, purpose)

    def _on_timer(self):
        now_sec = self._now_sec()

        self.route_memory.cleanup_expired_failed_zones(now_sec)
        self._update_localization_guard(now_sec)

        if self.state == MissionState.WAIT_FOR_LOCALIZATION_STABILITY:
            self._run_localization_wait(now_sec)
            return

        self._record_breadcrumb_if_valid()
        self._check_active_goal_timeout(now_sec)
        self._check_stall_watchdog(now_sec)
        self._run_reverse_state_machine(now_sec)
        self._run_goal_probe_state_machine(now_sec)
        self._run_controller_patience_state_machine(now_sec)
        self._run_direct_side_escape_state_machine(now_sec)

        if self.publish_debug_markers and self._should_publish_markers(now_sec):
            self.marker_pub.publish(
                self.route_memory.make_marker_array(self.get_clock().now().to_msg())
            )

    def _on_recovery_cmd_timer(self):
        if self.reverse_phase != ReversePhase.IDLE:
            if self.reverse_mission_id != self.mission_id:
                return
            if self.reverse_phase == ReversePhase.ACTIVE:
                self._publish_reverse_cmd()
            else:
                self._publish_zero_cmd()
            return

        if self.goal_probe_phase != GoalProbePhase.IDLE:
            if self.goal_probe_mission_id != self.mission_id:
                return
            if self.goal_probe_phase == GoalProbePhase.ACTIVE:
                self.cmd_vel_pub.publish(self.goal_probe_command)
            else:
                self._publish_zero_cmd()
            return

        if self.controller_patience_phase != ControllerPatiencePhase.IDLE:
            if self.controller_patience_mission_id != self.mission_id:
                return
            if self.controller_patience_phase == ControllerPatiencePhase.ACTIVE:
                self.cmd_vel_pub.publish(self.controller_patience_command)
            else:
                self._publish_zero_cmd()
            return

        if self.direct_side_phase != DirectSideEscapePhase.IDLE:
            if self.direct_side_mission_id != self.mission_id:
                return
            if self.direct_side_phase == DirectSideEscapePhase.ACTIVE:
                self.cmd_vel_pub.publish(self.direct_side_command)
            else:
                self._publish_zero_cmd()

    def _record_breadcrumb_if_valid(self):
        if self.active_goal_purpose not in [
            NavPurpose.GOAL,
            NavPurpose.REPLAN,
            NavPurpose.RETRY_AFTER_CONTROLLER_PATIENCE,
            NavPurpose.RETRY_AFTER_REVERSE,
            NavPurpose.RETRY_AFTER_GOAL_PROBE,
            NavPurpose.RETRY_FROM_SIDE_ESCAPE,
            NavPurpose.RETRY_FROM_BREADCRUMB,
        ]:
            return

        pose = self._get_current_pose()
        if pose is not None:
            self.route_memory.add_breadcrumb_if_needed(pose)

    # ------------------------------------------------------------------
    # Localization stability guard
    # ------------------------------------------------------------------

    def _update_localization_guard(self, now_sec: float):
        if not self.localization_guard_enabled:
            return

        if self.state in [
            MissionState.IDLE,
            MissionState.SUCCEEDED,
            MissionState.RETURNED_TO_BASE,
            MissionState.FAILED,
        ]:
            self.last_localization_map_pose = None
            self.last_localization_odom_xy = None
            self.last_localization_sample_sec = None
            return

        current_pose = self._get_current_pose()
        current_odom = self._current_odom_xy_if_fresh(now_sec)
        if current_pose is None or current_odom is None:
            return

        if (
            self.last_localization_map_pose is None
            or self.last_localization_odom_xy is None
            or self.last_localization_sample_sec is None
        ):
            self.last_localization_map_pose = self._copy_pose(current_pose)
            self.last_localization_odom_xy = current_odom
            self.last_localization_sample_sec = now_sec
            return

        dt = now_sec - self.last_localization_sample_sec
        if dt < self.localization_check_min_dt:
            return

        map_delta = self.route_memory.distance_xy(
            self.last_localization_map_pose,
            current_pose,
        )
        odom_delta = self._distance_xy_tuple(
            self.last_localization_odom_xy,
            current_odom,
        )
        disagreement = abs(map_delta - odom_delta)

        self.last_localization_map_pose = self._copy_pose(current_pose)
        self.last_localization_odom_xy = current_odom
        self.last_localization_sample_sec = now_sec

        jump_detected = (
            map_delta >= self.localization_jump_distance
            and disagreement >= self.localization_odom_disagreement
        )
        if jump_detected:
            self._enter_localization_wait(now_sec, map_delta, odom_delta)
            return

        if self.state == MissionState.WAIT_FOR_LOCALIZATION_STABILITY:
            self.localization_stable_samples += 1

    def _reset_localization_guard_sample(self, now_sec: float):
        current_pose = self._get_current_pose()
        current_odom = self._current_odom_xy_if_fresh(now_sec)
        if current_pose is None or current_odom is None:
            return

        self.last_localization_map_pose = self._copy_pose(current_pose)
        self.last_localization_odom_xy = current_odom
        self.last_localization_sample_sec = now_sec

    def _enter_localization_wait(
        self,
        now_sec: float,
        map_delta: float,
        odom_delta: float,
    ):
        if self.state == MissionState.WAIT_FOR_LOCALIZATION_STABILITY:
            self.localization_unstable_until_sec = max(
                self.localization_unstable_until_sec,
                now_sec + self.localization_settle_sec,
            )
            self.localization_stable_samples = 0
            return

        self.localization_resume_purpose = self.active_goal_purpose
        self.localization_unstable_until_sec = now_sec + self.localization_settle_sec
        self.localization_stable_samples = 0

        self._cancel_active_nav_goal("LOCALIZATION_UNSTABLE")
        self._stop_reverse_escape()
        self._stop_goal_probe()
        self._stop_controller_patience()
        self._stop_direct_side_escape()
        self._publish_zero_cmd()
        self._set_state(
            MissionState.WAIT_FOR_LOCALIZATION_STABILITY,
            "LOCALIZATION_UNSTABLE",
        )
        self.get_logger().warn(
            "Localization jump detected. "
            f"map_delta={map_delta:.2f}m, odom_delta={odom_delta:.2f}m. "
            "Pausing Nav2 recovery until pose settles."
        )

    def _run_localization_wait(self, now_sec: float):
        self._publish_zero_cmd()

        if now_sec < self.localization_unstable_until_sec:
            return

        if self.localization_stable_samples < self.localization_stable_sample_count:
            return

        resume_purpose = self.localization_resume_purpose
        self.localization_resume_purpose = None
        self.localization_unstable_until_sec = 0.0
        self.localization_stable_samples = 0
        self._reset_localization_guard_sample(now_sec)

        self.get_logger().warn("Localization appears stable again. Resuming mission.")

        if resume_purpose == NavPurpose.BASE:
            self._send_base_goal()
            return

        if resume_purpose == NavPurpose.BREADCRUMB:
            self._select_breadcrumb_or_return_to_base()
            return

        if resume_purpose == NavPurpose.SIDE_ESCAPE:
            self._begin_direct_side_escape_or_breadcrumb()
            return

        if self.original_goal is not None:
            self._send_original_goal(NavPurpose.REPLAN, MissionState.REPLAN_SAME_GOAL)
            return

        self._set_state(MissionState.FAILED, "FAILED_NO_RESUME_GOAL")

    # ------------------------------------------------------------------
    # Nav2 action handling
    # ------------------------------------------------------------------

    def _send_nav_goal(self, target_pose: PoseStamped, purpose: str):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"Nav2 action server unavailable: {self.nav_action_name}")
            self._handle_nav_failure(purpose, "ACTION_SERVER_UNAVAILABLE", target_pose)
            return

        self.goal_sequence_id += 1
        goal_sequence_id = self.goal_sequence_id
        mission_id = self.mission_id

        goal = NavigateToPose.Goal()
        goal.pose = self._copy_pose(target_pose)
        if self.nav_behavior_tree:
            goal.behavior_tree = self.nav_behavior_tree

        self.active_goal_handle = None
        self.active_goal_mission_id = mission_id
        self.active_goal_sequence_id = goal_sequence_id
        self.active_goal_purpose = purpose
        self.active_goal_pose = self._copy_pose(target_pose)
        self.active_goal_start_time_sec = self._now_sec()
        self.cancel_in_progress = False
        self.active_goal_cancel_reason = None

        self.get_logger().info(
            f"Sending Nav2 goal purpose={purpose}, mission_id={mission_id}, "
            f"goal_id={goal_sequence_id}"
        )

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done_future: self._on_nav_goal_response(
                done_future,
                mission_id,
                goal_sequence_id,
                purpose,
            )
        )

    def _on_nav_goal_response(
        self,
        future,
        mission_id: int,
        goal_sequence_id: int,
        purpose: str,
    ):
        try:
            goal_handle = future.result()
        except Exception as exc:
            if self._is_current_goal(mission_id, goal_sequence_id):
                self.get_logger().error(f"Nav2 goal send failed: {exc}")
                self._handle_nav_failure(purpose, "SEND_FAILED", self.active_goal_pose)
            return

        if not self._is_current_goal(mission_id, goal_sequence_id):
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal was rejected.")
            self._handle_nav_failure(purpose, "REJECTED", self.active_goal_pose)
            return

        self.active_goal_handle = goal_handle
        self.cancel_in_progress = False
        self.get_logger().info(f"Nav2 goal accepted. purpose={purpose}")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done_future: self._on_nav_result(
                done_future,
                mission_id,
                goal_sequence_id,
                purpose,
            )
        )

    def _on_nav_result(
        self,
        future,
        mission_id: int,
        goal_sequence_id: int,
        purpose: str,
    ):
        if not self._is_current_goal(mission_id, goal_sequence_id):
            return

        try:
            result = future.result()
        except Exception as exc:
            finished_goal_pose = self.active_goal_pose
            self.get_logger().error(f"Failed to read Nav2 result: {exc}")
            self._clear_active_goal()
            self._handle_nav_failure(purpose, "RESULT_ERROR", finished_goal_pose)
            return

        status = result.status
        cancel_reason = self.active_goal_cancel_reason
        finished_goal_pose = self.active_goal_pose
        self._clear_active_goal()

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._handle_nav_success(purpose)
        elif status == GoalStatus.STATUS_CANCELED:
            self._handle_nav_failure(
                purpose,
                cancel_reason or "CANCELED",
                finished_goal_pose,
            )
        elif status == GoalStatus.STATUS_ABORTED:
            self._handle_nav_failure(purpose, "ABORTED", finished_goal_pose)
        else:
            self._handle_nav_failure(purpose, f"STATUS_{status}", finished_goal_pose)

    def _handle_nav_success(self, purpose: str):
        self.get_logger().info(f"Nav2 success. purpose={purpose}")

        if purpose in [
            NavPurpose.GOAL,
            NavPurpose.REPLAN,
            NavPurpose.RETRY_AFTER_CONTROLLER_PATIENCE,
            NavPurpose.RETRY_AFTER_REVERSE,
            NavPurpose.RETRY_AFTER_GOAL_PROBE,
            NavPurpose.RETRY_FROM_SIDE_ESCAPE,
            NavPurpose.RETRY_FROM_BREADCRUMB,
        ]:
            self._publish_zero_cmd()
            self._set_state(MissionState.SUCCEEDED, "GOAL_REACHED")
            return

        if purpose == NavPurpose.SIDE_ESCAPE:
            self.reverse_escapes_this_stuck_event = 0
            self.phase5_goal_retries_this_stuck_event = 0
            self.goal_probe_attempts_this_stuck_event = 0
            self.controller_patience_attempts_this_stuck_event = 0
            self.goal_retries_after_reverse += 1
            self._send_original_goal(
                NavPurpose.RETRY_FROM_SIDE_ESCAPE,
                MissionState.RETRY_GOAL_FROM_SIDE_ESCAPE,
            )
            return

        if purpose == NavPurpose.BREADCRUMB:
            self.reverse_escapes_this_stuck_event = 0
            self.phase5_goal_retries_this_stuck_event = 0
            self.goal_probe_attempts_this_stuck_event = 0
            self.controller_patience_attempts_this_stuck_event = 0
            self.side_escape_attempts_this_stuck_event = 0
            self._send_original_goal(
                NavPurpose.RETRY_FROM_BREADCRUMB,
                MissionState.RETRY_GOAL_FROM_BREADCRUMB,
            )
            return

        if purpose == NavPurpose.BASE:
            self._publish_zero_cmd()
            self._set_state(MissionState.RETURNED_TO_BASE, "RETURNED_TO_BASE")
            return

    def _handle_nav_failure(
        self,
        purpose: str,
        status_text: str,
        failed_goal_pose: Optional[PoseStamped] = None,
    ):
        self.get_logger().warn(f"Nav2 failure. purpose={purpose}, status={status_text}")

        if status_text == "LOCALIZATION_UNSTABLE":
            self._set_state(
                MissionState.WAIT_FOR_LOCALIZATION_STABILITY,
                "LOCALIZATION_UNSTABLE",
            )
            return

        if purpose != NavPurpose.BASE:
            self._mark_current_failed_zone()
            self._record_goal_retry_failure(purpose)

        if purpose == NavPurpose.GOAL:
            self._recover_from_goal_failure()
            return

        if purpose == NavPurpose.REPLAN:
            self._start_controller_patience_or_reverse()
            return

        if purpose == NavPurpose.RETRY_AFTER_CONTROLLER_PATIENCE:
            self._handle_controller_patience_retry_failure()
            return

        if purpose == NavPurpose.RETRY_AFTER_REVERSE:
            self._start_controller_patience_or_side_escape()
            return

        if purpose == NavPurpose.RETRY_AFTER_GOAL_PROBE:
            self._start_controller_patience_or_side_escape()
            return

        if purpose == NavPurpose.SIDE_ESCAPE:
            if failed_goal_pose is not None:
                self.failed_escape_poses.append(self._copy_pose(failed_goal_pose))
            self._begin_direct_side_escape_or_breadcrumb()
            return

        if purpose == NavPurpose.RETRY_FROM_SIDE_ESCAPE:
            self._start_controller_patience_or_breadcrumb()
            return

        if purpose == NavPurpose.BREADCRUMB:
            if failed_goal_pose is not None:
                self.failed_retreat_poses.append(self._copy_pose(failed_goal_pose))
            self._select_breadcrumb_or_return_to_base()
            return

        if purpose == NavPurpose.RETRY_FROM_BREADCRUMB:
            self._start_controller_patience_or_breadcrumb()
            return

        if purpose == NavPurpose.BASE:
            self._publish_zero_cmd()
            self._set_state(MissionState.FAILED, "FAILED_RETURN_TO_BASE_FAILED")
            self.get_logger().error("Return to base failed. Manual assistance is required.")
            return

    def _recover_from_goal_failure(self):
        if self.last_failure_reason_was_stall:
            self.last_failure_reason_was_stall = False
            self.get_logger().warn(
                "Goal failure caused by stall watchdog. Skipping same-goal "
                "replan and jumping to Phase 5 live recovery."
            )
            if self._try_phase5_live_recovery("after stall watchdog"):
                return
            self._start_reverse_or_breadcrumb()
            return

        if self.same_goal_replans < self.max_same_goal_replans_before_reverse:
            if not self._use_recovery_cycle("same goal replan"):
                self._prepare_return_to_base()
                return

            self.same_goal_replans += 1
            self._send_original_goal(NavPurpose.REPLAN, MissionState.REPLAN_SAME_GOAL)
            return

        if self._try_phase5_live_recovery("goal failure after replan"):
            return

        self._start_reverse_or_breadcrumb()

    def _start_reverse_or_breadcrumb(self):
        if self._try_phase5_live_recovery("before reverse"):
            return

        if self.goal_retries_after_reverse >= self.max_goal_retries_after_reverse:
            self._select_breadcrumb_or_return_to_base()
            return

        if (
            self.reverse_escape_enabled
            and self.reverse_escapes_this_stuck_event
            < self.max_reverse_escapes_per_stuck_event
            and self.reverse_escapes_this_mission < self.max_reverse_escapes_per_mission
        ):
            if not self._use_recovery_cycle("reverse escape"):
                self._prepare_return_to_base()
                return

            if self._begin_reverse_escape(ReverseContext.GOAL):
                return

        self._select_breadcrumb_or_return_to_base()

    def _start_controller_patience_or_reverse(self):
        if self._try_phase5_live_recovery("before controller patience or reverse"):
            return

        if self._begin_controller_patience("reverse"):
            return

        self._start_reverse_or_breadcrumb()

    def _start_controller_patience_or_breadcrumb(self):
        if self._try_phase5_live_recovery(
            "before controller patience or breadcrumb",
            allow_reverse=False,
        ):
            return

        if self._begin_controller_patience("breadcrumb"):
            return

        self._select_breadcrumb_or_return_to_base()

    def _start_controller_patience_or_side_escape(self):
        if self._try_phase5_live_recovery(
            "before controller patience or side escape",
            allow_reverse=False,
        ):
            return

        if self._begin_controller_patience("side_escape"):
            return

        self._select_side_escape_or_breadcrumb()

    def _handle_controller_patience_retry_failure(self):
        if self.controller_patience_retry_fallback == "breadcrumb":
            self._select_breadcrumb_or_return_to_base()
            return

        if self.controller_patience_retry_fallback == "side_escape":
            self._select_side_escape_or_breadcrumb()
            return

        self._start_reverse_or_breadcrumb()

    def _select_breadcrumb_or_return_to_base(self):
        self._set_state(MissionState.SELECT_BREADCRUMB, "SELECT_BREADCRUMB")

        if self._try_phase5_live_recovery(
            "before breadcrumb",
            allow_controller_patience=False,
            allow_reverse=False,
        ):
            return

        if self.breadcrumb_attempts >= self.max_breadcrumb_attempts:
            self._prepare_return_to_base()
            return

        current_pose = self._get_current_pose()
        retreat_pose = self.route_memory.select_retreat_pose_avoiding(
            current_pose,
            self.failed_retreat_poses,
        )

        if retreat_pose is None:
            self.get_logger().warn("No safe breadcrumb available. Returning to base.")
            self._prepare_return_to_base()
            return

        if not self._use_recovery_cycle("breadcrumb retreat"):
            self._prepare_return_to_base()
            return

        self.breadcrumb_attempts += 1
        self._set_state(MissionState.NAVIGATING_TO_BREADCRUMB, "NAVIGATING_TO_BREADCRUMB")
        self._send_nav_goal(retreat_pose, NavPurpose.BREADCRUMB)

    def _select_side_escape_or_breadcrumb(self):
        if self._try_phase5_live_recovery(
            "before side escape",
            allow_controller_patience=False,
            allow_reverse=False,
        ):
            return

        now_sec = self._now_sec()
        scene = self._phase5_recovery_scene(now_sec)
        preferred_side = scene["best_side"] if scene is not None else None

        if (
            self.side_escape_enabled
            and self.side_escape_attempts_this_stuck_event
            < self.side_escape_max_attempts_per_event
        ):
            self._set_state(MissionState.SELECT_SIDE_ESCAPE, "SELECT_SIDE_ESCAPE")
            escape_pose = self._select_side_escape_pose(preferred_side)

            if escape_pose is not None:
                if not self._use_recovery_cycle("side escape"):
                    self._prepare_return_to_base()
                    return

                self.side_escape_attempts_this_stuck_event += 1
                self._set_state(
                    MissionState.NAVIGATING_TO_SIDE_ESCAPE,
                    "NAVIGATING_TO_SIDE_ESCAPE",
                )
                self._send_nav_goal(escape_pose, NavPurpose.SIDE_ESCAPE)
                return

        self._select_breadcrumb_or_return_to_base()

    def _begin_direct_side_escape_or_breadcrumb(self):
        if self._begin_direct_side_escape_only():
            return

        self._select_breadcrumb_or_return_to_base()

    def _begin_direct_side_escape_only(self, preferred_side: Optional[str] = None) -> bool:
        if (
            not self.direct_side_escape_enabled
            or self.direct_side_escape_attempts_this_stuck_event
            >= self.direct_side_escape_max_attempts_per_event
        ):
            return False

        selection = self._select_direct_side_escape_command(preferred_side)
        if selection is None:
            self.get_logger().warn("Direct side escape skipped: no safe scan command.")
            return False

        if not self._use_recovery_cycle("direct side escape"):
            return False

        command, heading_rad, summary = selection
        self._cancel_active_nav_goal("DIRECT_SIDE_ESCAPE")
        self._publish_zero_cmd()

        self.direct_side_escape_attempts_this_stuck_event += 1
        self.direct_side_mission_id = self.mission_id
        self.direct_side_phase = DirectSideEscapePhase.ACTIVE
        self.direct_side_phase_start_sec = self._now_sec()
        self.direct_side_active_start_sec = self.direct_side_phase_start_sec
        self.direct_side_command = command
        self.direct_side_heading_rad = heading_rad
        self.direct_side_stop_reason = ""

        self._set_state(
            MissionState.DIRECT_SIDE_ESCAPE_ACTIVE,
            "DIRECT_SIDE_ESCAPE_ACTIVE",
        )
        self.get_logger().warn(f"Starting direct side escape on /cmd_vel: {summary}")
        return True

    def _prepare_return_to_base(self):
        self._set_state(MissionState.PREPARE_RETURN_TO_BASE, "PREPARE_RETURN_TO_BASE")

        now_sec = self._now_sec()
        if self._send_goal_retry_if_open(
            now_sec,
            NavPurpose.RETRY_FROM_BREADCRUMB,
            MissionState.RETRY_GOAL_FROM_BREADCRUMB,
            "before return to base",
        ):
            return

        if self._try_phase5_live_recovery(
            "before return to base",
            allow_controller_patience=False,
            allow_reverse=False,
        ):
            return

        if self.base_pose is None:
            self._publish_zero_cmd()
            self._set_state(MissionState.FAILED, "FAILED_NO_BASE_POSE")
            return

        if (
            self.reverse_escape_enabled
            and self.reverse_escapes_this_mission < self.max_reverse_escapes_per_mission
        ):
            if self._begin_reverse_escape(ReverseContext.BASE):
                return

        self._send_base_goal()

    def _send_base_goal(self):
        if self.base_pose is None:
            self._set_state(MissionState.FAILED, "FAILED_NO_BASE_POSE")
            return

        self._set_state(MissionState.NAVIGATING_TO_BASE, "NAVIGATING_TO_BASE")
        self._send_nav_goal(self.base_pose, NavPurpose.BASE)

    def _use_recovery_cycle(self, reason: str) -> bool:
        if self.total_recovery_cycles >= self.max_total_recovery_cycles:
            self.get_logger().warn(
                f"Recovery cycle limit reached before {reason}. "
                f"limit={self.max_total_recovery_cycles}"
            )
            return False

        self.total_recovery_cycles += 1
        return True

    def _recovery_cycle_available(self) -> bool:
        return self.total_recovery_cycles < self.max_total_recovery_cycles

    # ------------------------------------------------------------------
    # Phase 5 live recovery decision layer
    # ------------------------------------------------------------------

    def _try_phase5_live_recovery(
        self,
        context: str,
        allow_goal_probe: bool = True,
        allow_controller_patience: bool = True,
        allow_direct_side: bool = True,
        allow_reverse: bool = False,
    ) -> bool:
        if not self.phase5_live_recovery_enabled:
            return False

        now_sec = self._now_sec()
        scene = self._phase5_recovery_scene(now_sec)
        if scene is None:
            return False

        if self.phase5_log_recovery_scene:
            self.get_logger().warn(
                "Phase5 recovery scene "
                f"context={context}, "
                f"class={scene['classification']}, "
                f"goal_heading={scene['goal_heading_deg']:.1f}deg, "
                f"goal_side={scene['goal_side']}, "
                f"front={self._format_clearance(scene['front_clearance'])}, "
                f"left={self._format_clearance(scene['left_clearance'])}, "
                f"right={self._format_clearance(scene['right_clearance'])}, "
                f"rear={self._format_clearance(scene['rear_clearance'])}"
            )

        if (
            self.phase5_goal_retries_this_stuck_event
            < self.phase5_max_goal_retries_per_stuck_event
        ):
            if self._send_goal_retry_if_open(
                now_sec,
                NavPurpose.RETRY_AFTER_GOAL_PROBE,
                MissionState.RETRY_GOAL_AFTER_GOAL_PROBE,
                f"phase5 {context}",
                ignore_failure_cooldown=self.phase5_goal_retry_ignores_failure_cooldown,
                require_stable_ticks=False,
            ):
                self.phase5_goal_retries_this_stuck_event += 1
                return True

        if not self._recovery_cycle_available():
            return False

        goal_directed_scene = (
            scene["front_open"]
            or (
                scene["goal_side"] in ["left", "right"]
                and scene["goal_side_open"]
            )
        )
        if allow_goal_probe and goal_directed_scene:
            if self._begin_goal_probe():
                return True

        if allow_direct_side and scene["best_side"] is not None:
            if self._begin_direct_side_escape_only(scene["best_side"]):
                return True

        if allow_controller_patience and scene["front_open"]:
            if self._begin_controller_patience("side_escape"):
                return True

        if allow_reverse and scene["trapped"]:
            if (
                self.reverse_escape_enabled
                and self.reverse_escapes_this_stuck_event
                < self.max_reverse_escapes_per_stuck_event
                and self.reverse_escapes_this_mission < self.max_reverse_escapes_per_mission
            ):
                if self._use_recovery_cycle("phase5 trapped reverse"):
                    if self._begin_reverse_escape(ReverseContext.GOAL):
                        return True

        return False

    def _phase5_recovery_scene(self, now_sec: float):
        if self.original_goal is None:
            return None

        if self.latest_scan is None or self.latest_scan_time_sec is None:
            return None

        if now_sec - self.latest_scan_time_sec > self.scan_stale_sec:
            return None

        current_pose = self._get_current_pose()
        if current_pose is None:
            return None

        dx = self.original_goal.pose.position.x - current_pose.pose.position.x
        dy = self.original_goal.pose.position.y - current_pose.pose.position.y
        goal_distance = math.hypot(dx, dy)
        robot_yaw = self._yaw_from_pose(current_pose)
        goal_heading = math.atan2(dy, dx)
        relative_goal_heading = self._angle_diff(goal_heading, robot_yaw)
        side_heading = math.radians(max(10.0, self.phase5_side_probe_heading_deg))

        front_clearance = self._scan_clearance_at_heading(0.0, 40.0)
        left_clearance = self._scan_clearance_at_heading(side_heading, 45.0)
        right_clearance = self._scan_clearance_at_heading(-side_heading, 45.0)
        rear_clearance = self._get_rear_clearance()

        side_goal_threshold = math.radians(max(0.0, self.phase5_goal_side_heading_deg))
        if relative_goal_heading > side_goal_threshold:
            goal_side = "left"
            goal_side_clearance = left_clearance
        elif relative_goal_heading < -side_goal_threshold:
            goal_side = "right"
            goal_side_clearance = right_clearance
        else:
            goal_side = "straight"
            goal_side_clearance = front_clearance

        front_open = self._phase5_clearance_open(
            front_clearance,
            self.phase5_front_open_clearance,
        )
        left_open = self._phase5_clearance_open(
            left_clearance,
            self.phase5_side_open_clearance,
        )
        right_open = self._phase5_clearance_open(
            right_clearance,
            self.phase5_side_open_clearance,
        )
        goal_side_open = self._phase5_clearance_open(
            goal_side_clearance,
            self.phase5_side_open_clearance
            if goal_side != "straight"
            else self.phase5_front_open_clearance,
        )

        side_options = []
        if left_open:
            side_options.append(("left", left_clearance if left_clearance is not None else 0.0))
        if right_open:
            side_options.append(("right", right_clearance if right_clearance is not None else 0.0))

        best_side = None
        if side_options:
            if goal_side in ["left", "right"] and goal_side_open:
                best_side = goal_side
            else:
                best_side = max(side_options, key=lambda item: item[1])[0]

        trapped = (
            self._phase5_clearance_blocked(front_clearance)
            and self._phase5_clearance_blocked(left_clearance)
            and self._phase5_clearance_blocked(right_clearance)
        )

        if front_open and goal_side == "straight":
            classification = "GOAL_CORRIDOR_LIKELY_OPEN"
        elif front_open and goal_side in ["left", "right"]:
            classification = "FORWARD_THEN_GOAL_SIDE"
        elif best_side is not None:
            classification = f"SIDE_OPEN_{best_side.upper()}"
        elif trapped:
            classification = "LOCAL_TRAP"
        else:
            classification = "LIMITED_CLEARANCE"

        return {
            "classification": classification,
            "goal_distance": goal_distance,
            "goal_heading_rad": relative_goal_heading,
            "goal_heading_deg": math.degrees(relative_goal_heading),
            "goal_side": goal_side,
            "goal_side_open": goal_side_open,
            "front_clearance": front_clearance,
            "left_clearance": left_clearance,
            "right_clearance": right_clearance,
            "rear_clearance": rear_clearance,
            "front_open": front_open,
            "left_open": left_open,
            "right_open": right_open,
            "best_side": best_side,
            "trapped": trapped,
        }

    def _phase5_clearance_open(self, clearance: Optional[float], required: float) -> bool:
        return clearance is not None and clearance >= required

    def _phase5_clearance_blocked(self, clearance: Optional[float]) -> bool:
        return clearance is not None and clearance <= self.phase5_blocked_clearance

    def _format_clearance(self, clearance: Optional[float]) -> str:
        if clearance is None:
            return "none"
        return f"{clearance:.2f}m"

    def _phase5_order_candidates(self, candidates: Sequence[str], preferred_side: Optional[str]):
        if preferred_side not in ["left", "right"]:
            return list(candidates)

        preferred = []
        opposite = []
        rest = []
        for candidate in candidates:
            direction = candidate.lower()
            if preferred_side in direction:
                preferred.append(candidate)
            elif ("left" in direction and preferred_side == "right") or (
                "right" in direction and preferred_side == "left"
            ):
                opposite.append(candidate)
            else:
                rest.append(candidate)

        return preferred + rest + opposite

    # ------------------------------------------------------------------
    # Goal approach probe
    # ------------------------------------------------------------------

    def _begin_goal_probe(self) -> bool:
        if not self.goal_probe_enabled:
            return False

        if (
            self.goal_probe_attempts_this_stuck_event
            >= self.goal_probe_max_attempts_per_stuck_event
        ):
            return False

        if self.goal_probe_attempts_this_mission >= self.goal_probe_max_attempts_per_mission:
            return False

        selection = self._select_goal_probe_command()
        if selection is None:
            return False

        if not self._use_recovery_cycle("goal approach probe"):
            return False

        command, heading_rad, summary = selection
        self._cancel_active_nav_goal("GOAL_PROBE")
        self._publish_zero_cmd()

        self.goal_probe_attempts_this_stuck_event += 1
        self.goal_probe_attempts_this_mission += 1
        self.goal_probe_mission_id = self.mission_id
        self.goal_probe_phase = GoalProbePhase.ACTIVE
        self.goal_probe_phase_start_sec = self._now_sec()
        self.goal_probe_active_start_sec = self.goal_probe_phase_start_sec
        self.goal_probe_command = command
        self.goal_probe_heading_rad = heading_rad
        self.goal_probe_stop_reason = ""

        self._set_state(MissionState.GOAL_PROBE_ACTIVE, "GOAL_PROBE_ACTIVE")
        self.get_logger().warn(f"Starting goal approach probe on /cmd_vel: {summary}")
        return True

    def _run_goal_probe_state_machine(self, now_sec: float):
        if self.goal_probe_phase == GoalProbePhase.IDLE:
            return

        if self.goal_probe_mission_id != self.mission_id:
            self._stop_goal_probe()
            return

        if self.goal_probe_phase == GoalProbePhase.ACTIVE:
            should_stop, reason = self._goal_probe_should_stop(now_sec)
            if should_stop:
                self.goal_probe_stop_reason = reason
                self._publish_zero_cmd()
                self.goal_probe_phase = GoalProbePhase.ZERO_AFTER
                self.goal_probe_phase_start_sec = now_sec
                self._set_state(
                    MissionState.WAIT_AFTER_GOAL_PROBE,
                    f"GOAL_PROBE_STOPPED_{reason}",
                )
                return

            if self._goal_probe_active_elapsed(now_sec) >= (
                self.goal_retry_preemption_min_active_sec
            ):
                if self._maybe_preempt_recovery_to_goal(now_sec, "goal probe"):
                    return

            return

        if self.goal_probe_phase == GoalProbePhase.ZERO_AFTER:
            self._publish_zero_cmd()
            if self._goal_probe_phase_elapsed(now_sec) >= (
                self.goal_probe_zero_cmd_after_sec
            ):
                self.goal_probe_phase = GoalProbePhase.WAIT_AFTER
                self.goal_probe_phase_start_sec = now_sec
            return

        if self.goal_probe_phase == GoalProbePhase.WAIT_AFTER:
            self._publish_zero_cmd()
            if self._goal_probe_phase_elapsed(now_sec) >= self.goal_probe_wait_after_sec:
                self._finish_goal_probe()

    def _goal_probe_should_stop(self, now_sec: float) -> Tuple[bool, str]:
        if self.goal_probe_active_start_sec is None:
            return False, ""

        elapsed = now_sec - self.goal_probe_active_start_sec
        if elapsed >= self.goal_probe_duration_sec:
            return True, "DURATION_REACHED"

        clearance = self._scan_clearance_at_heading(self.goal_probe_heading_rad, 35.0)
        if clearance is None:
            return True, "SCAN_LOST"

        if clearance < self.goal_probe_hard_stop_clearance:
            return True, "PROBE_UNSAFE"

        front_clearance = self._scan_clearance_at_heading(0.0, 35.0)
        if front_clearance is None:
            return True, "FRONT_SCAN_LOST"

        if front_clearance < self.goal_probe_hard_stop_clearance:
            return True, "FRONT_UNSAFE"

        if not rclpy.ok():
            return True, "ROS_SHUTDOWN"

        return False, ""

    def _finish_goal_probe(self):
        stop_reason = self.goal_probe_stop_reason
        self._stop_goal_probe()

        if stop_reason not in ["DURATION_REACHED", ""]:
            self.get_logger().warn(f"Goal probe did not complete safely: {stop_reason}")
            self._start_controller_patience_or_side_escape()
            return

        self._send_original_goal(
            NavPurpose.RETRY_AFTER_GOAL_PROBE,
            MissionState.RETRY_GOAL_AFTER_GOAL_PROBE,
        )

    def _stop_goal_probe(self):
        self.goal_probe_phase = GoalProbePhase.IDLE
        self.goal_probe_mission_id = None
        self.goal_probe_phase_start_sec = None
        self.goal_probe_active_start_sec = None
        self.goal_probe_command = Twist()
        self.goal_probe_heading_rad = 0.0
        self.goal_probe_stop_reason = ""
        self._publish_zero_cmd()

    def _goal_probe_phase_elapsed(self, now_sec: float) -> float:
        if self.goal_probe_phase_start_sec is None:
            return 0.0
        return now_sec - self.goal_probe_phase_start_sec

    def _select_goal_probe_command(self) -> Optional[Tuple[Twist, float, str]]:
        if self.original_goal is None:
            return None

        if self.latest_scan is None or self.latest_scan_time_sec is None:
            return None

        if self._now_sec() - self.latest_scan_time_sec > self.scan_stale_sec:
            return None

        current_pose = self._get_current_pose()
        if current_pose is None:
            return None

        dx = self.original_goal.pose.position.x - current_pose.pose.position.x
        dy = self.original_goal.pose.position.y - current_pose.pose.position.y
        goal_distance = math.hypot(dx, dy)
        if goal_distance <= 0.20:
            return None

        robot_yaw = self._yaw_from_pose(current_pose)
        goal_heading = math.atan2(dy, dx)
        relative_goal_heading = self._angle_diff(goal_heading, robot_yaw)
        max_goal_heading = math.radians(max(0.0, self.goal_probe_max_goal_heading_deg))
        if abs(relative_goal_heading) > max_goal_heading:
            return None

        front_clearance = self._scan_clearance_at_heading(0.0, 35.0)
        if front_clearance is None or front_clearance < self.goal_probe_front_clearance:
            return None

        candidates = self._goal_probe_candidate_order(relative_goal_heading)
        best = None
        best_score = None
        sector_width = max(25.0, min(60.0, abs(self.goal_probe_heading_deg) * 2.0))

        for order_index, candidate_name in enumerate(candidates):
            command_candidate = self._make_goal_probe_command(candidate_name)
            if command_candidate is None:
                continue

            command, heading_rad = command_candidate
            clearance = self._scan_clearance_at_heading(heading_rad, sector_width)
            if clearance is None or clearance < self.goal_probe_side_clearance:
                continue

            score = min(clearance, front_clearance) - order_index * 0.01
            if candidate_name == "forward":
                if abs(relative_goal_heading) <= math.radians(
                    self.goal_probe_straight_heading_deg
                ):
                    score += 0.08
            else:
                candidate_sign = 1.0 if candidate_name == "forward_left" else -1.0
                goal_sign = 1.0 if relative_goal_heading >= 0.0 else -1.0
                if candidate_sign == goal_sign:
                    score += 0.12
                else:
                    score -= 0.12

            summary = (
                f"candidate={candidate_name}, "
                f"goal_heading={math.degrees(relative_goal_heading):.1f}deg, "
                f"linear={command.linear.x:.2f}m/s, "
                f"angular={command.angular.z:.2f}rad/s, "
                f"front_clearance={front_clearance:.2f}m, "
                f"probe_clearance={clearance:.2f}m, "
                f"score={score:.2f}"
            )
            if best_score is None or score > best_score:
                best = command, heading_rad, summary
                best_score = score

        return best

    def _goal_probe_candidate_order(self, relative_goal_heading: float):
        straight_limit = math.radians(max(0.0, self.goal_probe_straight_heading_deg))
        if relative_goal_heading > straight_limit:
            return ["forward_left", "forward", "forward_right"]
        if relative_goal_heading < -straight_limit:
            return ["forward_right", "forward", "forward_left"]
        return ["forward", "forward_left", "forward_right"]

    def _make_goal_probe_command(
        self,
        candidate_name: str,
    ) -> Optional[Tuple[Twist, float]]:
        speed = abs(self.goal_probe_forward_speed)
        turn_speed = abs(self.goal_probe_angular_speed)
        heading = math.radians(abs(self.goal_probe_heading_deg))
        if speed <= 0.0:
            return None

        command = Twist()
        command.linear.x = speed

        if candidate_name == "forward_left":
            command.angular.z = turn_speed
            relative_heading = heading
        elif candidate_name == "forward_right":
            command.angular.z = -turn_speed
            relative_heading = -heading
        elif candidate_name == "forward":
            command.angular.z = 0.0
            relative_heading = 0.0
        else:
            return None

        return command, relative_heading

    # ------------------------------------------------------------------
    # Controller patience maneuver
    # ------------------------------------------------------------------

    def _begin_controller_patience(self, retry_fallback: str) -> bool:
        if not self.controller_patience_enabled:
            return False

        if (
            self.controller_patience_attempts_this_stuck_event
            >= self.controller_patience_max_attempts_per_stuck_event
        ):
            return False

        if (
            self.controller_patience_attempts_this_mission
            >= self.controller_patience_max_attempts_per_mission
        ):
            return False

        selection = self._select_controller_patience_command()
        if selection is None:
            self.get_logger().warn(
                "Controller patience skipped: no safe forward-turn command."
            )
            return False

        if not self._use_recovery_cycle("controller patience"):
            self._prepare_return_to_base()
            return True

        command, heading_rad, summary = selection
        self._cancel_active_nav_goal("CONTROLLER_PATIENCE")
        self._publish_zero_cmd()

        self.controller_patience_attempts_this_stuck_event += 1
        self.controller_patience_attempts_this_mission += 1
        self.controller_patience_retry_fallback = retry_fallback
        self.controller_patience_mission_id = self.mission_id
        self.controller_patience_phase = ControllerPatiencePhase.ACTIVE
        self.controller_patience_phase_start_sec = self._now_sec()
        self.controller_patience_active_start_sec = self.controller_patience_phase_start_sec
        self.controller_patience_command = command
        self.controller_patience_heading_rad = heading_rad
        self.controller_patience_stop_reason = ""

        self._set_state(
            MissionState.CONTROLLER_PATIENCE_ACTIVE,
            "CONTROLLER_PATIENCE_ACTIVE",
        )
        self.get_logger().warn(
            f"Starting controller patience maneuver on /cmd_vel: {summary}"
        )
        return True

    def _run_controller_patience_state_machine(self, now_sec: float):
        if self.controller_patience_phase == ControllerPatiencePhase.IDLE:
            return

        if self.controller_patience_mission_id != self.mission_id:
            self._stop_controller_patience()
            return

        if self.controller_patience_phase == ControllerPatiencePhase.ACTIVE:
            should_stop, reason = self._controller_patience_should_stop(now_sec)
            if should_stop:
                self.controller_patience_stop_reason = reason
                self._publish_zero_cmd()
                self.controller_patience_phase = ControllerPatiencePhase.ZERO_AFTER
                self.controller_patience_phase_start_sec = now_sec
                self._set_state(
                    MissionState.WAIT_AFTER_CONTROLLER_PATIENCE,
                    f"CONTROLLER_PATIENCE_STOPPED_{reason}",
                )
                return

            if self._controller_patience_active_elapsed(now_sec) >= (
                self.goal_retry_preemption_min_active_sec
            ):
                if self._maybe_preempt_recovery_to_goal(
                    now_sec,
                    "controller patience",
                ):
                    return

            return

        if self.controller_patience_phase == ControllerPatiencePhase.ZERO_AFTER:
            self._publish_zero_cmd()
            if self._maybe_preempt_recovery_to_goal(
                now_sec,
                "controller patience zero",
            ):
                return
            if self._controller_patience_phase_elapsed(now_sec) >= (
                self.controller_patience_zero_cmd_after_sec
            ):
                self.controller_patience_phase = ControllerPatiencePhase.WAIT_AFTER
                self.controller_patience_phase_start_sec = now_sec
            return

        if self.controller_patience_phase == ControllerPatiencePhase.WAIT_AFTER:
            self._publish_zero_cmd()
            if self._maybe_preempt_recovery_to_goal(
                now_sec,
                "controller patience wait",
            ):
                return
            if self._controller_patience_phase_elapsed(now_sec) >= (
                self.controller_patience_wait_after_sec
            ):
                self._finish_controller_patience()

    def _controller_patience_should_stop(self, now_sec: float) -> Tuple[bool, str]:
        if self.controller_patience_active_start_sec is None:
            return False, ""

        elapsed = now_sec - self.controller_patience_active_start_sec
        if elapsed >= self.controller_patience_duration_sec:
            return True, "DURATION_REACHED"

        clearance = self._scan_clearance_at_heading(
            self.controller_patience_heading_rad,
            max(25.0, min(70.0, abs(self.controller_patience_heading_deg) * 2.0)),
        )
        if clearance is None:
            return True, "SCAN_LOST"

        if clearance < self.controller_patience_hard_stop_clearance:
            return True, "MANEUVER_UNSAFE"

        front_clearance = self._scan_clearance_at_heading(0.0, 40.0)
        if front_clearance is None:
            return True, "FRONT_SCAN_LOST"

        if front_clearance < self.controller_patience_hard_stop_clearance:
            return True, "FRONT_UNSAFE"

        if not rclpy.ok():
            return True, "ROS_SHUTDOWN"

        return False, ""

    def _finish_controller_patience(self):
        stop_reason = self.controller_patience_stop_reason
        self._stop_controller_patience()

        if stop_reason not in ["DURATION_REACHED", ""]:
            self.get_logger().warn(
                f"Controller patience did not complete safely: {stop_reason}"
            )
            self._handle_controller_patience_retry_failure()
            return

        self._send_original_goal(
            NavPurpose.RETRY_AFTER_CONTROLLER_PATIENCE,
            MissionState.RETRY_GOAL_AFTER_CONTROLLER_PATIENCE,
        )

    def _stop_controller_patience(self):
        self.controller_patience_phase = ControllerPatiencePhase.IDLE
        self.controller_patience_mission_id = None
        self.controller_patience_phase_start_sec = None
        self.controller_patience_active_start_sec = None
        self.controller_patience_command = Twist()
        self.controller_patience_heading_rad = 0.0
        self.controller_patience_stop_reason = ""
        self._publish_zero_cmd()

    def _controller_patience_phase_elapsed(self, now_sec: float) -> float:
        if self.controller_patience_phase_start_sec is None:
            return 0.0
        return now_sec - self.controller_patience_phase_start_sec

    # ------------------------------------------------------------------
    # Direct side escape
    # ------------------------------------------------------------------

    def _run_direct_side_escape_state_machine(self, now_sec: float):
        if self.direct_side_phase == DirectSideEscapePhase.IDLE:
            return

        if self.direct_side_mission_id != self.mission_id:
            self._stop_direct_side_escape()
            return

        if self.direct_side_phase == DirectSideEscapePhase.ACTIVE:
            should_stop, reason = self._direct_side_escape_should_stop(now_sec)
            if should_stop:
                self.direct_side_stop_reason = reason
                self._publish_zero_cmd()
                self.direct_side_phase = DirectSideEscapePhase.ZERO_AFTER
                self.direct_side_phase_start_sec = now_sec
                self._set_state(
                    MissionState.WAIT_AFTER_DIRECT_SIDE_ESCAPE,
                    f"DIRECT_SIDE_ESCAPE_STOPPED_{reason}",
                )
                return

            if self._direct_side_active_elapsed(now_sec) >= (
                self.goal_retry_preemption_min_active_sec
            ):
                if self._maybe_preempt_recovery_to_goal(now_sec, "direct side escape"):
                    return

            return

        if self.direct_side_phase == DirectSideEscapePhase.ZERO_AFTER:
            self._publish_zero_cmd()
            if self._maybe_preempt_recovery_to_goal(
                now_sec,
                "direct side escape zero",
            ):
                return
            if self._direct_side_phase_elapsed(now_sec) >= (
                self.direct_side_escape_zero_cmd_after_sec
            ):
                self.direct_side_phase = DirectSideEscapePhase.WAIT_AFTER
                self.direct_side_phase_start_sec = now_sec
            return

        if self.direct_side_phase == DirectSideEscapePhase.WAIT_AFTER:
            self._publish_zero_cmd()
            if self._maybe_preempt_recovery_to_goal(
                now_sec,
                "direct side escape wait",
            ):
                return
            if self._direct_side_phase_elapsed(now_sec) >= (
                self.direct_side_escape_wait_after_sec
            ):
                self._finish_direct_side_escape()

    def _direct_side_escape_should_stop(self, now_sec: float) -> Tuple[bool, str]:
        if self.direct_side_active_start_sec is None:
            return False, ""

        elapsed = now_sec - self.direct_side_active_start_sec
        if elapsed >= self.direct_side_escape_duration_sec:
            return True, "DURATION_REACHED"

        clearance = self._scan_clearance_at_heading(
            self.direct_side_heading_rad,
            max(20.0, min(60.0, abs(self.side_escape_yaw_offset_deg) * 2.0)),
        )
        if clearance is None:
            return True, "SCAN_LOST"

        if clearance < self.direct_side_escape_hard_stop_clearance:
            return True, "SIDE_UNSAFE"

        forward_heading = 0.0 if self.direct_side_command.linear.x >= 0.0 else math.pi
        forward_clearance = self._scan_clearance_at_heading(forward_heading, 35.0)
        if forward_clearance is None:
            return True, "TRAVEL_SCAN_LOST"

        if forward_clearance < self.direct_side_escape_hard_stop_clearance:
            return True, "TRAVEL_UNSAFE"

        if not rclpy.ok():
            return True, "ROS_SHUTDOWN"

        return False, ""

    def _finish_direct_side_escape(self):
        stop_reason = self.direct_side_stop_reason
        self._stop_direct_side_escape()

        if stop_reason not in ["DURATION_REACHED", ""]:
            self.get_logger().warn(
                f"Direct side escape did not complete safely: {stop_reason}"
            )
            self._select_breadcrumb_or_return_to_base()
            return

        self.goal_retries_after_reverse += 1
        self._send_original_goal(
            NavPurpose.RETRY_FROM_SIDE_ESCAPE,
            MissionState.RETRY_GOAL_FROM_SIDE_ESCAPE,
        )

    def _stop_direct_side_escape(self):
        self.direct_side_phase = DirectSideEscapePhase.IDLE
        self.direct_side_mission_id = None
        self.direct_side_phase_start_sec = None
        self.direct_side_active_start_sec = None
        self.direct_side_command = Twist()
        self.direct_side_heading_rad = 0.0
        self.direct_side_stop_reason = ""
        self._publish_zero_cmd()

    def _direct_side_phase_elapsed(self, now_sec: float) -> float:
        if self.direct_side_phase_start_sec is None:
            return 0.0
        return now_sec - self.direct_side_phase_start_sec

    # ------------------------------------------------------------------
    # Reverse escape
    # ------------------------------------------------------------------

    def _begin_reverse_escape(self, context: str) -> bool:
        allowed_distance, allowed_duration, reason = self._compute_reverse_profile()

        if allowed_distance <= 0.0 or allowed_duration <= 0.0:
            self.get_logger().warn(f"Reverse escape skipped: {reason}")
            self._publish_status(f"REVERSE_SKIPPED_{reason}")
            return False

        self._cancel_active_nav_goal("REVERSE_ESCAPE")
        self._publish_zero_cmd()

        self.reverse_context = context
        self.reverse_mission_id = self.mission_id
        self.reverse_phase = ReversePhase.ZERO_BEFORE
        self.reverse_phase_start_sec = self._now_sec()
        self.reverse_active_start_sec = None
        self.reverse_start_odom_xy = None
        self.reverse_start_tf_pose = None
        self.reverse_allowed_distance = allowed_distance
        self.reverse_allowed_duration = allowed_duration
        self.reverse_stop_reason = ""

        if context == ReverseContext.GOAL:
            self.reverse_escapes_this_stuck_event += 1
        self.reverse_escapes_this_mission += 1

        state = (
            MissionState.REVERSE_ESCAPE_FOR_BASE
            if context == ReverseContext.BASE
            else MissionState.REVERSE_ESCAPE_PRECHECK
        )
        self._set_state(state, f"REVERSE_ESCAPE_PRECHECK_{context.upper()}")
        self.get_logger().warn(
            f"Starting reverse escape context={context}, "
            f"distance={allowed_distance:.2f}m, duration={allowed_duration:.2f}s"
        )
        return True

    def _run_reverse_state_machine(self, now_sec: float):
        if self.reverse_phase == ReversePhase.IDLE:
            return

        if self.reverse_mission_id != self.mission_id:
            self._stop_reverse_escape()
            return

        if self.reverse_phase == ReversePhase.ZERO_BEFORE:
            self._publish_zero_cmd()
            if self._phase_elapsed(now_sec) >= self.reverse_zero_cmd_before_sec:
                self.reverse_phase = ReversePhase.ACTIVE
                self.reverse_phase_start_sec = now_sec
                self.reverse_active_start_sec = now_sec
                self.reverse_start_odom_xy = self._current_odom_xy_if_fresh(now_sec)
                self.reverse_start_tf_pose = self._get_current_pose()
                self._set_state(MissionState.REVERSE_ESCAPE_ACTIVE, "REVERSE_ESCAPE_ACTIVE")
            return

        if self.reverse_phase == ReversePhase.ACTIVE:
            should_stop, reason = self._reverse_should_stop(now_sec)
            if should_stop:
                self.reverse_stop_reason = reason
                self._publish_zero_cmd()
                self.reverse_phase = ReversePhase.ZERO_AFTER
                self.reverse_phase_start_sec = now_sec
                self._set_state(MissionState.WAIT_AFTER_REVERSE, f"REVERSE_STOPPED_{reason}")
                return

            if self._reverse_active_elapsed(now_sec) >= (
                self.goal_retry_preemption_min_active_sec
            ):
                if self._maybe_preempt_recovery_to_goal(now_sec, "reverse escape"):
                    return

            return

        if self.reverse_phase == ReversePhase.ZERO_AFTER:
            self._publish_zero_cmd()
            if self.reverse_context == ReverseContext.GOAL:
                if self._maybe_preempt_recovery_to_goal(now_sec, "reverse zero"):
                    return
            if self._phase_elapsed(now_sec) >= self.reverse_zero_cmd_after_sec:
                self.reverse_phase = ReversePhase.WAIT_AFTER
                self.reverse_phase_start_sec = now_sec
            return

        if self.reverse_phase == ReversePhase.WAIT_AFTER:
            self._publish_zero_cmd()
            if self.reverse_context == ReverseContext.GOAL:
                if self._maybe_preempt_recovery_to_goal(now_sec, "reverse wait"):
                    return
            if self._phase_elapsed(now_sec) >= self.post_reverse_costmap_wait_sec:
                self._finish_reverse_escape()

    def _finish_reverse_escape(self):
        context = self.reverse_context
        self._stop_reverse_escape()

        if context == ReverseContext.BASE:
            self._send_base_goal()
            return

        self.goal_retries_after_reverse += 1
        self._send_original_goal(
            NavPurpose.RETRY_AFTER_REVERSE,
            MissionState.RETRY_GOAL_AFTER_REVERSE,
        )

    def _stop_reverse_escape(self):
        self.reverse_phase = ReversePhase.IDLE
        self.reverse_context = None
        self.reverse_mission_id = None
        self.reverse_phase_start_sec = None
        self.reverse_active_start_sec = None
        self.reverse_start_odom_xy = None
        self.reverse_start_tf_pose = None
        self.reverse_allowed_duration = 0.0
        self.reverse_allowed_distance = 0.0
        self.reverse_stop_reason = ""
        self._publish_zero_cmd()

    def _compute_reverse_profile(self) -> Tuple[float, float, str]:
        speed = abs(self.reverse_speed)
        if speed <= 0.001:
            return 0.0, 0.0, "BAD_REVERSE_SPEED"

        target = min(self.reverse_target_distance, self.reverse_absolute_max_distance)

        if not self.rear_check_enabled:
            return target, min(self.reverse_max_duration, target / speed), "FULL_NO_REAR_CHECK"

        clearance = self._get_rear_clearance()

        if clearance is None:
            if self.allow_reverse_without_scan:
                return target, min(self.reverse_max_duration, target / speed), "FULL_NO_SCAN_ALLOWED"
            return 0.0, 0.0, "NO_REAR_SCAN"

        if clearance >= self.rear_clearance_full_reverse:
            distance = target
            label = "FULL_REVERSE"
        elif clearance >= self.rear_clearance_short_reverse:
            distance = min(target, max(0.0, clearance - self.rear_clearance_hard_stop))
            label = "SHORT_REVERSE"
        elif clearance >= self.rear_clearance_hard_stop:
            distance = min(0.05, max(0.0, clearance - self.rear_clearance_hard_stop))
            label = "TINY_REVERSE"
        else:
            return 0.0, 0.0, "REAR_BLOCKED"

        distance = min(distance, self.reverse_absolute_max_distance)
        duration = min(self.reverse_max_duration, distance / speed)
        return distance, duration, label

    def _reverse_should_stop(self, now_sec: float) -> Tuple[bool, str]:
        if self.reverse_active_start_sec is None:
            return False, ""

        elapsed = now_sec - self.reverse_active_start_sec
        if elapsed >= self.reverse_allowed_duration:
            return True, "DURATION_REACHED"

        distance = self._current_reverse_distance(now_sec)
        if distance is not None:
            if distance >= self.reverse_absolute_max_distance:
                return True, "ABSOLUTE_DISTANCE_REACHED"
            if distance >= self.reverse_allowed_distance:
                return True, "TARGET_DISTANCE_REACHED"

        if self.rear_check_enabled:
            clearance = self._get_rear_clearance()
            if clearance is None and not self.allow_reverse_without_scan:
                return True, "REAR_SCAN_LOST"
            if clearance is not None and clearance < self.rear_clearance_hard_stop:
                return True, "REAR_UNSAFE"

        if not rclpy.ok():
            return True, "ROS_SHUTDOWN"

        return False, ""

    def _get_rear_clearance(self) -> Optional[float]:
        if self.latest_scan is None or self.latest_scan_time_sec is None:
            return None

        if self._now_sec() - self.latest_scan_time_sec > self.scan_stale_sec:
            return None

        scan = self.latest_scan
        center = math.radians(self.rear_sector_center_degrees)
        half_width = math.radians(self.rear_sector_degrees) * 0.5
        min_valid_range = max(scan.range_min, 0.0)
        max_valid_range = scan.range_max if scan.range_max > 0.0 else math.inf

        best = math.inf
        for index, scan_range in enumerate(scan.ranges):
            if not math.isfinite(scan_range):
                continue

            if scan_range < min_valid_range or scan_range > max_valid_range:
                continue

            angle = scan.angle_min + index * scan.angle_increment
            if abs(self._angle_diff(angle, center)) <= half_width:
                best = min(best, scan_range)

        if math.isinf(best):
            return None

        return best

    def _select_side_escape_pose(
        self,
        preferred_side: Optional[str] = None,
    ) -> Optional[PoseStamped]:
        current_pose = self._get_current_pose()
        if current_pose is None:
            self.get_logger().warn("Side escape skipped: current pose unavailable.")
            return None

        if self.latest_scan is None or self.latest_scan_time_sec is None:
            self.get_logger().warn("Side escape skipped: no LaserScan available.")
            return None

        if self._now_sec() - self.latest_scan_time_sec > self.scan_stale_sec:
            self.get_logger().warn("Side escape skipped: LaserScan is stale.")
            return None

        robot_yaw = self._yaw_from_pose(current_pose)
        best_pose = None
        best_score = None
        best_summary = ""

        candidate_order = self._phase5_order_candidates(
            self.side_escape_candidate_order,
            preferred_side,
        )
        for order_index, candidate_name in enumerate(candidate_order):
            candidate = self._make_side_escape_candidate(
                current_pose,
                robot_yaw,
                candidate_name,
            )
            if candidate is None:
                continue

            candidate_pose, relative_heading, travel_distance = candidate
            sector_width_deg = max(
                20.0,
                min(60.0, abs(self.side_escape_yaw_offset_deg) * 2.0),
            )
            clearance = self._scan_clearance_at_heading(
                relative_heading,
                sector_width_deg,
            )
            required_clearance = max(
                self.side_escape_clearance_required,
                travel_distance,
            )

            if clearance is None or clearance < required_clearance:
                continue

            if self.route_memory.pose_in_failed_zone(
                candidate_pose
            ) and not self._phase5_can_relax_failed_zone(clearance, required_clearance):
                continue

            if self._pose_near_any(
                candidate_pose,
                self.failed_escape_poses,
                self.failed_escape_goal_radius,
            ):
                continue

            score = clearance - order_index * 0.01
            if best_score is None or score > best_score:
                best_pose = candidate_pose
                best_score = score
                best_summary = (
                    f"candidate={candidate_name}, distance={travel_distance:.2f}m, "
                    f"clearance={clearance:.2f}m, score={score:.2f}"
                )

        if best_pose is None:
            self.get_logger().warn("Side escape found no safe scan-validated candidate.")
            return None

        self.get_logger().warn(f"Selected side escape candidate: {best_summary}")
        return best_pose

    def _phase5_can_relax_failed_zone(
        self,
        clearance: Optional[float],
        required_clearance: float,
    ) -> bool:
        if not self.phase5_relax_failed_zone_on_open_scan:
            return False

        if clearance is None:
            return False

        return clearance >= required_clearance + max(0.0, self.phase5_failed_zone_relax_margin)

    def _select_controller_patience_command(
        self,
    ) -> Optional[Tuple[Twist, float, str]]:
        if self.latest_scan is None or self.latest_scan_time_sec is None:
            return None

        if self._now_sec() - self.latest_scan_time_sec > self.scan_stale_sec:
            return None

        best = None
        best_score = None
        sector_width = max(
            25.0,
            min(70.0, abs(self.controller_patience_heading_deg) * 2.0),
        )

        for order_index, candidate_name in enumerate(
            self.controller_patience_candidate_order
        ):
            command_candidate = self._make_controller_patience_command(candidate_name)
            if command_candidate is None:
                continue

            command, heading_rad = command_candidate
            maneuver_clearance = self._scan_clearance_at_heading(
                heading_rad,
                sector_width,
            )
            if (
                maneuver_clearance is None
                or maneuver_clearance < self.controller_patience_clearance_required
            ):
                continue

            front_clearance = self._scan_clearance_at_heading(0.0, 40.0)
            if (
                front_clearance is None
                or front_clearance < self.controller_patience_hard_stop_clearance
            ):
                continue

            score = min(maneuver_clearance, front_clearance) - order_index * 0.01
            summary = (
                f"candidate={candidate_name}, "
                f"linear={command.linear.x:.2f}m/s, "
                f"angular={command.angular.z:.2f}rad/s, "
                f"maneuver_clearance={maneuver_clearance:.2f}m, "
                f"front_clearance={front_clearance:.2f}m"
            )
            if best_score is None or score > best_score:
                best = command, heading_rad, summary
                best_score = score

        return best

    def _make_controller_patience_command(
        self,
        candidate_name: str,
    ) -> Optional[Tuple[Twist, float]]:
        direction = candidate_name.lower()
        speed = abs(self.controller_patience_forward_speed)
        turn_speed = abs(self.controller_patience_angular_speed)
        heading = math.radians(abs(self.controller_patience_heading_deg))
        if speed <= 0.0:
            return None

        command = Twist()
        command.linear.x = speed

        if direction == "forward_left":
            command.angular.z = turn_speed
            relative_heading = heading
        elif direction == "forward_right":
            command.angular.z = -turn_speed
            relative_heading = -heading
        elif direction == "forward":
            command.angular.z = 0.0
            relative_heading = 0.0
        else:
            return None

        return command, relative_heading

    def _select_direct_side_escape_command(
        self,
        preferred_side: Optional[str] = None,
    ) -> Optional[Tuple[Twist, float, str]]:
        if self.latest_scan is None or self.latest_scan_time_sec is None:
            return None

        if self._now_sec() - self.latest_scan_time_sec > self.scan_stale_sec:
            return None

        best = None
        best_score = None
        candidate_order = self._phase5_order_candidates(
            self.side_escape_candidate_order,
            preferred_side,
        )
        for order_index, candidate_name in enumerate(candidate_order):
            command_candidate = self._make_direct_side_escape_command(candidate_name)
            if command_candidate is None:
                continue

            command, heading_rad = command_candidate
            side_clearance = self._scan_clearance_at_heading(
                heading_rad,
                max(20.0, min(60.0, abs(self.side_escape_yaw_offset_deg) * 2.0)),
            )
            if (
                side_clearance is None
                or side_clearance < self.direct_side_escape_clearance_required
            ):
                continue

            travel_heading = 0.0 if command.linear.x >= 0.0 else math.pi
            travel_clearance = self._scan_clearance_at_heading(travel_heading, 35.0)
            if (
                travel_clearance is None
                or travel_clearance < self.direct_side_escape_hard_stop_clearance
            ):
                continue

            score = min(side_clearance, travel_clearance) - order_index * 0.01
            summary = (
                f"candidate={candidate_name}, "
                f"linear={command.linear.x:.2f}m/s, "
                f"angular={command.angular.z:.2f}rad/s, "
                f"side_clearance={side_clearance:.2f}m, "
                f"travel_clearance={travel_clearance:.2f}m"
            )
            if best_score is None or score > best_score:
                best = command, heading_rad, summary
                best_score = score

        return best

    def _make_direct_side_escape_command(
        self,
        candidate_name: str,
    ) -> Optional[Tuple[Twist, float]]:
        direction = candidate_name.lower()
        speed = abs(self.direct_side_escape_speed)
        turn_speed = abs(self.direct_side_escape_angular_speed)
        if speed <= 0.0 or turn_speed <= 0.0:
            return None

        command = Twist()
        forward_bias = max(0.01, self.side_escape_forward_bias)
        lateral = max(0.0, self.side_escape_distance)

        if direction == "left":
            command.linear.x = speed
            command.angular.z = turn_speed
            relative_heading = math.atan2(lateral, forward_bias)
        elif direction == "right":
            command.linear.x = speed
            command.angular.z = -turn_speed
            relative_heading = math.atan2(-lateral, forward_bias)
        elif direction == "back_left":
            command.linear.x = -speed
            command.angular.z = -turn_speed
            relative_heading = math.atan2(lateral, -forward_bias)
        elif direction == "back_right":
            command.linear.x = -speed
            command.angular.z = turn_speed
            relative_heading = math.atan2(-lateral, -forward_bias)
        else:
            return None

        return command, relative_heading

    def _make_side_escape_candidate(
        self,
        current_pose: PoseStamped,
        robot_yaw: float,
        candidate_name: str,
    ) -> Optional[Tuple[PoseStamped, float, float]]:
        direction = candidate_name.lower()
        distance = max(0.0, self.side_escape_distance)
        forward_bias = max(0.0, self.side_escape_forward_bias)
        yaw_offset = math.radians(abs(self.side_escape_yaw_offset_deg))

        if direction == "left":
            forward = forward_bias
            lateral = distance
            goal_yaw = robot_yaw + yaw_offset
        elif direction == "right":
            forward = forward_bias
            lateral = -distance
            goal_yaw = robot_yaw - yaw_offset
        elif direction == "back_left":
            forward = -forward_bias
            lateral = distance
            goal_yaw = robot_yaw + yaw_offset
        elif direction == "back_right":
            forward = -forward_bias
            lateral = -distance
            goal_yaw = robot_yaw - yaw_offset
        else:
            self.get_logger().warn(
                f"Ignoring unknown side escape candidate '{candidate_name}'."
            )
            return None

        if distance <= 0.0:
            return None

        world_x = (
            current_pose.pose.position.x
            + forward * math.cos(robot_yaw)
            - lateral * math.sin(robot_yaw)
        )
        world_y = (
            current_pose.pose.position.y
            + forward * math.sin(robot_yaw)
            + lateral * math.cos(robot_yaw)
        )
        travel_distance = math.hypot(forward, lateral)
        relative_heading = math.atan2(lateral, forward)
        return (
            self._make_pose_from_xy_yaw(world_x, world_y, goal_yaw),
            relative_heading,
            travel_distance,
        )

    def _scan_clearance_at_heading(
        self,
        heading_rad: float,
        sector_degrees: float,
    ) -> Optional[float]:
        scan = self.latest_scan
        if scan is None:
            return None

        half_width = math.radians(sector_degrees) * 0.5
        min_valid_range = max(scan.range_min, 0.0)
        max_valid_range = scan.range_max if scan.range_max > 0.0 else math.inf

        best = math.inf
        for index, scan_range in enumerate(scan.ranges):
            if not math.isfinite(scan_range):
                continue

            if scan_range < min_valid_range or scan_range > max_valid_range:
                continue

            angle = scan.angle_min + index * scan.angle_increment
            if abs(self._angle_diff(angle, heading_rad)) <= half_width:
                best = min(best, scan_range)

        if math.isinf(best):
            return None

        return best

    def _current_reverse_distance(self, now_sec: float) -> Optional[float]:
        current_odom = self._current_odom_xy_if_fresh(now_sec)
        if self.reverse_start_odom_xy is not None and current_odom is not None:
            return self._distance_xy_tuple(self.reverse_start_odom_xy, current_odom)

        if self.reverse_start_tf_pose is None:
            return None

        current_pose = self._get_current_pose()
        if current_pose is None:
            return None

        return self.route_memory.distance_xy(self.reverse_start_tf_pose, current_pose)

    def _reverse_active_elapsed(self, now_sec: float) -> float:
        if self.reverse_active_start_sec is None:
            return 0.0
        return now_sec - self.reverse_active_start_sec

    def _controller_patience_active_elapsed(self, now_sec: float) -> float:
        if self.controller_patience_active_start_sec is None:
            return 0.0
        return now_sec - self.controller_patience_active_start_sec

    def _goal_probe_active_elapsed(self, now_sec: float) -> float:
        if self.goal_probe_active_start_sec is None:
            return 0.0
        return now_sec - self.goal_probe_active_start_sec

    def _direct_side_active_elapsed(self, now_sec: float) -> float:
        if self.direct_side_active_start_sec is None:
            return 0.0
        return now_sec - self.direct_side_active_start_sec

    def _maybe_preempt_recovery_to_goal(self, now_sec: float, context: str) -> bool:
        purpose, state = self._goal_retry_purpose_for_active_recovery()
        if purpose is None or state is None:
            return False

        return self._send_goal_retry_if_open(now_sec, purpose, state, context)

    def _send_goal_retry_if_open(
        self,
        now_sec: float,
        purpose: str,
        state: MissionState,
        context: str,
        ignore_failure_cooldown: bool = False,
        require_stable_ticks: bool = True,
    ) -> bool:
        if (
            purpose == NavPurpose.RETRY_AFTER_REVERSE
            and self.goal_retries_after_reverse >= self.max_goal_retries_after_reverse
        ):
            self.goal_corridor_clear_ticks = 0
            return False

        summary = self._open_goal_corridor_summary(
            now_sec,
            ignore_failure_cooldown=ignore_failure_cooldown,
        )
        if summary is None:
            self.goal_corridor_clear_ticks = 0
            return False

        if require_stable_ticks:
            self.goal_corridor_clear_ticks += 1
            required_ticks = max(1, self.goal_retry_preemption_required_stable_ticks)
            if self.goal_corridor_clear_ticks < required_ticks:
                return False

        self.goal_corridor_clear_ticks = 0
        if purpose == NavPurpose.RETRY_AFTER_REVERSE:
            self.goal_retries_after_reverse += 1

        self.last_goal_preemption_sec = now_sec
        self.get_logger().warn(
            f"Goal corridor opened during {context}; retrying original goal. {summary}"
        )
        self._stop_reverse_escape()
        self._stop_goal_probe()
        self._stop_controller_patience()
        self._stop_direct_side_escape()
        self._send_original_goal(purpose, state)
        return True

    def _goal_retry_purpose_for_active_recovery(
        self,
    ) -> Tuple[Optional[str], Optional[MissionState]]:
        if self.reverse_phase != ReversePhase.IDLE:
            if self.reverse_context == ReverseContext.BASE:
                return None, None
            return NavPurpose.RETRY_AFTER_REVERSE, MissionState.RETRY_GOAL_AFTER_REVERSE

        if self.goal_probe_phase != GoalProbePhase.IDLE:
            return (
                NavPurpose.RETRY_AFTER_GOAL_PROBE,
                MissionState.RETRY_GOAL_AFTER_GOAL_PROBE,
            )

        if self.controller_patience_phase != ControllerPatiencePhase.IDLE:
            return (
                NavPurpose.RETRY_AFTER_CONTROLLER_PATIENCE,
                MissionState.RETRY_GOAL_AFTER_CONTROLLER_PATIENCE,
            )

        if self.direct_side_phase != DirectSideEscapePhase.IDLE:
            return NavPurpose.RETRY_FROM_SIDE_ESCAPE, MissionState.RETRY_GOAL_FROM_SIDE_ESCAPE

        return None, None

    def _open_goal_corridor_summary(
        self,
        now_sec: float,
        ignore_failure_cooldown: bool = False,
    ) -> Optional[str]:
        if not self.goal_retry_preemption_enabled:
            return None

        if now_sec - self.last_goal_preemption_sec < (
            self.goal_retry_preemption_cooldown_sec
        ):
            return None

        if self.original_goal is None:
            return None

        if self.latest_scan is None or self.latest_scan_time_sec is None:
            return None

        if now_sec - self.latest_scan_time_sec > self.scan_stale_sec:
            return None

        current_pose = self._get_current_pose()
        if current_pose is None:
            return None

        if (
            not ignore_failure_cooldown
            and self._goal_retry_failure_cooldown_active(now_sec, current_pose)
        ):
            return None

        dx = self.original_goal.pose.position.x - current_pose.pose.position.x
        dy = self.original_goal.pose.position.y - current_pose.pose.position.y
        goal_distance = math.hypot(dx, dy)
        if goal_distance <= 0.20:
            return f"goal_distance={goal_distance:.2f}m"

        robot_yaw = self._yaw_from_pose(current_pose)
        goal_heading = math.atan2(dy, dx)
        relative_heading = self._angle_diff(goal_heading, robot_yaw)
        max_heading_error = math.radians(
            max(0.0, self.goal_retry_preemption_max_heading_error_degrees)
        )
        if abs(relative_heading) > max_heading_error:
            return None

        front_clearance = self._scan_clearance_at_heading(
            0.0,
            min(60.0, max(25.0, self.goal_retry_preemption_sector_degrees)),
        )
        if front_clearance is None:
            return None

        if front_clearance < self.goal_retry_preemption_front_clearance:
            return None

        clearance = self._scan_clearance_at_heading(
            relative_heading,
            self.goal_retry_preemption_sector_degrees,
        )
        if clearance is None:
            return None

        required_clearance = min(
            max(0.25, self.goal_retry_preemption_clearance),
            max(0.25, goal_distance),
        )
        if clearance < required_clearance:
            return None

        return (
            f"goal_distance={goal_distance:.2f}m, "
            f"heading_error={math.degrees(relative_heading):.1f}deg, "
            f"clearance={clearance:.2f}m, "
            f"front_clearance={front_clearance:.2f}m, "
            f"required={required_clearance:.2f}m"
        )

    def _record_goal_retry_failure(self, purpose: str):
        if purpose not in [
            NavPurpose.RETRY_AFTER_CONTROLLER_PATIENCE,
            NavPurpose.RETRY_AFTER_REVERSE,
            NavPurpose.RETRY_AFTER_GOAL_PROBE,
            NavPurpose.RETRY_FROM_SIDE_ESCAPE,
            NavPurpose.RETRY_FROM_BREADCRUMB,
        ]:
            return

        current_pose = self._get_current_pose()
        if current_pose is None:
            return

        self.last_goal_retry_failure_pose = self._copy_pose(current_pose)
        self.last_goal_retry_failure_sec = self._now_sec()
        self.goal_corridor_clear_ticks = 0

    def _goal_retry_failure_cooldown_active(
        self,
        now_sec: float,
        current_pose: PoseStamped,
    ) -> bool:
        if self.last_goal_retry_failure_pose is None:
            return False

        if now_sec - self.last_goal_retry_failure_sec > self.goal_retry_failure_cooldown_sec:
            return False

        return (
            self.route_memory.distance_xy(
                self.last_goal_retry_failure_pose,
                current_pose,
            )
            <= self.goal_retry_failure_radius
        )

    def _current_odom_xy_if_fresh(self, now_sec: float) -> Optional[Tuple[float, float]]:
        if self.latest_odom is None or self.latest_odom_time_sec is None:
            return None

        if now_sec - self.latest_odom_time_sec > self.odom_stale_sec:
            return None

        position = self.latest_odom.pose.pose.position
        return position.x, position.y

    def _phase_elapsed(self, now_sec: float) -> float:
        if self.reverse_phase_start_sec is None:
            return 0.0
        return now_sec - self.reverse_phase_start_sec

    # ------------------------------------------------------------------
    # Active goal timeout/cancel
    # ------------------------------------------------------------------

    def _check_active_goal_timeout(self, now_sec: float):
        if self.active_goal_start_time_sec is None or self.cancel_in_progress:
            return

        if self.active_goal_purpose is None:
            return

        timeout = self._timeout_for_purpose(self.active_goal_purpose)
        if now_sec - self.active_goal_start_time_sec <= timeout:
            return

        self.get_logger().warn(
            f"Nav2 goal timeout. purpose={self.active_goal_purpose}, "
            f"timeout={timeout:.1f}s"
        )
        self._publish_status(f"NAV_TIMEOUT_{self.active_goal_purpose.upper()}")
        self._cancel_active_nav_goal("TIMEOUT")

    # ------------------------------------------------------------------
    # Phase 7C: Stall watchdog
    # ------------------------------------------------------------------

    _GOAL_TRACKING_STATES = frozenset({
        MissionState.NAVIGATING_TO_GOAL,
        MissionState.REPLAN_SAME_GOAL,
        MissionState.RETRY_GOAL_AFTER_REVERSE,
        MissionState.RETRY_GOAL_AFTER_GOAL_PROBE,
        MissionState.RETRY_GOAL_AFTER_CONTROLLER_PATIENCE,
        MissionState.RETRY_GOAL_FROM_SIDE_ESCAPE,
        MissionState.RETRY_GOAL_FROM_BREADCRUMB,
    })

    def _check_stall_watchdog(self, now_sec: float):
        if not self.stall_watchdog_enabled:
            return

        if self.cancel_in_progress:
            return

        if self.state not in self._GOAL_TRACKING_STATES:
            self._reset_stall_watchdog()
            return

        if self.active_goal_handle is None or self.original_goal is None:
            self._reset_stall_watchdog()
            return

        current_pose = self._get_current_pose()
        if current_pose is None:
            return

        goal_distance = self._distance_between(current_pose, self.original_goal)
        if goal_distance < self.stall_watchdog_min_goal_distance_m:
            self._reset_stall_watchdog()
            return

        if self.stall_reference_pose is None:
            self.stall_reference_pose = self._copy_pose(current_pose)
            self.stall_reference_time_sec = now_sec
            self.stall_grace_until_sec = (
                now_sec + self.stall_watchdog_grace_period_sec
            )
            return

        if now_sec < (self.stall_grace_until_sec or now_sec):
            return

        elapsed = now_sec - (self.stall_reference_time_sec or now_sec)
        if elapsed < self.stall_watchdog_time_window_sec:
            return

        displacement = self.route_memory.distance_xy(
            self.stall_reference_pose,
            current_pose,
        )

        if displacement >= self.stall_watchdog_min_displacement_m:
            self.stall_reference_pose = self._copy_pose(current_pose)
            self.stall_reference_time_sec = now_sec
            return

        self.get_logger().warn(
            f"Stall watchdog: robot moved only {displacement:.2f}m in "
            f"{elapsed:.1f}s while NAV2 goal active "
            f"(purpose={self.active_goal_purpose}, goal_distance={goal_distance:.2f}m). "
            f"Cancelling NAV2 and jumping to Phase 5."
        )
        self._publish_status("STALL_WATCHDOG_TRIGGERED")
        self.last_failure_reason_was_stall = True
        self._cancel_active_nav_goal("STALLED_BY_WATCHDOG")
        self._reset_stall_watchdog()

    def _reset_stall_watchdog(self):
        self.stall_reference_pose = None
        self.stall_reference_time_sec = None
        self.stall_grace_until_sec = None

    def _distance_between(
        self,
        a_pose: PoseStamped,
        b_pose: PoseStamped,
    ) -> float:
        dx = a_pose.pose.position.x - b_pose.pose.position.x
        dy = a_pose.pose.position.y - b_pose.pose.position.y
        return math.hypot(dx, dy)

    def _timeout_for_purpose(self, purpose: str) -> float:
        if purpose == NavPurpose.SIDE_ESCAPE:
            return self.side_escape_goal_timeout_sec
        if purpose == NavPurpose.BREADCRUMB:
            return self.breadcrumb_timeout_sec
        if purpose == NavPurpose.BASE:
            return self.return_to_base_timeout_sec
        return self.goal_timeout_sec

    def _cancel_active_nav_goal(self, reason: str):
        self.active_goal_cancel_reason = reason

        if self.active_goal_handle is None:
            return

        if self.cancel_in_progress:
            return

        self.cancel_in_progress = True
        future = self.active_goal_handle.cancel_goal_async()
        future.add_done_callback(self._on_cancel_done)

    def _on_cancel_done(self, future):
        try:
            future.result()
            self.get_logger().info("Active Nav2 goal cancel completed.")
        except Exception as exc:
            self.get_logger().warn(f"Active Nav2 goal cancel failed: {exc}")

    def _clear_active_goal(self):
        self.active_goal_handle = None
        self.active_goal_mission_id = None
        self.active_goal_sequence_id = None
        self.active_goal_purpose = None
        self.active_goal_pose = None
        self.active_goal_start_time_sec = None
        self.cancel_in_progress = False
        self.active_goal_cancel_reason = None

    def _is_current_goal(self, mission_id: int, goal_sequence_id: int) -> bool:
        return (
            mission_id == self.mission_id
            and goal_sequence_id == self.active_goal_sequence_id
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _mark_current_failed_zone(self):
        current_pose = self._get_current_pose()
        if current_pose is None:
            return

        self.last_failed_pose = self._copy_pose(current_pose)
        added = self.route_memory.mark_failed_zone(current_pose, self._now_sec())
        if added:
            self.get_logger().warn(
                f"Marked failed zone at "
                f"({current_pose.pose.position.x:.2f}, "
                f"{current_pose.pose.position.y:.2f})"
            )

    def _get_current_pose(self) -> Optional[PoseStamped]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.current_pose_timeout_sec),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().debug(f"Current pose unavailable: {exc}")
            return None

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _make_pose_from_xy_yaw(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def _publish_reverse_cmd(self):
        msg = Twist()
        msg.linear.x = self.reverse_speed
        self.cmd_vel_pub.publish(msg)

    def _publish_zero_cmd(self):
        self.cmd_vel_pub.publish(Twist())

    def _set_state(self, state: MissionState, status: str):
        if self.state != state:
            self.get_logger().info(f"Mission state: {self.state.value} -> {state.value}")
        self.state = state
        self._publish_status(status)

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _should_publish_markers(self, now_sec: float) -> bool:
        if self.marker_publish_period_sec <= 0.0:
            return True

        if now_sec - self.last_marker_publish_sec < self.marker_publish_period_sec:
            return False

        self.last_marker_publish_sec = now_sec
        return True

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _copy_pose(pose: PoseStamped) -> PoseStamped:
        copied = PoseStamped()
        copied.header.stamp.sec = pose.header.stamp.sec
        copied.header.stamp.nanosec = pose.header.stamp.nanosec
        copied.header.frame_id = pose.header.frame_id
        copied.pose.position.x = pose.pose.position.x
        copied.pose.position.y = pose.pose.position.y
        copied.pose.position.z = pose.pose.position.z
        copied.pose.orientation.x = pose.pose.orientation.x
        copied.pose.orientation.y = pose.pose.orientation.y
        copied.pose.orientation.z = pose.pose.orientation.z
        copied.pose.orientation.w = pose.pose.orientation.w
        return copied

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        return math.atan2(math.sin(a - b), math.cos(a - b))

    @staticmethod
    def _yaw_from_pose(pose: PoseStamped) -> float:
        orientation = pose.pose.orientation
        siny_cosp = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        return math.atan2(siny_cosp, cosy_cosp)

    def _pose_near_any(
        self,
        pose: PoseStamped,
        other_poses: Sequence[PoseStamped],
        radius: float,
    ) -> bool:
        for other_pose in other_poses:
            if other_pose is None:
                continue
            if self.route_memory.distance_xy(pose, other_pose) <= radius:
                return True
        return False

    @staticmethod
    def _distance_xy_tuple(a: Sequence[float], b: Sequence[float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])


def main(args=None):
    rclpy.init(args=args)
    node = GpMissionSupervisorV2()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_zero_cmd()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
