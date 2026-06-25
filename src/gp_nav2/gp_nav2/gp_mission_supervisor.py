#!/usr/bin/env python3

import math
from enum import Enum
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

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
    NAVIGATING_TO_GOAL = "NAVIGATING_TO_GOAL"
    RETRYING_GOAL = "RETRYING_GOAL"
    RETREATING_TO_SAFE_POINT = "RETREATING_TO_SAFE_POINT"
    RETURNING_TO_BASE = "RETURNING_TO_BASE"
    SUCCEEDED = "SUCCEEDED"
    RETURNED_TO_BASE = "RETURNED_TO_BASE"
    FAILED = "FAILED"


class GpMissionSupervisor(Node):
    """
    Mission-level supervisor above Nav2.

    It does not replace Nav2 controller/planner/costmaps.

    It controls Nav2 at mission level:

      - save base point
      - receive mission goal
      - send goal to Nav2
      - record safe breadcrumbs
      - detect failed route
      - retreat to previous safe point
      - retry goal
      - return to base if goal is unreachable
    """

    def __init__(self):
        super().__init__("gp_mission_supervisor")

        # Frames / topics
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")

        self.declare_parameter("mission_goal_topic", "/mission_goal")
        self.declare_parameter("mission_status_topic", "/mission_status")
        self.declare_parameter("mission_marker_topic", "/gp_mission_markers")

        self.declare_parameter("nav_action_name", "/navigate_to_pose")
        self.declare_parameter("nav_behavior_tree", "")

        # Route memory
        self.declare_parameter("breadcrumb_distance", 0.25)
        self.declare_parameter("max_breadcrumbs", 40)
        self.declare_parameter("min_retreat_distance", 0.45)

        self.declare_parameter("failed_zone_radius", 0.35)
        self.declare_parameter("failed_zone_lifetime_sec", 120.0)
        self.declare_parameter("same_failure_distance", 0.35)

        # Attempt limits
        self.declare_parameter("max_direct_goal_attempts", 2)
        self.declare_parameter("max_retreat_attempts", 2)
        self.declare_parameter("max_total_mission_attempts", 5)

        # Timeouts
        self.declare_parameter("goal_timeout_sec", 120.0)
        self.declare_parameter("retreat_timeout_sec", 45.0)
        self.declare_parameter("return_to_base_timeout_sec", 120.0)

        # Behavior
        self.declare_parameter("allow_preemption", True)
        self.declare_parameter("record_breadcrumbs_during_retreat", False)
        self.declare_parameter("publish_debug_markers", True)

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.mission_goal_topic = self.get_parameter("mission_goal_topic").value
        self.mission_status_topic = self.get_parameter("mission_status_topic").value
        self.mission_marker_topic = self.get_parameter("mission_marker_topic").value

        self.nav_action_name = self.get_parameter("nav_action_name").value
        self.nav_behavior_tree = self.get_parameter("nav_behavior_tree").value

        self.max_direct_goal_attempts = int(
            self.get_parameter("max_direct_goal_attempts").value
        )
        self.max_retreat_attempts = int(
            self.get_parameter("max_retreat_attempts").value
        )
        self.max_total_mission_attempts = int(
            self.get_parameter("max_total_mission_attempts").value
        )

        self.goal_timeout_sec = float(self.get_parameter("goal_timeout_sec").value)
        self.retreat_timeout_sec = float(self.get_parameter("retreat_timeout_sec").value)
        self.return_to_base_timeout_sec = float(
            self.get_parameter("return_to_base_timeout_sec").value
        )

        self.allow_preemption = bool(self.get_parameter("allow_preemption").value)
        self.record_breadcrumbs_during_retreat = bool(
            self.get_parameter("record_breadcrumbs_during_retreat").value
        )
        self.publish_debug_markers = bool(
            self.get_parameter("publish_debug_markers").value
        )

        # Route memory manager
        memory_config = RouteMemoryConfig(
            breadcrumb_distance=float(
                self.get_parameter("breadcrumb_distance").value
            ),
            max_breadcrumbs=int(
                self.get_parameter("max_breadcrumbs").value
            ),
            min_retreat_distance=float(
                self.get_parameter("min_retreat_distance").value
            ),
            failed_zone_radius=float(
                self.get_parameter("failed_zone_radius").value
            ),
            failed_zone_lifetime_sec=float(
                self.get_parameter("failed_zone_lifetime_sec").value
            ),
            same_failure_distance=float(
                self.get_parameter("same_failure_distance").value
            ),
            marker_frame=self.map_frame,
        )

        self.route_memory = GpRouteMemoryManager(memory_config)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action_name)

        # ROS I/O
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.mission_goal_topic,
            self._on_new_mission_goal,
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            self.mission_status_topic,
            10,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.mission_marker_topic,
            10,
        )

        # Mission state
        self.state = MissionState.IDLE

        self.base_pose: Optional[PoseStamped] = None
        self.goal_pose: Optional[PoseStamped] = None

        self.direct_goal_attempts = 0
        self.retreat_attempts = 0
        self.total_mission_attempts = 0

        self.active_goal_handle = None
        self.active_goal_purpose: Optional[str] = None
        self.active_goal_pose: Optional[PoseStamped] = None
        self.active_goal_start_time_sec: Optional[float] = None

        self.cancel_in_progress = False
        self.pending_preempt_goal: Optional[PoseStamped] = None
        self.last_failed_retreat_pose: Optional[PoseStamped] = None

        self.timer = self.create_timer(0.2, self._on_timer)

        self._publish_status("IDLE")

        self.get_logger().info("gp_mission_supervisor is ready.")
        self.get_logger().info(f"Mission goal topic: {self.mission_goal_topic}")
        self.get_logger().info(f"Nav2 action: {self.nav_action_name}")

    # ------------------------------------------------------------------
    # Mission input
    # ------------------------------------------------------------------

    def _on_new_mission_goal(self, goal_msg: PoseStamped):
        if goal_msg.header.frame_id == "":
            goal_msg.header.frame_id = self.map_frame

        if goal_msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f"Goal frame is '{goal_msg.header.frame_id}', "
                f"but expected '{self.map_frame}'. "
                "Publish mission goals in map frame."
            )
            return

        # If idle/succeeded/failed, start immediately.
        if self.state in [
            MissionState.IDLE,
            MissionState.SUCCEEDED,
            MissionState.RETURNED_TO_BASE,
            MissionState.FAILED,
        ]:
            self._start_new_mission(goal_msg)
            return

        # If any mission/recovery/base-return is active, a new RViz goal should override it.
        if not self.allow_preemption:
            self.get_logger().warn("Mission already running. New goal ignored.")
            self._publish_status("MISSION_ALREADY_RUNNING_GOAL_IGNORED")
            return

        self.get_logger().warn("New RViz mission goal received. Preempting current mission/recovery.")
        self._publish_status("PREEMPTING_CURRENT_MISSION")

        self.pending_preempt_goal = goal_msg

        # If Nav2 currently has an active goal, cancel it first.
        if self.active_goal_handle is not None:
            self._cancel_active_nav_goal()
            return

        # If we are between goals/recoveries and no active handle exists,
        # start the new mission immediately instead of leaving it pending forever.
        new_goal = self.pending_preempt_goal
        self.pending_preempt_goal = None
        self._start_new_mission(new_goal)

    def _start_new_mission(self, goal_pose: PoseStamped):
        current_pose = self._get_current_pose()

        if current_pose is None:
            self.get_logger().error("Cannot start mission: current robot pose unavailable.")
            self._publish_status("FAILED_NO_CURRENT_POSE")
            return

        self.base_pose = current_pose
        self.goal_pose = goal_pose

        self.route_memory.reset()

        self.direct_goal_attempts = 0
        self.retreat_attempts = 0
        self.total_mission_attempts = 0
        self.last_failed_retreat_pose = None

        self.route_memory.add_breadcrumb_if_needed(current_pose, force=True)

        self.state = MissionState.NAVIGATING_TO_GOAL

        self.get_logger().info(
            "New mission started. "
            f"Base=({self.base_pose.pose.position.x:.2f}, "
            f"{self.base_pose.pose.position.y:.2f}), "
            f"Goal=({self.goal_pose.pose.position.x:.2f}, "
            f"{self.goal_pose.pose.position.y:.2f})"
        )

        self._publish_status("MISSION_STARTED")
        self._send_goal_to_nav2(self.goal_pose, purpose="goal")

    # ------------------------------------------------------------------
    # Timer
    # ------------------------------------------------------------------

    def _on_timer(self):
        now_sec = self._now_sec()

        removed = self.route_memory.cleanup_expired_failed_zones(now_sec)
        if removed > 0:
            self.get_logger().info(f"Expired {removed} old failed zone(s).")

        if self.state in [
            MissionState.NAVIGATING_TO_GOAL,
            MissionState.RETRYING_GOAL,
        ]:
            self._record_breadcrumb_if_valid()

        if (
            self.record_breadcrumbs_during_retreat
            and self.state == MissionState.RETREATING_TO_SAFE_POINT
        ):
            self._record_breadcrumb_if_valid()

        self._check_active_goal_timeout()

        if self.publish_debug_markers:
            self._publish_debug_markers()

    # ------------------------------------------------------------------
    # Nav2 action
    # ------------------------------------------------------------------

    def _send_goal_to_nav2(self, target_pose: PoseStamped, purpose: str):
        if self.total_mission_attempts >= self.max_total_mission_attempts and purpose != "base":
            self.get_logger().warn("Maximum mission attempts reached. Switching to return-to-base.")
            self._go_return_to_base()
            return

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                f"Nav2 action server not available: {self.nav_action_name}"
            )
            self._publish_status("FAILED_NAV2_ACTION_SERVER_NOT_AVAILABLE")
            self._go_return_to_base()
            return

        goal = NavigateToPose.Goal()
        goal.pose = target_pose

        if self.nav_behavior_tree:
            goal.behavior_tree = self.nav_behavior_tree

        if purpose != "base":
            self.total_mission_attempts += 1

        if purpose in ["goal", "retry_goal"]:
            self.direct_goal_attempts += 1

        if purpose == "retreat":
            self.retreat_attempts += 1

        self.active_goal_purpose = purpose
        self.active_goal_pose = self._copy_pose(target_pose)
        self.active_goal_start_time_sec = self._now_sec()

        self.get_logger().info(
            f"Sending Nav2 goal. purpose={purpose}, "
            f"total_attempts={self.total_mission_attempts}, "
            f"direct_goal_attempts={self.direct_goal_attempts}, "
            f"retreat_attempts={self.retreat_attempts}, "
            f"memory=({self.route_memory.summary()})"
        )

        self._publish_status(f"SENDING_NAV2_GOAL_{purpose.upper()}")

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_nav_goal_response)

    def _on_nav_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Nav2 goal send failed: {exc}")
            self._handle_nav_result(False, "SEND_FAILED")
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal was rejected.")
            self._handle_nav_result(False, "REJECTED")
            return

        self.active_goal_handle = goal_handle
        self.cancel_in_progress = False

        self.get_logger().info(f"Nav2 goal accepted. purpose={self.active_goal_purpose}")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to get Nav2 result: {exc}")
            self._handle_nav_result(False, "RESULT_ERROR")
            return

        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._handle_nav_result(True, "SUCCEEDED")
        elif status == GoalStatus.STATUS_CANCELED:
            self._handle_nav_result(False, "CANCELED")
        elif status == GoalStatus.STATUS_ABORTED:
            self._handle_nav_result(False, "ABORTED")
        else:
            self._handle_nav_result(False, f"STATUS_{status}")

    def _handle_nav_result(self, success: bool, status_text: str):
        purpose = self.active_goal_purpose

        self.active_goal_handle = None
        self.active_goal_purpose = None
        self.active_goal_start_time_sec = None
        self.cancel_in_progress = False

        if self.pending_preempt_goal is not None:
            new_goal = self.pending_preempt_goal
            self.pending_preempt_goal = None
            self.get_logger().warn("Starting preempted mission goal.")
            self._start_new_mission(new_goal)
            return

        if purpose is None:
            self.get_logger().warn("Received Nav2 result with unknown purpose.")
            return

        self.get_logger().info(
            f"Nav2 result. purpose={purpose}, success={success}, status={status_text}"
        )

        if success:
            self._handle_successful_nav_result(purpose)
        else:
            self._handle_failed_nav_result(purpose, status_text)

    def _handle_successful_nav_result(self, purpose: str):
        if purpose in ["goal", "retry_goal"]:
            self.state = MissionState.SUCCEEDED
            self.get_logger().info("Mission succeeded: goal reached.")
            self._publish_status("GOAL_REACHED")
            return

        if purpose == "retreat":
            self.get_logger().info("Retreat succeeded. Trying original goal again.")

            self.direct_goal_attempts = 0
            self.last_failed_retreat_pose = None

            self.state = MissionState.RETRYING_GOAL
            self._publish_status("RETREAT_SUCCEEDED_RETRYING_GOAL")

            self._send_goal_to_nav2(self.goal_pose, purpose="retry_goal")
            return

        if purpose == "base":
            self.state = MissionState.RETURNED_TO_BASE
            self.get_logger().warn("Goal unreachable, but robot returned to base.")
            self._publish_status("GOAL_UNREACHABLE_RETURNED_TO_BASE")
            return

    def _handle_failed_nav_result(self, purpose: str, status_text: str):
        current_pose = self._get_current_pose()
        # Mark failed zones for goal/retry/retreat failures.
        # Do not mark failed zones while returning to base, because base return
        # is the final safety behavior and should not keep poisoning the route memory.
        if current_pose is not None and purpose != "base":
            added = self.route_memory.mark_failed_zone(current_pose, self._now_sec())
            if added:
                self.get_logger().warn(
                    f"Marked new failed zone at "
                    f"({current_pose.pose.position.x:.2f}, "
                    f"{current_pose.pose.position.y:.2f})"
                )
            else:
                self.get_logger().warn("Refreshed nearby existing failed zone.")

        if purpose == "retreat" and self.active_goal_pose is not None:
            self.last_failed_retreat_pose = self._copy_pose(self.active_goal_pose)

        if purpose in ["goal", "retry_goal"]:
            self.get_logger().warn(f"Goal attempt failed: {status_text}")
            self._recover_from_goal_failure()
            return

        if purpose == "retreat":
            self.get_logger().warn(f"Retreat attempt failed: {status_text}")
            self._recover_from_retreat_failure()
            return

        if purpose == "base":
            self.state = MissionState.FAILED
            self.active_goal_handle = None
            self.active_goal_purpose = None
            self.active_goal_pose = None
            self.active_goal_start_time_sec = None
            self.cancel_in_progress = False

            self.get_logger().error(
                "Return-to-base failed. Manual assistance or a new RViz goal is required."
            )
            self._publish_status("FAILED_RETURN_TO_BASE_FAILED_NEW_GOAL_ALLOWED")
            return

    # ------------------------------------------------------------------
    # Recovery decisions
    # ------------------------------------------------------------------

    def _recover_from_goal_failure(self):
        if self.total_mission_attempts >= self.max_total_mission_attempts:
            self.get_logger().warn("Total mission attempt limit reached.")
            self._go_return_to_base()
            return

        if self.direct_goal_attempts < self.max_direct_goal_attempts:
            self.state = MissionState.RETRYING_GOAL
            self.get_logger().warn("Retrying goal to allow a different global route.")
            self._publish_status("GOAL_FAILED_RETRYING_ALTERNATIVE_ROUTE")
            self._send_goal_to_nav2(self.goal_pose, purpose="retry_goal")
            return

        current_pose = self._get_current_pose()
        retreat_pose = self.route_memory.select_retreat_pose(current_pose)

        if retreat_pose is not None and self.retreat_attempts < self.max_retreat_attempts:
            self.state = MissionState.RETREATING_TO_SAFE_POINT
            self.get_logger().warn("Repeated goal failure. Retreating to safe breadcrumb.")
            self._publish_status("RETREATING_TO_SAFE_BREADCRUMB")
            self._send_goal_to_nav2(retreat_pose, purpose="retreat")
            return

        self.get_logger().warn("No valid retreat option. Returning to base.")
        self._go_return_to_base()

    def _recover_from_retreat_failure(self):
        if self.total_mission_attempts >= self.max_total_mission_attempts:
            self._go_return_to_base()
            return

        current_pose = self._get_current_pose()
        retreat_pose = self.route_memory.select_older_retreat_pose(
            current_pose,
            self.last_failed_retreat_pose,
        )

        if retreat_pose is not None and self.retreat_attempts < self.max_retreat_attempts:
            self.state = MissionState.RETREATING_TO_SAFE_POINT
            self.get_logger().warn("Trying an older safe breadcrumb for retreat.")
            self._publish_status("TRYING_OLDER_RETREAT_BREADCRUMB")
            self._send_goal_to_nav2(retreat_pose, purpose="retreat")
            return

        self.get_logger().warn("Retreat failed. Returning to base.")
        self._go_return_to_base()

    def _go_return_to_base(self):
        if self.base_pose is None:
            self.state = MissionState.FAILED
            self.get_logger().error("Cannot return to base: no saved base pose.")
            self._publish_status("FAILED_NO_BASE_POSE")
            return

        # Prevent repeated re-entry while a base return goal is already active.
        if (
            self.state == MissionState.RETURNING_TO_BASE
            and self.active_goal_purpose == "base"
            and self.active_goal_handle is not None
        ):
            self.get_logger().warn("Already returning to base. Ignoring duplicate return request.")
            return

        self.state = MissionState.RETURNING_TO_BASE
        self._publish_status("RETURNING_TO_BASE")

        self.get_logger().warn(
            f"Returning to base at "
            f"({self.base_pose.pose.position.x:.2f}, "
            f"{self.base_pose.pose.position.y:.2f})"
        )

        self._send_goal_to_nav2(self.base_pose, purpose="base")

    # ------------------------------------------------------------------
    # Breadcrumbs / markers
    # ------------------------------------------------------------------

    def _record_breadcrumb_if_valid(self):
        pose = self._get_current_pose()
        if pose is None:
            return

        added = self.route_memory.add_breadcrumb_if_needed(pose)

        if added:
            self.get_logger().debug(
                f"Breadcrumb saved. {self.route_memory.summary()}"
            )

    def _publish_debug_markers(self):
        marker_array = self.route_memory.make_marker_array(
            self.get_clock().now().to_msg()
        )
        self.marker_pub.publish(marker_array)

    # ------------------------------------------------------------------
    # Timeout / cancellation
    # ------------------------------------------------------------------

    def _check_active_goal_timeout(self):
        if self.active_goal_handle is None:
            return

        if self.active_goal_start_time_sec is None:
            return

        if self.cancel_in_progress:
            return

        purpose = self.active_goal_purpose

        if purpose in ["goal", "retry_goal"]:
            timeout = self.goal_timeout_sec
        elif purpose == "retreat":
            timeout = self.retreat_timeout_sec
        elif purpose == "base":
            timeout = self.return_to_base_timeout_sec
        else:
            timeout = self.goal_timeout_sec

        elapsed = self._now_sec() - self.active_goal_start_time_sec

        if elapsed > timeout:
            self.get_logger().warn(
                f"Nav2 goal timeout. purpose={purpose}, "
                f"elapsed={elapsed:.1f}s, timeout={timeout:.1f}s"
            )
            self._publish_status(f"NAV_GOAL_TIMEOUT_{purpose}")
            self._cancel_active_nav_goal()

    def _cancel_active_nav_goal(self):
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
            self.get_logger().warn("Active Nav2 goal cancel completed.")
        except Exception as exc:
            self.get_logger().error(f"Failed to cancel Nav2 goal: {exc}")

    # ------------------------------------------------------------------
    # TF / utilities
    # ------------------------------------------------------------------

    def _get_current_pose(self) -> Optional[PoseStamped]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
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

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _copy_pose(pose: PoseStamped) -> PoseStamped:
        copied = PoseStamped()
        copied.header = pose.header
        copied.pose = pose.pose
        return copied

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = GpMissionSupervisor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
