#!/usr/bin/env python3

from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class GpGoalPoseRelay(Node):
    """Plain /goal_pose to NavigateToPose bridge for unsupervised Nav2 runs."""

    def __init__(self):
        super().__init__("gp_plain_goal_relay")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("mission_goal_topic", "/mission_goal")
        self.declare_parameter("rviz_goal_topic", "/goal_pose")
        self.declare_parameter("nav_action_name", "/navigate_to_pose")
        self.declare_parameter("nav_behavior_tree", "")

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.mission_goal_topic = str(self.get_parameter("mission_goal_topic").value)
        self.rviz_goal_topic = str(self.get_parameter("rviz_goal_topic").value)
        self.nav_action_name = str(self.get_parameter("nav_action_name").value)
        self.nav_behavior_tree = str(self.get_parameter("nav_behavior_tree").value)

        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action_name)
        self.active_goal_handle = None
        self.pending_goal: Optional[PoseStamped] = None
        self.cancel_in_progress = False

        self.create_subscription(
            PoseStamped,
            self.rviz_goal_topic,
            lambda msg: self._on_goal(msg, "rviz_goal"),
            10,
        )
        if self.mission_goal_topic != self.rviz_goal_topic:
            self.create_subscription(
                PoseStamped,
                self.mission_goal_topic,
                lambda msg: self._on_goal(msg, "mission_goal"),
                10,
            )

        self.get_logger().info(
            f"Plain goal relay ready: {self.rviz_goal_topic}, "
            f"{self.mission_goal_topic} -> {self.nav_action_name}"
        )

    def _on_goal(self, msg: PoseStamped, source: str):
        goal = self._copy_pose(msg)
        if goal.header.frame_id == "":
            goal.header.frame_id = self.map_frame

        if goal.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f"Ignoring {source} in frame '{goal.header.frame_id}'. "
                f"Expected '{self.map_frame}'."
            )
            return

        self.pending_goal = goal
        if self.active_goal_handle is not None and not self.cancel_in_progress:
            self.cancel_in_progress = True
            cancel_future = self.active_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._on_cancel_done)
            return

        if not self.cancel_in_progress:
            self._send_pending_goal()

    def _on_cancel_done(self, future):
        try:
            future.result()
        except Exception as exc:
            self.get_logger().warn(f"Previous Nav2 goal cancel failed: {exc}")

        self.active_goal_handle = None
        self.cancel_in_progress = False
        self._send_pending_goal()

    def _send_pending_goal(self):
        if self.pending_goal is None:
            return

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"Nav2 action server unavailable: {self.nav_action_name}")
            return

        target_pose = self.pending_goal
        self.pending_goal = None

        goal = NavigateToPose.Goal()
        goal.pose = target_pose
        if self.nav_behavior_tree:
            goal.behavior_tree = self.nav_behavior_tree

        self.get_logger().info(
            f"Sending plain Nav2 goal: x={goal.pose.pose.position.x:.2f}, "
            f"y={goal.pose.pose.position.y:.2f}"
        )
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Nav2 goal send failed: {exc}")
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal was rejected.")
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)
        self.get_logger().info("Nav2 goal accepted.")

    def _on_result(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to read Nav2 result: {exc}")
            self.active_goal_handle = None
            return

        self.active_goal_handle = None
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Plain Nav2 goal succeeded.")
        else:
            self.get_logger().warn(f"Plain Nav2 goal finished with status={result.status}.")

        if self.pending_goal is not None:
            self._send_pending_goal()

    @staticmethod
    def _copy_pose(pose: PoseStamped) -> PoseStamped:
        copied = PoseStamped()
        copied.header.stamp = pose.header.stamp
        copied.header.frame_id = pose.header.frame_id
        copied.pose.position.x = pose.pose.position.x
        copied.pose.position.y = pose.pose.position.y
        copied.pose.position.z = pose.pose.position.z
        copied.pose.orientation.x = pose.pose.orientation.x
        copied.pose.orientation.y = pose.pose.orientation.y
        copied.pose.orientation.z = pose.pose.orientation.z
        copied.pose.orientation.w = pose.pose.orientation.w
        return copied


def main(args=None):
    rclpy.init(args=args)
    node = GpGoalPoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
