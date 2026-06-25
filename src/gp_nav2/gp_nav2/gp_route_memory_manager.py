#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class FailedZone:
    pose: PoseStamped
    created_time_sec: float
    radius: float


@dataclass
class RouteMemoryConfig:
    breadcrumb_distance: float = 0.25
    max_breadcrumbs: int = 40
    min_retreat_distance: float = 0.45
    preferred_retreat_distance: float = 0.70
    max_retreat_distance: float = 1.50

    failed_zone_radius: float = 0.35
    failed_zone_lifetime_sec: float = 180.0
    same_failure_distance: float = 0.35

    marker_frame: str = "map"


class GpRouteMemoryManager:
    """
    Route memory helper for the GP mission supervisor.

    This class stores:
      - mission breadcrumbs
      - temporary failed zones
      - retreat-point selection logic

    It does not call Nav2 directly.
    It only answers questions like:

      "Where did the robot come from?"
      "Which previous point is safe enough to retreat to?"
      "Is this pose inside a failed area?"
      "Should we avoid this same failed place again?"

    The mission supervisor remains the boss.
    """

    def __init__(self, config: RouteMemoryConfig):
        self.config = config

        self.breadcrumbs: List[PoseStamped] = []
        self.failed_zones: List[FailedZone] = []
        self._last_breadcrumb_marker_count = 0
        self._last_failed_zone_marker_count = 0

    # ---------------------------------------------------------------------
    # Mission lifecycle
    # ---------------------------------------------------------------------

    def reset(self):
        self.breadcrumbs.clear()
        self.failed_zones.clear()

    def reset_breadcrumbs(self):
        self.breadcrumbs.clear()

    def reset_failed_zones(self):
        self.failed_zones.clear()

    # ---------------------------------------------------------------------
    # Breadcrumb logic
    # ---------------------------------------------------------------------

    def add_breadcrumb_if_needed(self, pose: PoseStamped, force: bool = False) -> bool:
        """
        Add a breadcrumb if:
          - force is True, or
          - there are no breadcrumbs yet, or
          - robot moved enough distance from last breadcrumb

        Returns True if a breadcrumb was added.
        """

        if pose is None:
            return False

        if self.pose_in_failed_zone(pose):
            return False

        if force or len(self.breadcrumbs) == 0:
            self._append_breadcrumb(pose)
            return True

        last_pose = self.breadcrumbs[-1]
        distance = self.distance_xy(pose, last_pose)

        if distance >= self.config.breadcrumb_distance:
            self._append_breadcrumb(pose)
            return True

        return False

    def _append_breadcrumb(self, pose: PoseStamped):
        copied_pose = self.copy_pose(pose)

        self.breadcrumbs.append(copied_pose)

        while len(self.breadcrumbs) > self.config.max_breadcrumbs:
            self.breadcrumbs.pop(0)

    def get_breadcrumb_count(self) -> int:
        return len(self.breadcrumbs)

    def get_last_breadcrumb(self) -> Optional[PoseStamped]:
        if not self.breadcrumbs:
            return None
        return self.breadcrumbs[-1]

    # ---------------------------------------------------------------------
    # Failed zone logic
    # ---------------------------------------------------------------------

    def mark_failed_zone(self, pose: PoseStamped, now_sec: float) -> bool:
        """
        Add or update a failed zone around the robot.

        Returns:
          True  -> new failed zone was added
          False -> existing nearby failed zone was refreshed
        """

        if pose is None:
            return False

        for zone in self.failed_zones:
            if self.distance_xy(pose, zone.pose) <= self.config.same_failure_distance:
                zone.created_time_sec = now_sec
                zone.radius = self.config.failed_zone_radius
                return False

        self.failed_zones.append(
            FailedZone(
                pose=self.copy_pose(pose),
                created_time_sec=now_sec,
                radius=self.config.failed_zone_radius,
            )
        )

        return True

    def cleanup_expired_failed_zones(self, now_sec: float) -> int:
        """
        Remove old failed zones.

        Returns the number of removed zones.
        """

        before = len(self.failed_zones)

        self.failed_zones = [
            zone
            for zone in self.failed_zones
            if now_sec - zone.created_time_sec <= self.config.failed_zone_lifetime_sec
        ]

        return before - len(self.failed_zones)

    def pose_in_failed_zone(self, pose: PoseStamped) -> bool:
        if pose is None:
            return False

        for zone in self.failed_zones:
            if self.distance_xy(pose, zone.pose) <= zone.radius:
                return True

        return False

    def failed_zone_count(self) -> int:
        return len(self.failed_zones)

    # ---------------------------------------------------------------------
    # Retreat selection logic
    # ---------------------------------------------------------------------

    def select_retreat_pose(self, current_pose: PoseStamped) -> Optional[PoseStamped]:
        """
        Select a safe retreat pose from breadcrumb history.

        Priority:
          1. Prefer breadcrumbs near preferred_retreat_distance.
          2. Skip breadcrumbs too close to the current pose.
          3. Skip breadcrumbs inside failed zones.
          4. Skip breadcrumbs farther than max_retreat_distance.
          5. Bias toward the newer breadcrumb when scores are similar.

        This creates the U-turn behavior:
          "go back to a previous safe area, not just blind backup."
        """

        return self._select_retreat_pose(current_pose, blocked_poses=[])

    def select_older_retreat_pose(
        self,
        current_pose: PoseStamped,
        last_failed_retreat_pose: Optional[PoseStamped],
    ) -> Optional[PoseStamped]:
        """
        Select an older retreat pose if the most recent retreat failed.

        This prevents:
          retreat to same point -> fail -> retreat to same point -> fail
        """

        blocked = []
        if last_failed_retreat_pose is not None:
            blocked.append(last_failed_retreat_pose)

        return self._select_retreat_pose(current_pose, blocked_poses=blocked)

    def select_retreat_pose_avoiding(
        self,
        current_pose: PoseStamped,
        blocked_poses: Sequence[PoseStamped],
    ) -> Optional[PoseStamped]:
        return self._select_retreat_pose(current_pose, blocked_poses=blocked_poses)

    def _select_retreat_pose(
        self,
        current_pose: PoseStamped,
        blocked_poses: Sequence[PoseStamped],
    ) -> Optional[PoseStamped]:
        if current_pose is None or not self.breadcrumbs:
            return None

        best_candidate = None
        best_score = None

        for reverse_index, candidate in enumerate(reversed(self.breadcrumbs)):
            distance = self.distance_xy(current_pose, candidate)

            if distance < self.config.min_retreat_distance:
                continue

            if distance > self.config.max_retreat_distance:
                continue

            if self.pose_in_failed_zone(candidate):
                continue

            if self._is_near_any(candidate, blocked_poses):
                continue

            distance_error = abs(distance - self.config.preferred_retreat_distance)
            recency_penalty = reverse_index * 0.001
            score = distance_error + recency_penalty

            if best_score is None or score < best_score:
                best_candidate = candidate
                best_score = score

        if best_candidate is None:
            return None

        return self.copy_pose(best_candidate)

    def _is_near_any(
        self,
        pose: PoseStamped,
        other_poses: Sequence[PoseStamped],
    ) -> bool:
        for other_pose in other_poses:
            if other_pose is None:
                continue
            if self.distance_xy(pose, other_pose) <= self.config.same_failure_distance:
                return True
        return False

    # ---------------------------------------------------------------------
    # Route repetition / failure-position helpers
    # ---------------------------------------------------------------------

    def is_near_previous_failed_zone(self, pose: PoseStamped) -> bool:
        if pose is None:
            return False

        for zone in self.failed_zones:
            if self.distance_xy(pose, zone.pose) <= self.config.same_failure_distance:
                return True

        return False

    def nearest_failed_zone_distance(self, pose: PoseStamped) -> Optional[float]:
        if pose is None or not self.failed_zones:
            return None

        return min(self.distance_xy(pose, zone.pose) for zone in self.failed_zones)

    # ---------------------------------------------------------------------
    # RViz visualization helpers
    # ---------------------------------------------------------------------

    def make_marker_array(self, now_msg) -> MarkerArray:
        """
        Create RViz markers for debugging.

        Green-ish spheres: breadcrumbs.
        Red-ish cylinders: failed zones.

        The caller should publish this MarkerArray.
        """

        marker_array = MarkerArray()

        # Breadcrumb markers
        for index, pose in enumerate(self.breadcrumbs):
            marker = Marker()
            marker.header.frame_id = self.config.marker_frame
            marker.header.stamp = now_msg
            marker.ns = "gp_breadcrumbs"
            marker.id = index

            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose = self.copy_pose(pose).pose

            marker.scale.x = 0.06
            marker.scale.y = 0.06
            marker.scale.z = 0.06

            marker.color.r = 0.0
            marker.color.g = 0.8
            marker.color.b = 0.2
            marker.color.a = 0.8

            marker_array.markers.append(marker)

        # Failed zone markers
        for index, zone in enumerate(self.failed_zones):
            marker = Marker()
            marker.header.frame_id = self.config.marker_frame
            marker.header.stamp = now_msg
            marker.ns = "gp_failed_zones"
            marker.id = index

            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose = self.copy_pose(zone.pose).pose
            marker.pose.position.z = 0.02

            marker.scale.x = zone.radius * 2.0
            marker.scale.y = zone.radius * 2.0
            marker.scale.z = 0.04

            marker.color.r = 1.0
            marker.color.g = 0.1
            marker.color.b = 0.1
            marker.color.a = 0.35

            marker_array.markers.append(marker)

        self._append_delete_markers(
            marker_array,
            "gp_breadcrumbs",
            len(self.breadcrumbs),
            self._last_breadcrumb_marker_count,
            now_msg,
        )
        self._append_delete_markers(
            marker_array,
            "gp_failed_zones",
            len(self.failed_zones),
            self._last_failed_zone_marker_count,
            now_msg,
        )

        self._last_breadcrumb_marker_count = len(self.breadcrumbs)
        self._last_failed_zone_marker_count = len(self.failed_zones)

        return marker_array

    def _append_delete_markers(
        self,
        marker_array: MarkerArray,
        namespace: str,
        current_count: int,
        previous_count: int,
        now_msg,
    ):
        for delete_id in range(current_count, previous_count):
            marker = Marker()
            marker.header.frame_id = self.config.marker_frame
            marker.header.stamp = now_msg
            marker.ns = namespace
            marker.id = delete_id
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)

    # ---------------------------------------------------------------------
    # Utility functions
    # ---------------------------------------------------------------------

    @staticmethod
    def distance_xy(a: PoseStamped, b: PoseStamped) -> float:
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        return math.hypot(dx, dy)

    @staticmethod
    def copy_pose(pose: PoseStamped) -> PoseStamped:
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

    def summary(self) -> str:
        return (
            f"breadcrumbs={len(self.breadcrumbs)}, "
            f"failed_zones={len(self.failed_zones)}"
        )
