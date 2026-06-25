import math
from typing import Iterable, List

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


DEFAULT_POSE_COVARIANCE_DIAGONAL = [0.05, 0.05, 1000000.0, 1000000.0, 1000000.0, 0.1]
DEFAULT_TWIST_COVARIANCE_DIAGONAL = [0.02, 0.02, 1000000.0, 1000000.0, 1000000.0, 0.05]
MAX_DT = 1.0


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quaternion(yaw: float):
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def covariance_from_diagonal(diagonal: Iterable[float]) -> List[float]:
    covariance = [0.0] * 36
    values = list(diagonal)
    for index in range(6):
        covariance[index * 6 + index] = float(values[index])
    return covariance


class DifferentialOdomNode(Node):
    def __init__(self) -> None:
        super().__init__('differential_odom_node')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('pose_covariance_diagonal', DEFAULT_POSE_COVARIANCE_DIAGONAL)
        self.declare_parameter('twist_covariance_diagonal', DEFAULT_TWIST_COVARIANCE_DIAGONAL)

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.publish_rate = max(1.0, float(self.get_parameter('publish_rate').value))

        self.pose_covariance = covariance_from_diagonal(
            self.get_covariance_diagonal(
                'pose_covariance_diagonal',
                DEFAULT_POSE_COVARIANCE_DIAGONAL,
            )
        )
        self.twist_covariance = covariance_from_diagonal(
            self.get_covariance_diagonal(
                'twist_covariance_diagonal',
                DEFAULT_TWIST_COVARIANCE_DIAGONAL,
            )
        )

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last_velocity_time = None
        self.received_velocity = False

        self.tf_broadcaster = None
        if self.publish_tf:
            try:
                from tf2_ros import TransformBroadcaster

                self.tf_broadcaster = TransformBroadcaster(self)
            except ImportError as exc:
                self.publish_tf = False
                self.get_logger().error(f'publish_tf requested, but tf2_ros is unavailable: {exc}')

        self.odom_pub = self.create_publisher(Odometry, '/odom_raw', 10)
        self.vel_sub = self.create_subscription(Twist, '/vel_raw', self.vel_raw_callback, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_odom)

    def get_covariance_diagonal(self, parameter_name: str, default: List[float]) -> List[float]:
        values = list(self.get_parameter(parameter_name).value)
        if len(values) != 6:
            self.get_logger().warn(
                f'{parameter_name} must contain 6 values; using default diagonal'
            )
            return default
        return [float(value) for value in values]

    def vel_raw_callback(self, msg: Twist) -> None:
        now = self.get_clock().now()

        v = float(msg.linear.x)
        w = float(msg.angular.z)

        if self.last_velocity_time is not None:
            dt = (now - self.last_velocity_time).nanoseconds / 1e9
            if 0.0 < dt <= MAX_DT:
                self.integrate(v, w, dt)

        self.last_velocity_time = now
        self.v = v
        self.w = w
        self.received_velocity = True

    def integrate(self, v: float, w: float, dt: float) -> None:
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta = normalize_angle(self.theta + (w * dt))

    def publish_odom(self) -> None:
        if not self.received_velocity:
            return

        now = self.get_clock().now()
        output_v = self.v
        output_w = self.w
        if self.last_velocity_time is not None:
            velocity_age = (now - self.last_velocity_time).nanoseconds / 1e9
            if velocity_age < 0.0 or velocity_age > MAX_DT:
                output_v = 0.0
                output_w = 0.0

        stamp = now.to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(self.theta)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance = self.pose_covariance

        odom.twist.twist.linear.x = output_v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = output_w
        odom.twist.covariance = self.twist_covariance

        self.odom_pub.publish(odom)
        if self.publish_tf and self.tf_broadcaster is not None:
            self.publish_transform(stamp, qx, qy, qz, qw)

    def publish_transform(self, stamp, qx: float, qy: float, qz: float, qw: float) -> None:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DifferentialOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
