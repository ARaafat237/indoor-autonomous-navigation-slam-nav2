import re
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

try:
    import serial
    from serial import SerialException
except ImportError:  # pragma: no cover
    serial = None
    SerialException = Exception


MSPD_RE = re.compile(
    rb'\$MSPD:([-+]?\d+(?:\.\d+)?),([-+]?\d+(?:\.\d+)?),'
    rb'([-+]?\d+(?:\.\d+)?),([-+]?\d+(?:\.\d+)?)#'
)


class MotorSerialDriver(Node):
    def __init__(self) -> None:
        super().__init__('motor_serial_driver')

        # Serial / robot parameters
        self.declare_parameter('port', '/dev/my_motor')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_base', 0.12)

        # Legacy sign parameters kept for backward compatibility.
        # If the new command/feedback signs below are left at 0.0,
        # the driver uses these legacy values.
        self.declare_parameter('left_sign', 1.0)
        self.declare_parameter('right_sign', 1.0)

        # Separate signs are safer because motor command direction and
        # encoder feedback direction are not always corrected the same way.
        # Use 0.0 to inherit from left_sign/right_sign.
        self.declare_parameter('left_cmd_sign', 0.0)
        self.declare_parameter('right_cmd_sign', 0.0)
        self.declare_parameter('left_feedback_sign', 0.0)
        self.declare_parameter('right_feedback_sign', 0.0)
        self.declare_parameter('swap_command_wheels', False)
        self.declare_parameter('swap_feedback_wheels', False)

        # Feedback scale keeps the current assumption that board speed is mm/s,
        # but lets us calibrate later without editing code.
        self.declare_parameter('feedback_scale', 1.0)
        self.declare_parameter('left_feedback_scale', 1.0)
        self.declare_parameter('right_feedback_scale', 1.0)

        # Command-side wheel calibration.
        # These are applied before motor minimum compensation and before command
        # signs, so they correct physical wheel response without changing topic
        # direction conventions.
        self.declare_parameter('left_forward_cmd_scale', 1.0)
        self.declare_parameter('right_forward_cmd_scale', 1.0)
        self.declare_parameter('left_reverse_cmd_scale', 1.0)
        self.declare_parameter('right_reverse_cmd_scale', 1.0)
        self.declare_parameter('forward_turn_outer_wheel_scale', 1.0)
        self.declare_parameter('forward_turn_inner_wheel_scale', 1.0)
        self.declare_parameter('forward_left_turn_outer_wheel_scale', 0.0)
        self.declare_parameter('forward_left_turn_inner_wheel_scale', 0.0)
        self.declare_parameter('forward_right_turn_outer_wheel_scale', 0.0)
        self.declare_parameter('forward_right_turn_inner_wheel_scale', 0.0)
        self.declare_parameter('forward_turn_preserve_linear_speed', True)
        self.declare_parameter('pure_spin_left_cmd_scale', 1.0)
        self.declare_parameter('pure_spin_right_cmd_scale', 1.0)
        self.declare_parameter('pure_spin_left_min_wheel_speed_mm_s', 0.0)
        self.declare_parameter('pure_spin_right_min_wheel_speed_mm_s', 0.0)

        # Straight-driving feedback cleanup.
        # Your tests showed +/-0.099 and +/-0.199 rad/s fake yaw steps at
        # straight command. This zeros those small encoder-quantization yaw
        # steps only when the commanded angular velocity is approximately zero.
        self.declare_parameter('straight_cmd_w_epsilon', 0.02)
        self.declare_parameter('straight_feedback_w_deadband', 0.22)
        self.declare_parameter('straight_feedback_force_zero', False)
        self.declare_parameter('straight_feedback_force_zero_min_v_m_s', 0.02)

        # Optional low-pass filtering for /vel_raw.
        # 1.0 = no smoothing. 0.35 is useful if /vel_raw still looks jumpy.
        self.declare_parameter('feedback_smoothing_alpha', 1.0)

        # Absolute wheel command safety limit
        self.declare_parameter('max_speed_mm_s', 300.0)

        # Safer real-robot velocity limits before converting to wheels
        self.declare_parameter('max_linear_m_s', 0.22)
        self.declare_parameter('max_angular_rad_s', 0.65)

        # Hardware-aware ramping.
        # These protect the caster/universal wheels from sudden yaw commands.
        self.declare_parameter('max_linear_accel_m_s2', 0.30)
        self.declare_parameter('max_angular_accel_rad_s2', 0.80)

        # Pure spin protection.
        # If linear velocity is almost zero, angular command is scaled down.
        self.declare_parameter('pure_spin_v_threshold', 0.03)
        self.declare_parameter('pure_spin_angular_scale', 0.65)

        # Motor deadband handling.
        # Very tiny commands often do not move the motors but still confuse Nav2.
        self.declare_parameter('command_deadband_mm_s', 8.0)
        self.declare_parameter('min_effective_speed_mm_s', 22.0)
        self.declare_parameter('min_turn_wheel_speed_mm_s', 0.0)
        self.declare_parameter('min_spin_wheel_speed_mm_s', 0.0)
        self.declare_parameter('turn_min_w_threshold_rad_s', 0.08)
        self.declare_parameter('left_forward_min_speed_mm_s', 0.0)
        self.declare_parameter('left_reverse_min_speed_mm_s', 0.0)
        self.declare_parameter('right_forward_min_speed_mm_s', 0.0)
        self.declare_parameter('right_reverse_min_speed_mm_s', 0.0)

        # Timeout and rates
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('command_rate', 20.0)
        self.declare_parameter('feedback_rate', 10.0)
        self.declare_parameter('speed_command_epsilon_mm_s', 1.0)
        self.declare_parameter('speed_command_keepalive_sec', 0.10)

        # Serial recovery
        self.declare_parameter('reconnect_on_error', True)
        self.declare_parameter('reconnect_interval', 1.0)

        # Yahboom upload command behavior.
        # Keeping request_feedback_every_cycle=True is the safest default because
        # it matches your current working behavior. If the board streams after one
        # upload command, you can later set this False.
        self.declare_parameter('request_feedback_on_connect', True)
        self.declare_parameter('request_feedback_every_cycle', True)

        # Debug
        self.declare_parameter('debug_commands', False)
        self.declare_parameter('debug_feedback', False)
        self.declare_parameter('debug_feedback_rate', 2.0)

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)

        self.left_sign = float(self.get_parameter('left_sign').value)
        self.right_sign = float(self.get_parameter('right_sign').value)

        left_cmd_sign = float(self.get_parameter('left_cmd_sign').value)
        right_cmd_sign = float(self.get_parameter('right_cmd_sign').value)
        left_feedback_sign = float(self.get_parameter('left_feedback_sign').value)
        right_feedback_sign = float(self.get_parameter('right_feedback_sign').value)

        self.left_cmd_sign = self.left_sign if left_cmd_sign == 0.0 else left_cmd_sign
        self.right_cmd_sign = self.right_sign if right_cmd_sign == 0.0 else right_cmd_sign
        self.left_feedback_sign = self.left_sign if left_feedback_sign == 0.0 else left_feedback_sign
        self.right_feedback_sign = self.right_sign if right_feedback_sign == 0.0 else right_feedback_sign
        self.swap_command_wheels = bool(self.get_parameter('swap_command_wheels').value)
        self.swap_feedback_wheels = bool(self.get_parameter('swap_feedback_wheels').value)

        self.feedback_scale = float(self.get_parameter('feedback_scale').value)
        self.left_feedback_scale = float(self.get_parameter('left_feedback_scale').value)
        self.right_feedback_scale = float(self.get_parameter('right_feedback_scale').value)

        self.left_forward_cmd_scale = self.positive_scale(
            float(self.get_parameter('left_forward_cmd_scale').value)
        )
        self.right_forward_cmd_scale = self.positive_scale(
            float(self.get_parameter('right_forward_cmd_scale').value)
        )
        self.left_reverse_cmd_scale = self.positive_scale(
            float(self.get_parameter('left_reverse_cmd_scale').value)
        )
        self.right_reverse_cmd_scale = self.positive_scale(
            float(self.get_parameter('right_reverse_cmd_scale').value)
        )
        self.forward_turn_outer_wheel_scale = self.positive_scale(
            float(self.get_parameter('forward_turn_outer_wheel_scale').value)
        )
        self.forward_turn_inner_wheel_scale = self.positive_scale(
            float(self.get_parameter('forward_turn_inner_wheel_scale').value)
        )
        self.forward_left_turn_outer_wheel_scale = self.optional_positive_scale(
            float(self.get_parameter('forward_left_turn_outer_wheel_scale').value),
            self.forward_turn_outer_wheel_scale,
        )
        self.forward_left_turn_inner_wheel_scale = self.optional_positive_scale(
            float(self.get_parameter('forward_left_turn_inner_wheel_scale').value),
            self.forward_turn_inner_wheel_scale,
        )
        self.forward_right_turn_outer_wheel_scale = self.optional_positive_scale(
            float(self.get_parameter('forward_right_turn_outer_wheel_scale').value),
            self.forward_turn_outer_wheel_scale,
        )
        self.forward_right_turn_inner_wheel_scale = self.optional_positive_scale(
            float(self.get_parameter('forward_right_turn_inner_wheel_scale').value),
            self.forward_turn_inner_wheel_scale,
        )
        self.forward_turn_preserve_linear_speed = bool(
            self.get_parameter('forward_turn_preserve_linear_speed').value
        )
        self.pure_spin_left_cmd_scale = self.positive_scale(
            float(self.get_parameter('pure_spin_left_cmd_scale').value)
        )
        self.pure_spin_right_cmd_scale = self.positive_scale(
            float(self.get_parameter('pure_spin_right_cmd_scale').value)
        )
        self.pure_spin_left_min_wheel_speed_mm_s = abs(
            float(self.get_parameter('pure_spin_left_min_wheel_speed_mm_s').value)
        )
        self.pure_spin_right_min_wheel_speed_mm_s = abs(
            float(self.get_parameter('pure_spin_right_min_wheel_speed_mm_s').value)
        )

        self.straight_cmd_w_epsilon = abs(float(self.get_parameter('straight_cmd_w_epsilon').value))
        self.straight_feedback_w_deadband = abs(float(self.get_parameter('straight_feedback_w_deadband').value))
        self.straight_feedback_force_zero = bool(
            self.get_parameter('straight_feedback_force_zero').value
        )
        self.straight_feedback_force_zero_min_v_m_s = abs(
            float(self.get_parameter('straight_feedback_force_zero_min_v_m_s').value)
        )
        self.feedback_smoothing_alpha = self.limit(
            float(self.get_parameter('feedback_smoothing_alpha').value), 0.0, 1.0
        )

        self.max_speed_mm_s = abs(float(self.get_parameter('max_speed_mm_s').value))
        self.max_linear_m_s = abs(float(self.get_parameter('max_linear_m_s').value))
        self.max_angular_rad_s = abs(float(self.get_parameter('max_angular_rad_s').value))

        self.max_linear_accel_m_s2 = abs(float(self.get_parameter('max_linear_accel_m_s2').value))
        self.max_angular_accel_rad_s2 = abs(float(self.get_parameter('max_angular_accel_rad_s2').value))

        self.pure_spin_v_threshold = abs(float(self.get_parameter('pure_spin_v_threshold').value))
        self.pure_spin_angular_scale = float(self.get_parameter('pure_spin_angular_scale').value)

        self.command_deadband_mm_s = abs(float(self.get_parameter('command_deadband_mm_s').value))
        self.min_effective_speed_mm_s = abs(float(self.get_parameter('min_effective_speed_mm_s').value))
        self.min_turn_wheel_speed_mm_s = abs(
            float(self.get_parameter('min_turn_wheel_speed_mm_s').value)
        )
        self.min_spin_wheel_speed_mm_s = abs(
            float(self.get_parameter('min_spin_wheel_speed_mm_s').value)
        )
        self.turn_min_w_threshold_rad_s = abs(
            float(self.get_parameter('turn_min_w_threshold_rad_s').value)
        )
        self.left_forward_min_speed_mm_s = abs(
            float(self.get_parameter('left_forward_min_speed_mm_s').value)
        )
        self.left_reverse_min_speed_mm_s = abs(
            float(self.get_parameter('left_reverse_min_speed_mm_s').value)
        )
        self.right_forward_min_speed_mm_s = abs(
            float(self.get_parameter('right_forward_min_speed_mm_s').value)
        )
        self.right_reverse_min_speed_mm_s = abs(
            float(self.get_parameter('right_reverse_min_speed_mm_s').value)
        )

        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.command_rate = max(1.0, float(self.get_parameter('command_rate').value))
        self.feedback_rate = max(1.0, float(self.get_parameter('feedback_rate').value))
        self.speed_command_epsilon_mm_s = abs(
            float(self.get_parameter('speed_command_epsilon_mm_s').value)
        )
        self.speed_command_keepalive_sec = max(
            0.02,
            float(self.get_parameter('speed_command_keepalive_sec').value),
        )

        self.reconnect_on_error = bool(self.get_parameter('reconnect_on_error').value)
        self.reconnect_interval = float(self.get_parameter('reconnect_interval').value)

        self.request_feedback_on_connect = bool(self.get_parameter('request_feedback_on_connect').value)
        self.request_feedback_every_cycle = bool(self.get_parameter('request_feedback_every_cycle').value)

        self.debug_commands = bool(self.get_parameter('debug_commands').value)
        self.debug_feedback = bool(self.get_parameter('debug_feedback').value)
        self.debug_feedback_rate = max(0.1, float(self.get_parameter('debug_feedback_rate').value))

        if self.wheel_base <= 0.0:
            self.get_logger().warn('wheel_base must be positive; using 0.12 m')
            self.wheel_base = 0.12

        self.serial_port = None
        self.rx_buffer = b''

        now = self.get_clock().now()
        self.last_cmd_time = now
        self.last_ramp_time = now
        self.last_feedback_time = now
        self.last_reconnect_attempt = now
        self.last_feedback_debug_log = now
        self.last_speed_command_time = now
        self.last_sent_m2: Optional[int] = None
        self.last_sent_m4: Optional[int] = None

        self.timed_out = False

        # Target command from /cmd_vel
        self.target_v = 0.0
        self.target_w = 0.0

        # Current ramped command actually sent to motor board
        self.current_v = 0.0
        self.current_w = 0.0

        # Optional filtered feedback state
        self.filtered_v = 0.0
        self.filtered_w = 0.0
        self.feedback_filter_initialized = False
        self.last_feedback_filter_turn_sign = 0

        self.vel_pub = self.create_publisher(Twist, '/vel_raw', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.timer = self.create_timer(1.0 / self.command_rate, self.timer_callback)

        self.open_serial()
        self.send_zero()
        self.enable_feedback_upload()

        self.get_logger().info(
            'Motor driver ready: '
            f'max_v={self.max_linear_m_s:.2f} m/s, '
            f'max_w={self.max_angular_rad_s:.2f} rad/s, '
            f'linear_acc={self.max_linear_accel_m_s2:.2f} m/s^2, '
            f'angular_acc={self.max_angular_accel_rad_s2:.2f} rad/s^2, '
            f'cmd_signs=({self.left_cmd_sign:+.0f},{self.right_cmd_sign:+.0f}), '
            f'feedback_signs=({self.left_feedback_sign:+.0f},{self.right_feedback_sign:+.0f}), '
            f'swap_cmd={self.swap_command_wheels}, '
            f'swap_feedback={self.swap_feedback_wheels}, '
            f'min_turn={self.min_turn_wheel_speed_mm_s:.1f} mm/s, '
            f'min_spin={self.min_spin_wheel_speed_mm_s:.1f} mm/s, '
            f'cmd_scales=LF:{self.left_forward_cmd_scale:.2f},'
            f'RF:{self.right_forward_cmd_scale:.2f},'
            f'LR:{self.left_reverse_cmd_scale:.2f},'
            f'RR:{self.right_reverse_cmd_scale:.2f},'
            f'outer:{self.forward_turn_outer_wheel_scale:.2f},'
            f'inner:{self.forward_turn_inner_wheel_scale:.2f},'
            f'Lout:{self.forward_left_turn_outer_wheel_scale:.2f},'
            f'Lin:{self.forward_left_turn_inner_wheel_scale:.2f},'
            f'Rout:{self.forward_right_turn_outer_wheel_scale:.2f},'
            f'Rin:{self.forward_right_turn_inner_wheel_scale:.2f},'
            f'preserve_v:{self.forward_turn_preserve_linear_speed}, '
            f'spin_left:{self.pure_spin_left_cmd_scale:.2f},'
            f'spin_right:{self.pure_spin_right_cmd_scale:.2f}, '
            f'spin_min_left:{self.pure_spin_left_min_wheel_speed_mm_s:.1f},'
            f'spin_min_right:{self.pure_spin_right_min_wheel_speed_mm_s:.1f}, '
            f'straight_w_deadband={self.straight_feedback_w_deadband:.3f} rad/s, '
            f'straight_force_zero={self.straight_feedback_force_zero}, '
            f'smoothing_alpha={self.feedback_smoothing_alpha:.2f}'
        )

    def open_serial(self) -> None:
        if serial is None:
            self.get_logger().error('python3-serial is not available; motor serial port cannot open')
            return

        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.0,
                write_timeout=0.1,
            )
            try:
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
            except Exception:
                pass

            self.rx_buffer = b''
            self.get_logger().info(f'Opened motor serial port {self.port} at {self.baudrate}')
        except (OSError, SerialException) as exc:
            self.serial_port = None
            self.get_logger().error(f'Failed to open motor serial port {self.port}: {exc}')

    def close_serial(self) -> None:
        if self.serial_port is not None:
            try:
                if self.serial_port.is_open:
                    self.serial_port.close()
            except (OSError, SerialException) as exc:
                self.get_logger().warn(f'Motor serial close failed: {exc}')
        self.serial_port = None

    def mark_serial_error(self, reason: str) -> None:
        self.get_logger().warn(reason)
        if self.reconnect_on_error:
            self.close_serial()

    def try_reconnect(self) -> None:
        if not self.reconnect_on_error:
            return

        if self.serial_port is not None and self.serial_port.is_open:
            return

        now = self.get_clock().now()
        elapsed = (now - self.last_reconnect_attempt).nanoseconds / 1e9
        if elapsed < self.reconnect_interval:
            return

        self.last_reconnect_attempt = now
        self.get_logger().warn(f'Trying to reconnect motor serial port {self.port}')
        self.open_serial()
        if self.serial_port is not None and self.serial_port.is_open:
            self.send_zero()
            self.enable_feedback_upload()

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.last_cmd_time = self.get_clock().now()
        self.timed_out = False

        v = self.limit(float(msg.linear.x), -self.max_linear_m_s, self.max_linear_m_s)
        w = self.limit(float(msg.angular.z), -self.max_angular_rad_s, self.max_angular_rad_s)

        # Caster/universal wheel protection:
        # pure spin is mechanically bad for this chassis, so reduce spin-in-place commands.
        if abs(v) < self.pure_spin_v_threshold and abs(w) > 0.0:
            w *= self.pure_spin_angular_scale

        self.target_v = v
        self.target_w = w

    def timer_callback(self) -> None:
        self.try_reconnect()

        now = self.get_clock().now()

        elapsed_cmd = (now - self.last_cmd_time).nanoseconds / 1e9
        if elapsed_cmd > self.cmd_timeout and not self.timed_out:
            self.target_v = 0.0
            self.target_w = 0.0
            self.timed_out = True

        self.update_ramped_command(now)
        self.send_current_speed()

        elapsed_feedback = (now - self.last_feedback_time).nanoseconds / 1e9
        if elapsed_feedback >= (1.0 / self.feedback_rate):
            self.last_feedback_time = now
            if self.request_feedback_every_cycle:
                self.request_feedback()

        self.read_feedback()

    def update_ramped_command(self, now) -> None:
        dt = (now - self.last_ramp_time).nanoseconds / 1e9
        self.last_ramp_time = now

        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / self.command_rate

        max_dv = self.max_linear_accel_m_s2 * dt
        max_dw = self.max_angular_accel_rad_s2 * dt

        self.current_v = self.step_toward(self.current_v, self.target_v, max_dv)
        self.current_w = self.step_toward(self.current_w, self.target_w, max_dw)

    def send_current_speed(self) -> None:
        left_mps = self.current_v - (self.current_w * self.wheel_base / 2.0)
        right_mps = self.current_v + (self.current_w * self.wheel_base / 2.0)

        left_mm_s, right_mm_s = self.calibrate_wheel_pair(
            left_mps * 1000.0,
            right_mps * 1000.0,
        )

        if self.swap_command_wheels:
            left_mm_s, right_mm_s = right_mm_s, left_mm_s

        left_mm_s *= self.left_cmd_sign
        right_mm_s *= self.right_cmd_sign

        m2 = self.prepare_motor_speed(left_mm_s)
        m4 = self.prepare_motor_speed(right_mm_s)

        if self.debug_commands:
            self.get_logger().info(
                f'cmd target(v={self.target_v:.3f}, w={self.target_w:.3f}) '
                f'ramped(v={self.current_v:.3f}, w={self.current_w:.3f}) '
                f'wheel_cmd(left={left_mm_s:.1f}, right={right_mm_s:.1f}) '
                f'M2={m2}, M4={m4}'
            )

        if self.should_send_speed(m2, m4):
            self.send_speed(m2, m4)

    def calibrate_wheel_pair(
        self,
        left_mm_s: float,
        right_mm_s: float,
    ) -> Tuple[float, float]:
        left_mm_s = self.apply_direction_scale(left_mm_s, is_left=True)
        right_mm_s = self.apply_direction_scale(right_mm_s, is_left=False)
        left_mm_s, right_mm_s = self.apply_forward_turn_scale(left_mm_s, right_mm_s)

        left_mm_s = self.apply_command_deadband(
            self.limit(left_mm_s, -self.max_speed_mm_s, self.max_speed_mm_s)
        )
        right_mm_s = self.apply_command_deadband(
            self.limit(right_mm_s, -self.max_speed_mm_s, self.max_speed_mm_s)
        )

        pair_min_speed = 0.0
        if abs(self.current_w) >= self.turn_min_w_threshold_rad_s:
            pair_min_speed = max(pair_min_speed, self.min_turn_wheel_speed_mm_s)
            if abs(self.current_v) < self.pure_spin_v_threshold:
                pair_min_speed = max(pair_min_speed, self.min_spin_wheel_speed_mm_s)

        peak_speed = max(abs(left_mm_s), abs(right_mm_s))
        if 0.0 < peak_speed < pair_min_speed:
            scale = pair_min_speed / peak_speed
            left_mm_s *= scale
            right_mm_s *= scale

        left_mm_s, right_mm_s = self.apply_pure_spin_scale(left_mm_s, right_mm_s)
        left_mm_s, right_mm_s = self.apply_pure_spin_minimum(left_mm_s, right_mm_s)

        left_mm_s = self.apply_per_wheel_minimum(left_mm_s, is_left=True)
        right_mm_s = self.apply_per_wheel_minimum(right_mm_s, is_left=False)

        return (
            self.limit(left_mm_s, -self.max_speed_mm_s, self.max_speed_mm_s),
            self.limit(right_mm_s, -self.max_speed_mm_s, self.max_speed_mm_s),
        )

    def apply_direction_scale(self, speed_mm_s: float, is_left: bool) -> float:
        if speed_mm_s > 0.0:
            scale = self.left_forward_cmd_scale if is_left else self.right_forward_cmd_scale
        elif speed_mm_s < 0.0:
            scale = self.left_reverse_cmd_scale if is_left else self.right_reverse_cmd_scale
        else:
            scale = 1.0
        return speed_mm_s * scale

    def apply_forward_turn_scale(
        self,
        left_mm_s: float,
        right_mm_s: float,
    ) -> Tuple[float, float]:
        if (
            self.current_v <= self.pure_spin_v_threshold
            or abs(self.current_w) < self.turn_min_w_threshold_rad_s
            or left_mm_s <= 0.0
            or right_mm_s <= 0.0
        ):
            return left_mm_s, right_mm_s

        original_mean_speed = (left_mm_s + right_mm_s) * 0.5

        if self.current_w > 0.0:
            left_mm_s *= self.forward_left_turn_inner_wheel_scale
            right_mm_s *= self.forward_left_turn_outer_wheel_scale
        else:
            left_mm_s *= self.forward_right_turn_outer_wheel_scale
            right_mm_s *= self.forward_right_turn_inner_wheel_scale

        if self.forward_turn_preserve_linear_speed:
            scaled_mean_speed = (left_mm_s + right_mm_s) * 0.5
            if scaled_mean_speed > 0.0:
                correction = original_mean_speed / scaled_mean_speed
                left_mm_s *= correction
                right_mm_s *= correction

        return left_mm_s, right_mm_s

    def apply_pure_spin_scale(
        self,
        left_mm_s: float,
        right_mm_s: float,
    ) -> Tuple[float, float]:
        if (
            abs(self.current_v) >= self.pure_spin_v_threshold
            or abs(self.current_w) < self.turn_min_w_threshold_rad_s
        ):
            return left_mm_s, right_mm_s

        scale = (
            self.pure_spin_left_cmd_scale
            if self.current_w > 0.0
            else self.pure_spin_right_cmd_scale
        )

        return left_mm_s * scale, right_mm_s * scale

    def apply_pure_spin_minimum(
        self,
        left_mm_s: float,
        right_mm_s: float,
    ) -> Tuple[float, float]:
        if (
            abs(self.current_v) >= self.pure_spin_v_threshold
            or abs(self.current_w) < self.turn_min_w_threshold_rad_s
        ):
            return left_mm_s, right_mm_s

        min_speed = (
            self.pure_spin_left_min_wheel_speed_mm_s
            if self.current_w > 0.0
            else self.pure_spin_right_min_wheel_speed_mm_s
        )
        if min_speed <= 0.0:
            return left_mm_s, right_mm_s

        return (
            self.apply_signed_minimum(left_mm_s, min_speed),
            self.apply_signed_minimum(right_mm_s, min_speed),
        )

    @staticmethod
    def apply_signed_minimum(speed_mm_s: float, min_speed_mm_s: float) -> float:
        if speed_mm_s == 0.0 or abs(speed_mm_s) >= min_speed_mm_s:
            return speed_mm_s
        return min_speed_mm_s if speed_mm_s > 0.0 else -min_speed_mm_s

    def apply_command_deadband(self, speed_mm_s: float) -> float:
        if abs(speed_mm_s) < self.command_deadband_mm_s:
            return 0.0
        return speed_mm_s

    def apply_per_wheel_minimum(self, speed_mm_s: float, is_left: bool) -> float:
        if speed_mm_s == 0.0:
            return 0.0

        min_speed = self.min_effective_speed_mm_s
        if is_left:
            direction_min = (
                self.left_forward_min_speed_mm_s
                if speed_mm_s > 0.0
                else self.left_reverse_min_speed_mm_s
            )
        else:
            direction_min = (
                self.right_forward_min_speed_mm_s
                if speed_mm_s > 0.0
                else self.right_reverse_min_speed_mm_s
            )

        min_speed = max(min_speed, direction_min)

        if 0.0 < abs(speed_mm_s) < min_speed:
            speed_mm_s = min_speed if speed_mm_s > 0.0 else -min_speed

        return speed_mm_s

    def prepare_motor_speed(self, speed_mm_s: float) -> int:
        speed_mm_s = self.limit(speed_mm_s, -self.max_speed_mm_s, self.max_speed_mm_s)
        return int(round(speed_mm_s))

    def should_send_speed(self, m2: int, m4: int) -> bool:
        if self.last_sent_m2 is None or self.last_sent_m4 is None:
            return True

        if abs(m2 - self.last_sent_m2) >= self.speed_command_epsilon_mm_s:
            return True

        if abs(m4 - self.last_sent_m4) >= self.speed_command_epsilon_mm_s:
            return True

        elapsed = (
            self.get_clock().now() - self.last_speed_command_time
        ).nanoseconds / 1e9
        return elapsed >= self.speed_command_keepalive_sec

    @staticmethod
    def limit(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    @staticmethod
    def positive_scale(value: float) -> float:
        if value <= 0.0:
            return 1.0
        return value

    @staticmethod
    def optional_positive_scale(value: float, fallback: float) -> float:
        if value <= 0.0:
            return fallback
        return value

    @staticmethod
    def step_toward(current: float, target: float, max_step: float) -> float:
        if target > current + max_step:
            return current + max_step
        if target < current - max_step:
            return current - max_step
        return target

    def send_zero(self) -> None:
        self.current_v = 0.0
        self.current_w = 0.0
        self.target_v = 0.0
        self.target_w = 0.0
        self.feedback_filter_initialized = False
        self.last_feedback_filter_turn_sign = 0
        self.send_command('$spd:0,0,0,0#')
        self.last_sent_m2 = 0
        self.last_sent_m4 = 0
        self.last_speed_command_time = self.get_clock().now()

    def send_speed(self, m2: int, m4: int) -> None:
        self.send_command(f'$spd:0,{m2},0,{m4}#')
        self.last_sent_m2 = m2
        self.last_sent_m4 = m4
        self.last_speed_command_time = self.get_clock().now()

    def enable_feedback_upload(self) -> None:
        if self.request_feedback_on_connect:
            self.request_feedback()

    def request_feedback(self) -> None:
        self.send_command('$upload:0,0,1#')

    def send_command(self, command: str) -> None:
        if self.serial_port is None or not self.serial_port.is_open:
            return

        try:
            self.serial_port.write(command.encode('ascii'))
        except (OSError, SerialException) as exc:
            self.mark_serial_error(f'Motor serial write failed: {exc}')

    def read_feedback(self) -> None:
        if self.serial_port is None or not self.serial_port.is_open:
            return

        try:
            available = self.serial_port.in_waiting
            if available <= 0:
                return
            self.rx_buffer += self.serial_port.read(available)
        except (OSError, SerialException) as exc:
            self.mark_serial_error(f'Motor serial read failed: {exc}')
            return

        if len(self.rx_buffer) > 512:
            self.rx_buffer = self.rx_buffer[-512:]

        while True:
            parsed = self.extract_mspd()
            if parsed is None:
                break
            self.handle_feedback(parsed)

    def extract_mspd(self) -> Optional[Tuple[float, float, float, float]]:
        match = MSPD_RE.search(self.rx_buffer)
        if match is None:
            last_start = self.rx_buffer.rfind(b'$')
            if last_start > 0:
                self.rx_buffer = self.rx_buffer[last_start:]
            return None

        self.rx_buffer = self.rx_buffer[match.end():]
        try:
            return tuple(float(value) for value in match.groups())
        except ValueError:
            return None

    def handle_feedback(self, feedback: Tuple[float, float, float, float]) -> None:
        m1, m2, m3, m4 = feedback

        # Robot uses M2 and M4 as the active differential-drive wheels.
        left_feedback = m2
        right_feedback = m4

        if self.swap_feedback_wheels:
            left_feedback, right_feedback = right_feedback, left_feedback

        self.publish_vel_raw(left_feedback, right_feedback, feedback)

    def publish_vel_raw(
        self,
        left_feedback_raw: float,
        right_feedback_raw: float,
        all_feedback: Optional[Tuple[float, float, float, float]] = None,
    ) -> None:
        left_mps = (
            left_feedback_raw
            * self.left_feedback_sign
            * self.feedback_scale
            * self.left_feedback_scale
        ) / 1000.0
        right_mps = (
            right_feedback_raw
            * self.right_feedback_sign
            * self.feedback_scale
            * self.right_feedback_scale
        ) / 1000.0

        raw_v = (left_mps + right_mps) / 2.0
        raw_w = (right_mps - left_mps) / self.wheel_base

        publish_v = raw_v
        publish_w = raw_w

        commanded_straight = (
            abs(self.current_w) <= self.straight_cmd_w_epsilon
            and abs(self.current_v) >= self.straight_feedback_force_zero_min_v_m_s
        )

        # Kill small fake yaw caused by encoder quantization only when the robot
        # is commanded to drive straight. Real turn commands are not suppressed.
        if commanded_straight and abs(publish_w) <= self.straight_feedback_w_deadband:
            publish_w = 0.0

        alpha = self.feedback_smoothing_alpha
        turn_sign = self.feedback_filter_turn_sign()
        if turn_sign != self.last_feedback_filter_turn_sign:
            self.feedback_filter_initialized = False
            if turn_sign == 0:
                self.filtered_w = 0.0
            self.last_feedback_filter_turn_sign = turn_sign

        if alpha <= 0.0:
            # alpha=0 means hold last filtered value after initialization.
            # This is not useful for normal driving, but it is safe.
            if self.feedback_filter_initialized:
                publish_v = self.filtered_v
                publish_w = self.filtered_w
        elif alpha < 1.0:
            if not self.feedback_filter_initialized:
                self.filtered_v = publish_v
                self.filtered_w = publish_w
                self.feedback_filter_initialized = True
            else:
                self.filtered_v = alpha * publish_v + (1.0 - alpha) * self.filtered_v
                self.filtered_w = alpha * publish_w + (1.0 - alpha) * self.filtered_w
                publish_v = self.filtered_v
                publish_w = self.filtered_w
        else:
            self.filtered_v = publish_v
            self.filtered_w = publish_w
            self.feedback_filter_initialized = True

        # If the command path is explicitly straight, do not let mismatched
        # encoder channels inject a large yaw rate into /vel_raw. Real yaw while
        # driving straight should come from the IMU in the EKF, not from bad
        # wheel-difference feedback.
        if commanded_straight and self.straight_feedback_force_zero:
            publish_w = 0.0
            self.filtered_w = 0.0

        self.maybe_log_feedback(
            all_feedback=all_feedback,
            left_mps=left_mps,
            right_mps=right_mps,
            raw_v=raw_v,
            raw_w=raw_w,
            publish_v=publish_v,
            publish_w=publish_w,
        )

        msg = Twist()
        msg.linear.x = publish_v
        msg.linear.y = 0.0
        msg.angular.z = publish_w
        self.vel_pub.publish(msg)

    def feedback_filter_turn_sign(self) -> int:
        if abs(self.current_w) <= self.straight_cmd_w_epsilon:
            return 0
        return 1 if self.current_w > 0.0 else -1

    def maybe_log_feedback(
        self,
        all_feedback: Optional[Tuple[float, float, float, float]],
        left_mps: float,
        right_mps: float,
        raw_v: float,
        raw_w: float,
        publish_v: float,
        publish_w: float,
    ) -> None:
        if not self.debug_feedback:
            return

        now = self.get_clock().now()
        elapsed = (now - self.last_feedback_debug_log).nanoseconds / 1e9
        if elapsed < (1.0 / self.debug_feedback_rate):
            return

        self.last_feedback_debug_log = now

        if all_feedback is None:
            feedback_text = 'raw feedback unavailable'
        else:
            m1, m2, m3, m4 = all_feedback
            feedback_text = f'M1={m1:.3f}, M2={m2:.3f}, M3={m3:.3f}, M4={m4:.3f}'

        self.get_logger().info(
            f'feedback {feedback_text} | '
            f'left={left_mps:.4f} m/s, right={right_mps:.4f} m/s | '
            f'raw(v={raw_v:.4f}, w={raw_w:.4f}) | '
            f'pub(v={publish_v:.4f}, w={publish_w:.4f})'
        )

    def shutdown(self) -> None:
        self.send_zero()
        self.close_serial()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorSerialDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
