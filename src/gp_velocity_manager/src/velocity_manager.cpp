#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using Twist = geometry_msgs::msg::Twist;
using LaserScan = sensor_msgs::msg::LaserScan;
using Odometry = nav_msgs::msg::Odometry;

class VelocityManager : public rclcpp::Node
{
public:
  VelocityManager() : Node("gp_velocity_manager")
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/cmd_vel");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/cmd_vel_motor");

    scan_speed_scaling_enable_ =
      this->declare_parameter<bool>("scan_speed_scaling_enable", false);
    scan_topic_ = this->declare_parameter<std::string>("scan_topic", "/scan");
    scan_speed_scaling_stale_sec_ =
      this->declare_parameter<double>("scan_speed_scaling_stale_sec", 0.75);
    scan_speed_scaling_sector_deg_ =
      this->declare_parameter<double>("scan_speed_scaling_sector_deg", 70.0);
    scan_speed_scaling_open_clearance_m_ =
      this->declare_parameter<double>("scan_speed_scaling_open_clearance_m", 0.90);
    scan_speed_scaling_medium_clearance_m_ =
      this->declare_parameter<double>("scan_speed_scaling_medium_clearance_m", 0.65);
    scan_speed_scaling_tight_clearance_m_ =
      this->declare_parameter<double>("scan_speed_scaling_tight_clearance_m", 0.45);
    scan_speed_scaling_danger_clearance_m_ =
      this->declare_parameter<double>("scan_speed_scaling_danger_clearance_m", 0.30);
    scan_speed_scaling_stop_clearance_m_ =
      this->declare_parameter<double>("scan_speed_scaling_stop_clearance_m", 0.22);
    scan_speed_scaling_medium_scale_ =
      this->declare_parameter<double>("scan_speed_scaling_medium_scale", 0.75);
    scan_speed_scaling_tight_scale_ =
      this->declare_parameter<double>("scan_speed_scaling_tight_scale", 0.60);
    scan_speed_scaling_danger_scale_ =
      this->declare_parameter<double>("scan_speed_scaling_danger_scale", 0.40);
    scan_speed_scaling_scale_angular_ =
      this->declare_parameter<bool>("scan_speed_scaling_scale_angular", false);
    scan_speed_scaling_min_angular_scale_ =
      this->declare_parameter<double>("scan_speed_scaling_min_angular_scale", 0.70);
    scan_speed_scaling_stop_forward_ =
      this->declare_parameter<bool>("scan_speed_scaling_stop_forward", true);

    max_linear_m_s_ = this->declare_parameter<double>("max_linear_m_s", 0.22);
    max_angular_rad_s_ = this->declare_parameter<double>("max_angular_rad_s", 0.65);

    max_linear_accel_m_s2_ = this->declare_parameter<double>("max_linear_accel_m_s2", 1.0);
    max_angular_accel_rad_s2_ = this->declare_parameter<double>("max_angular_accel_rad_s2", 4.0);

    linear_deadband_m_s_ = this->declare_parameter<double>("linear_deadband_m_s", 0.003);
    angular_deadband_rad_s_ = this->declare_parameter<double>("angular_deadband_rad_s", 0.03);

    pure_spin_v_threshold_ = this->declare_parameter<double>("pure_spin_v_threshold", 0.03);
    min_pure_spin_enable_ = this->declare_parameter<bool>("min_pure_spin_enable", false);
    min_pure_spin_angular_rad_s_ = this->declare_parameter<double>("min_pure_spin_angular_rad_s", 0.35);

    pure_spin_start_boost_enable_ =
      this->declare_parameter<bool>("pure_spin_start_boost_enable", false);
    pure_spin_start_boost_angular_rad_s_ =
      this->declare_parameter<double>("pure_spin_start_boost_angular_rad_s", 0.65);
    pure_spin_start_boost_duration_s_ =
      this->declare_parameter<double>("pure_spin_start_boost_duration_s", 0.18);

    turn_settle_enable_ =
      this->declare_parameter<bool>("turn_settle_enable", false);
    turn_settle_forward_v_threshold_m_s_ =
      this->declare_parameter<double>("turn_settle_forward_v_threshold_m_s", 0.03);
    turn_settle_start_w_threshold_rad_s_ =
      this->declare_parameter<double>("turn_settle_start_w_threshold_rad_s", 0.12);
    turn_settle_target_w_threshold_rad_s_ =
      this->declare_parameter<double>("turn_settle_target_w_threshold_rad_s", 0.05);
    turn_settle_done_w_threshold_rad_s_ =
      this->declare_parameter<double>("turn_settle_done_w_threshold_rad_s", 0.05);
    turn_settle_linear_cap_m_s_ =
      this->declare_parameter<double>("turn_settle_linear_cap_m_s", 0.02);
    turn_settle_min_hold_s_ =
      this->declare_parameter<double>("turn_settle_min_hold_s", 0.20);
    turn_settle_timeout_s_ =
      this->declare_parameter<double>("turn_settle_timeout_s", 0.40);

    post_spin_realign_enable_ =
      this->declare_parameter<bool>("post_spin_realign_enable", false);
    post_spin_realign_recent_window_s_ =
      this->declare_parameter<double>("post_spin_realign_recent_window_s", 5.0);
    post_spin_realign_forward_v_threshold_m_s_ =
      this->declare_parameter<double>("post_spin_realign_forward_v_threshold_m_s", 0.03);
    post_spin_realign_spin_w_threshold_rad_s_ =
      this->declare_parameter<double>("post_spin_realign_spin_w_threshold_rad_s", 0.12);
    post_spin_realign_target_w_threshold_rad_s_ =
      this->declare_parameter<double>("post_spin_realign_target_w_threshold_rad_s", 0.05);
    post_spin_realign_linear_cap_m_s_ =
      this->declare_parameter<double>("post_spin_realign_linear_cap_m_s", 0.05);
    post_spin_realign_support_start_ratio_ =
      this->declare_parameter<double>("post_spin_realign_support_start_ratio", 1.00);
    post_spin_realign_support_end_ratio_ =
      this->declare_parameter<double>("post_spin_realign_support_end_ratio", 0.70);
    post_spin_realign_support_duration_s_ =
      this->declare_parameter<double>("post_spin_realign_support_duration_s", 2.00);
    // Closed-loop overshoot guard. The counter-yaw is open-loop; if the
    // chassis yaws in the assist direction faster than this threshold
    // (per EKF) while moving forward, the caster has clearly engaged —
    // terminate the assist to avoid pushing through into reverse drift.
    post_spin_realign_overshoot_kill_enable_ =
      this->declare_parameter<bool>("post_spin_realign_overshoot_kill_enable", true);
    post_spin_realign_overshoot_kill_w_rad_s_ =
      this->declare_parameter<double>("post_spin_realign_overshoot_kill_w_rad_s", 0.10);
    post_spin_realign_overshoot_kill_min_v_m_s_ =
      this->declare_parameter<double>("post_spin_realign_overshoot_kill_min_v_m_s", 0.01);

    // Rotation-to-Forward Assist (RTFA): one-shot ramp on the rising edge
    // from in-place rotation to forward motion. Linear vx ramps from a low
    // floor to commanded over x_ramp_duration; counter-yaw (same sign as
    // last spin, opposite to caster drift) injects with a growing ratio
    // across z_assist_duration. Catches the drift window that
    // yaw_stabilization cannot — it gates on min forward velocity and is
    // disarmed during the dangerous sub-threshold startup moment.
    rotation_to_forward_assist_enable_ =
      this->declare_parameter<bool>("rotation_to_forward_assist_enable", false);
    rotation_to_forward_assist_spin_w_threshold_rad_s_ =
      this->declare_parameter<double>("rotation_to_forward_assist_spin_w_threshold_rad_s", 0.12);
    rotation_to_forward_assist_idle_v_threshold_m_s_ =
      this->declare_parameter<double>("rotation_to_forward_assist_idle_v_threshold_m_s", 0.02);
    rotation_to_forward_assist_fire_v_threshold_m_s_ =
      this->declare_parameter<double>("rotation_to_forward_assist_fire_v_threshold_m_s", 0.02);
    rotation_to_forward_assist_arm_min_duration_s_ =
      this->declare_parameter<double>("rotation_to_forward_assist_arm_min_duration_s", 0.10);
    rotation_to_forward_assist_arm_recent_window_s_ =
      this->declare_parameter<double>("rotation_to_forward_assist_arm_recent_window_s", 5.0);
    rotation_to_forward_assist_x_ramp_duration_s_ =
      this->declare_parameter<double>("rotation_to_forward_assist_x_ramp_duration_s", 2.0);
    rotation_to_forward_assist_x_ramp_start_m_s_ =
      this->declare_parameter<double>("rotation_to_forward_assist_x_ramp_start_m_s", 0.06);
    rotation_to_forward_assist_z_assist_duration_s_ =
      this->declare_parameter<double>("rotation_to_forward_assist_z_assist_duration_s", 1.5);
    rotation_to_forward_assist_z_assist_start_ratio_ =
      this->declare_parameter<double>("rotation_to_forward_assist_z_assist_start_ratio", 0.4);
    rotation_to_forward_assist_z_assist_end_ratio_ =
      this->declare_parameter<double>("rotation_to_forward_assist_z_assist_end_ratio", 0.8);
    rotation_to_forward_assist_z_assist_max_rad_s_ =
      this->declare_parameter<double>("rotation_to_forward_assist_z_assist_max_rad_s", 0.25);
    rotation_to_forward_assist_z_inject_max_nav_w_rad_s_ =
      this->declare_parameter<double>("rotation_to_forward_assist_z_inject_max_nav_w_rad_s", 0.08);
    rotation_to_forward_assist_abort_nav_w_rad_s_ =
      this->declare_parameter<double>("rotation_to_forward_assist_abort_nav_w_rad_s", 0.20);
    rotation_to_forward_assist_overshoot_kill_w_rad_s_ =
      this->declare_parameter<double>("rotation_to_forward_assist_overshoot_kill_w_rad_s", 0.10);

    cmd_timeout_s_ = this->declare_parameter<double>("cmd_timeout_s", 0.35);
    output_rate_hz_ = this->declare_parameter<double>("output_rate_hz", 20.0);
    publish_keepalive_s_ = this->declare_parameter<double>("publish_keepalive_s", 0.20);
    publish_linear_epsilon_m_s_ =
      this->declare_parameter<double>("publish_linear_epsilon_m_s", 0.001);
    publish_angular_epsilon_rad_s_ =
      this->declare_parameter<double>("publish_angular_epsilon_rad_s", 0.005);
    immediate_stop_ = this->declare_parameter<bool>("immediate_stop", true);
    debug_ = this->declare_parameter<bool>("debug", false);

    // M1.A: supervisor priority arbitration.
    // When messages are seen on supervisor_input_topic, /cmd_vel is ignored
    // until supervisor_priority_timeout_s has elapsed since the last one.
    supervisor_input_topic_ = this->declare_parameter<std::string>(
      "supervisor_input_topic", "/cmd_vel_supervisor");
    supervisor_priority_enable_ = this->declare_parameter<bool>(
      "supervisor_priority_enable", true);
    supervisor_priority_timeout_s_ = this->declare_parameter<double>(
      "supervisor_priority_timeout_s", 0.20);
    supervisor_priority_timeout_s_ = std::max(0.02, supervisor_priority_timeout_s_);

    // Closed-loop yaw rate stabilization. Subscribes to odom (typically
    // EKF-fused /odometry/filtered) and corrects the commanded yaw rate
    // toward the actual yaw rate so chassis tracks intent despite caster
    // drag, wheel slip, and asymmetric friction.
    odom_input_topic_ = this->declare_parameter<std::string>(
      "odom_input_topic", "/odometry/filtered");
    yaw_stabilization_enable_ = this->declare_parameter<bool>(
      "yaw_stabilization_enable", true);
    yaw_stabilization_kp_ = this->declare_parameter<double>(
      "yaw_stabilization_kp", 1.5);
    yaw_stabilization_max_correction_rad_s_ = this->declare_parameter<double>(
      "yaw_stabilization_max_correction_rad_s", 0.30);
    yaw_stabilization_odom_stale_sec_ = this->declare_parameter<double>(
      "yaw_stabilization_odom_stale_sec", 0.30);
    yaw_stabilization_log_threshold_rad_s_ = this->declare_parameter<double>(
      "yaw_stabilization_log_threshold_rad_s", 0.15);
    // Speed gate — at near-zero forward velocity actual yaw is mostly
    // sensor noise; correcting on noise wastes wheel motion and at high
    // gains can drive instability. Disable yaw stab below this speed.
    yaw_stabilization_min_forward_v_m_s_ = this->declare_parameter<double>(
      "yaw_stabilization_min_forward_v_m_s", 0.03);

    // Soft start — cap forward velocity briefly after every zero→forward
    // transition so the caster wheel can swivel into alignment naturally
    // during the gentle ramp instead of being fought by the yaw stabilizer.
    soft_start_enable_ = this->declare_parameter<bool>(
      "soft_start_enable", true);
    soft_start_duration_s_ = this->declare_parameter<double>(
      "soft_start_duration_s", 0.6);
    soft_start_linear_cap_m_s_ = this->declare_parameter<double>(
      "soft_start_linear_cap_m_s", 0.05);
    soft_start_idle_threshold_m_s_ = this->declare_parameter<double>(
      "soft_start_idle_threshold_m_s", 0.02);

    max_linear_m_s_ = std::abs(max_linear_m_s_);
    max_angular_rad_s_ = std::abs(max_angular_rad_s_);
    scan_speed_scaling_stale_sec_ = std::max(0.05, scan_speed_scaling_stale_sec_);
    scan_speed_scaling_sector_deg_ =
      clamp(std::abs(scan_speed_scaling_sector_deg_), 5.0, 180.0);
    scan_speed_scaling_stop_clearance_m_ =
      std::max(0.0, scan_speed_scaling_stop_clearance_m_);
    scan_speed_scaling_danger_clearance_m_ =
      std::max(scan_speed_scaling_stop_clearance_m_, scan_speed_scaling_danger_clearance_m_);
    scan_speed_scaling_tight_clearance_m_ =
      std::max(scan_speed_scaling_danger_clearance_m_, scan_speed_scaling_tight_clearance_m_);
    scan_speed_scaling_medium_clearance_m_ =
      std::max(scan_speed_scaling_tight_clearance_m_, scan_speed_scaling_medium_clearance_m_);
    scan_speed_scaling_open_clearance_m_ =
      std::max(scan_speed_scaling_medium_clearance_m_, scan_speed_scaling_open_clearance_m_);
    scan_speed_scaling_medium_scale_ =
      clamp(scan_speed_scaling_medium_scale_, 0.0, 1.0);
    scan_speed_scaling_tight_scale_ =
      clamp(scan_speed_scaling_tight_scale_, 0.0, scan_speed_scaling_medium_scale_);
    scan_speed_scaling_danger_scale_ =
      clamp(scan_speed_scaling_danger_scale_, 0.0, scan_speed_scaling_tight_scale_);
    scan_speed_scaling_min_angular_scale_ =
      clamp(scan_speed_scaling_min_angular_scale_, 0.0, 1.0);
    max_linear_accel_m_s2_ = std::abs(max_linear_accel_m_s2_);
    max_angular_accel_rad_s2_ = std::abs(max_angular_accel_rad_s2_);
    min_pure_spin_angular_rad_s_ = std::abs(min_pure_spin_angular_rad_s_);
    pure_spin_start_boost_angular_rad_s_ = std::abs(pure_spin_start_boost_angular_rad_s_);
    turn_settle_forward_v_threshold_m_s_ = std::abs(turn_settle_forward_v_threshold_m_s_);
    turn_settle_start_w_threshold_rad_s_ = std::abs(turn_settle_start_w_threshold_rad_s_);
    turn_settle_target_w_threshold_rad_s_ = std::abs(turn_settle_target_w_threshold_rad_s_);
    turn_settle_done_w_threshold_rad_s_ = std::abs(turn_settle_done_w_threshold_rad_s_);
    turn_settle_linear_cap_m_s_ = std::abs(turn_settle_linear_cap_m_s_);
    turn_settle_min_hold_s_ = std::max(0.0, turn_settle_min_hold_s_);
    turn_settle_timeout_s_ = std::max(0.0, turn_settle_timeout_s_);
    post_spin_realign_recent_window_s_ = std::max(0.0, post_spin_realign_recent_window_s_);
    post_spin_realign_forward_v_threshold_m_s_ =
      std::abs(post_spin_realign_forward_v_threshold_m_s_);
    post_spin_realign_spin_w_threshold_rad_s_ =
      std::abs(post_spin_realign_spin_w_threshold_rad_s_);
    post_spin_realign_target_w_threshold_rad_s_ =
      std::abs(post_spin_realign_target_w_threshold_rad_s_);
    post_spin_realign_linear_cap_m_s_ = std::abs(post_spin_realign_linear_cap_m_s_);
    post_spin_realign_support_start_ratio_ =
      std::max(0.0, post_spin_realign_support_start_ratio_);
    post_spin_realign_support_end_ratio_ =
      std::max(0.0, post_spin_realign_support_end_ratio_);
    post_spin_realign_support_duration_s_ =
      std::max(0.0, post_spin_realign_support_duration_s_);
    post_spin_realign_overshoot_kill_w_rad_s_ =
      std::abs(post_spin_realign_overshoot_kill_w_rad_s_);
    post_spin_realign_overshoot_kill_min_v_m_s_ =
      std::abs(post_spin_realign_overshoot_kill_min_v_m_s_);

    rotation_to_forward_assist_spin_w_threshold_rad_s_ =
      std::abs(rotation_to_forward_assist_spin_w_threshold_rad_s_);
    rotation_to_forward_assist_idle_v_threshold_m_s_ =
      std::abs(rotation_to_forward_assist_idle_v_threshold_m_s_);
    rotation_to_forward_assist_fire_v_threshold_m_s_ =
      std::abs(rotation_to_forward_assist_fire_v_threshold_m_s_);
    rotation_to_forward_assist_arm_min_duration_s_ =
      std::max(0.0, rotation_to_forward_assist_arm_min_duration_s_);
    rotation_to_forward_assist_arm_recent_window_s_ =
      std::max(0.0, rotation_to_forward_assist_arm_recent_window_s_);
    rotation_to_forward_assist_x_ramp_duration_s_ =
      std::max(0.05, rotation_to_forward_assist_x_ramp_duration_s_);
    rotation_to_forward_assist_x_ramp_start_m_s_ =
      std::max(0.0, rotation_to_forward_assist_x_ramp_start_m_s_);
    rotation_to_forward_assist_z_assist_duration_s_ =
      std::max(0.0, rotation_to_forward_assist_z_assist_duration_s_);
    rotation_to_forward_assist_z_assist_start_ratio_ =
      std::max(0.0, rotation_to_forward_assist_z_assist_start_ratio_);
    rotation_to_forward_assist_z_assist_end_ratio_ =
      std::max(0.0, rotation_to_forward_assist_z_assist_end_ratio_);
    rotation_to_forward_assist_z_assist_max_rad_s_ =
      std::abs(rotation_to_forward_assist_z_assist_max_rad_s_);
    rotation_to_forward_assist_z_inject_max_nav_w_rad_s_ =
      std::abs(rotation_to_forward_assist_z_inject_max_nav_w_rad_s_);
    rotation_to_forward_assist_abort_nav_w_rad_s_ =
      std::abs(rotation_to_forward_assist_abort_nav_w_rad_s_);
    rotation_to_forward_assist_overshoot_kill_w_rad_s_ =
      std::abs(rotation_to_forward_assist_overshoot_kill_w_rad_s_);

    if (output_rate_hz_ < 1.0) {
      output_rate_hz_ = 20.0;
    }
    publish_keepalive_s_ = std::max(0.05, publish_keepalive_s_);
    publish_linear_epsilon_m_s_ = std::abs(publish_linear_epsilon_m_s_);
    publish_angular_epsilon_rad_s_ = std::abs(publish_angular_epsilon_rad_s_);

    auto now = this->now();
    last_cmd_time_ = now;
    last_scan_time_ = now;
    last_update_time_ = now;
    last_publish_time_ = now;
    boost_until_time_ = now;
    recent_turn_until_time_ = now;
    turn_settle_min_until_time_ = now;
    turn_settle_until_time_ = now;
    recent_pure_spin_until_time_ = now;
    post_spin_realign_start_time_ = now;
    post_spin_realign_until_time_ = now;
    last_supervisor_cmd_time_ = now;
    last_odom_time_ = now;
    soft_start_until_time_ = now;
    rtfa_arm_start_time_ = now;
    rtfa_fire_time_ = now;

    pub_ = this->create_publisher<Twist>(output_topic_, 10);

    sub_ = this->create_subscription<Twist>(
      input_topic_,
      10,
      std::bind(&VelocityManager::cmdCallback, this, std::placeholders::_1));

    if (supervisor_priority_enable_) {
      supervisor_sub_ = this->create_subscription<Twist>(
        supervisor_input_topic_,
        10,
        std::bind(&VelocityManager::supervisorCmdCallback, this, std::placeholders::_1));
    }

    if (yaw_stabilization_enable_) {
      odom_sub_ = this->create_subscription<Odometry>(
        odom_input_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&VelocityManager::odomCallback, this, std::placeholders::_1));
    }

    if (scan_speed_scaling_enable_) {
      scan_sub_ = this->create_subscription<LaserScan>(
        scan_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&VelocityManager::scanCallback, this, std::placeholders::_1));
    }

    auto period = std::chrono::duration<double>(1.0 / output_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&VelocityManager::timerCallback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Velocity manager ready: %s -> %s, max_v=%.2f, max_w=%.2f, rate=%.1fHz, keepalive=%.2fs, scan_scale=%s topic=%s sector=%.0fdeg stop=%.2f danger=%.2f tight=%.2f medium=%.2f open=%.2f, min_spin=%s %.2f, boost=%s %.2f for %.2fs, turn_settle=%s cap=%.2f done_w=%.2f hold=%.2fs timeout=%.2fs, post_spin=%s cap=%.2f support_ratio=%.2f->%.2f duration=%.2fs",
      input_topic_.c_str(),
      output_topic_.c_str(),
      max_linear_m_s_,
      max_angular_rad_s_,
      output_rate_hz_,
      publish_keepalive_s_,
      scan_speed_scaling_enable_ ? "on" : "off",
      scan_topic_.c_str(),
      scan_speed_scaling_sector_deg_,
      scan_speed_scaling_stop_clearance_m_,
      scan_speed_scaling_danger_clearance_m_,
      scan_speed_scaling_tight_clearance_m_,
      scan_speed_scaling_medium_clearance_m_,
      scan_speed_scaling_open_clearance_m_,
      min_pure_spin_enable_ ? "on" : "off",
      min_pure_spin_angular_rad_s_,
      pure_spin_start_boost_enable_ ? "on" : "off",
      pure_spin_start_boost_angular_rad_s_,
      pure_spin_start_boost_duration_s_,
      turn_settle_enable_ ? "on" : "off",
      turn_settle_linear_cap_m_s_,
      turn_settle_done_w_threshold_rad_s_,
      turn_settle_min_hold_s_,
      turn_settle_timeout_s_,
      post_spin_realign_enable_ ? "on" : "off",
      post_spin_realign_linear_cap_m_s_,
      post_spin_realign_support_start_ratio_,
      post_spin_realign_support_end_ratio_,
      post_spin_realign_support_duration_s_);
  }

private:
  static constexpr double kPi = 3.14159265358979323846;

  static double sign(double x)
  {
    return (x >= 0.0) ? 1.0 : -1.0;
  }

  static double clamp(double value, double lower, double upper)
  {
    return std::max(lower, std::min(upper, value));
  }

  static double normalizeAngle(double angle)
  {
    while (angle > kPi) {
      angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
      angle += 2.0 * kPi;
    }
    return angle;
  }

  static double stepToward(double current, double target, double max_step)
  {
    if (target > current + max_step) {
      return current + max_step;
    }
    if (target < current - max_step) {
      return current - max_step;
    }
    return target;
  }

  void scanCallback(const LaserScan::SharedPtr msg)
  {
    latest_front_clearance_m_ = frontClearanceFromScan(*msg);
    last_scan_time_ = this->now();
    have_scan_ = true;
  }

  double frontClearanceFromScan(const LaserScan & scan) const
  {
    const double half_sector_rad = 0.5 * scan_speed_scaling_sector_deg_ * kPi / 180.0;
    double min_range = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const float range_value = scan.ranges[i];
      if (!std::isfinite(range_value)) {
        continue;
      }

      const double range = static_cast<double>(range_value);
      if (range < static_cast<double>(scan.range_min) ||
        range > static_cast<double>(scan.range_max))
      {
        continue;
      }

      const double angle =
        static_cast<double>(scan.angle_min) + (static_cast<double>(i) * scan.angle_increment);
      if (std::abs(normalizeAngle(angle)) > half_sector_rad) {
        continue;
      }

      min_range = std::min(min_range, range);
    }

    return min_range;
  }

  bool scanScalingAvailable(const rclcpp::Time & now) const
  {
    if (!scan_speed_scaling_enable_ || !have_scan_) {
      return false;
    }

    const double scan_age = (now - last_scan_time_).nanoseconds() * 1e-9;
    return scan_age <= scan_speed_scaling_stale_sec_;
  }

  double scanSpeedScale(const rclcpp::Time & now) const
  {
    if (!scanScalingAvailable(now) || !std::isfinite(latest_front_clearance_m_)) {
      return 1.0;
    }

    if (latest_front_clearance_m_ >= scan_speed_scaling_open_clearance_m_) {
      return 1.0;
    }
    if (latest_front_clearance_m_ >= scan_speed_scaling_medium_clearance_m_) {
      return scan_speed_scaling_medium_scale_;
    }
    if (latest_front_clearance_m_ >= scan_speed_scaling_tight_clearance_m_) {
      return scan_speed_scaling_tight_scale_;
    }
    return scan_speed_scaling_danger_scale_;
  }

  bool shouldStopForwardForScan(const rclcpp::Time & now) const
  {
    return scan_speed_scaling_stop_forward_ &&
      scanScalingAvailable(now) &&
      std::isfinite(latest_front_clearance_m_) &&
      latest_front_clearance_m_ < scan_speed_scaling_stop_clearance_m_;
  }

  double angularScaleForScan(double linear_scale) const
  {
    if (!scan_speed_scaling_scale_angular_) {
      return 1.0;
    }
    return std::max(scan_speed_scaling_min_angular_scale_, linear_scale);
  }

  bool isSupervisorPriorityActive(const rclcpp::Time & now) const
  {
    if (!supervisor_priority_enable_ || !have_supervisor_cmd_) {
      return false;
    }
    const double age = (now - last_supervisor_cmd_time_).nanoseconds() * 1e-9;
    return age <= supervisor_priority_timeout_s_;
  }

  void cmdCallback(const Twist::SharedPtr msg)
  {
    const auto now = this->now();
    if (isSupervisorPriorityActive(now)) {
      return;
    }
    if (supervisor_priority_logged_) {
      supervisor_priority_logged_ = false;
      RCLCPP_INFO(get_logger(), "Supervisor priority released — /cmd_vel resumed");
    }
    processIncomingCmd(msg);
  }

  void supervisorCmdCallback(const Twist::SharedPtr msg)
  {
    last_supervisor_cmd_time_ = this->now();
    have_supervisor_cmd_ = true;
    if (!supervisor_priority_logged_) {
      supervisor_priority_logged_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Supervisor priority active on %s — /cmd_vel ignored",
        supervisor_input_topic_.c_str());
    }
    processIncomingCmd(msg);
  }

  void odomCallback(const Odometry::SharedPtr msg)
  {
    actual_yaw_rate_ = msg->twist.twist.angular.z;
    last_odom_time_ = this->now();
    have_odom_ = true;
  }

  void processIncomingCmd(const Twist::SharedPtr msg)
  {
    const auto now = this->now();
    last_cmd_time_ = now;
    have_cmd_ = true;

    double v = clamp(msg->linear.x, -max_linear_m_s_, max_linear_m_s_);
    double w = clamp(msg->angular.z, -max_angular_rad_s_, max_angular_rad_s_);

    if (std::abs(v) < linear_deadband_m_s_) {
      v = 0.0;
    }
    if (std::abs(w) < angular_deadband_rad_s_) {
      w = 0.0;
    }

    const bool stop_cmd = (v == 0.0 && w == 0.0);
    const bool active_turn_cmd = std::abs(w) >= turn_settle_start_w_threshold_rad_s_;
    if (turn_settle_enable_ && active_turn_cmd) {
      recent_turn_until_time_ =
        now + rclcpp::Duration::from_seconds(turn_settle_timeout_s_);
    }

    const bool active_pure_spin_cmd =
      std::abs(v) < pure_spin_v_threshold_ &&
      std::abs(w) >= post_spin_realign_spin_w_threshold_rad_s_;
    if (post_spin_realign_enable_ && active_pure_spin_cmd) {
      const int spin_sign = (w > 0.0) ? 1 : -1;
      if (!post_spin_realign_armed_ || last_pure_spin_sign_ != spin_sign) {
        last_pure_spin_abs_w_ = 0.0;
      }

      post_spin_realign_armed_ = true;
      last_pure_spin_sign_ = spin_sign;
      last_pure_spin_abs_w_ = std::max(last_pure_spin_abs_w_, std::abs(w));
      recent_pure_spin_until_time_ =
        now + rclcpp::Duration::from_seconds(post_spin_realign_recent_window_s_);
    } else if (
      post_spin_realign_armed_ &&
      now.nanoseconds() > recent_pure_spin_until_time_.nanoseconds())
    {
      post_spin_realign_armed_ = false;
      last_pure_spin_sign_ = 0;
      last_pure_spin_abs_w_ = 0.0;
    }

    // RTFA arm/fire state machine. ARMED while in-place rotation is active
    // (|w| above spin threshold, |v| below idle threshold). FIRES when the
    // first forward command (v above fire threshold, positive) is seen
    // after the arm window held for at least the debounce duration.
    if (rotation_to_forward_assist_enable_) {
      const bool in_rotation =
        std::abs(w) >= rotation_to_forward_assist_spin_w_threshold_rad_s_ &&
        std::abs(v) < rotation_to_forward_assist_idle_v_threshold_m_s_;

      if (in_rotation) {
        if (!rtfa_armed_) {
          rtfa_armed_ = true;
          rtfa_arm_start_time_ = now;
          rtfa_last_spin_abs_w_ = 0.0;
        }
        rtfa_last_spin_sign_ = (w > 0.0) ? 1 : -1;
        rtfa_last_spin_abs_w_ = std::max(rtfa_last_spin_abs_w_, std::abs(w));
      } else if (rtfa_armed_ && !rtfa_active_) {
        const double armed_age = (now - rtfa_arm_start_time_).nanoseconds() * 1e-9;
        const bool fire_speed_reached =
          v > rotation_to_forward_assist_fire_v_threshold_m_s_;
        const bool min_arm_satisfied =
          armed_age >= rotation_to_forward_assist_arm_min_duration_s_;
        const bool arm_window_open =
          armed_age <= rotation_to_forward_assist_arm_recent_window_s_;

        if (fire_speed_reached && min_arm_satisfied && arm_window_open) {
          rtfa_active_ = true;
          rtfa_fire_time_ = now;
          rtfa_armed_ = false;
          RCLCPP_INFO(
            get_logger(),
            "RTFA engaged: last_spin_w=%.2f rad/s sign=%d, x_ramp=%.2fs from %.2fm/s, z_assist=%.2fs ratio %.2f->%.2f",
            rtfa_last_spin_abs_w_, rtfa_last_spin_sign_,
            rotation_to_forward_assist_x_ramp_duration_s_,
            rotation_to_forward_assist_x_ramp_start_m_s_,
            rotation_to_forward_assist_z_assist_duration_s_,
            rotation_to_forward_assist_z_assist_start_ratio_,
            rotation_to_forward_assist_z_assist_end_ratio_);
        } else if (!arm_window_open) {
          rtfa_armed_ = false;
        }
      }
    }

    if (stop_cmd && immediate_stop_) {
      target_v_ = 0.0;
      target_w_ = 0.0;
      current_v_ = 0.0;
      current_w_ = 0.0;
      boost_active_ = false;
      turn_settle_active_ = false;
      post_spin_realign_active_ = false;
      rtfa_active_ = false;
      rtfa_armed_ = false;
      publishCurrent(true);
      return;
    }

    const bool requested_straight_motion =
      std::abs(v) >= turn_settle_forward_v_threshold_m_s_ &&
      std::abs(w) <= turn_settle_target_w_threshold_rad_s_;

    const bool requested_post_spin_forward =
      std::abs(v) >= post_spin_realign_forward_v_threshold_m_s_ &&
      std::abs(w) <= post_spin_realign_target_w_threshold_rad_s_;

    if (
      post_spin_realign_enable_ &&
      post_spin_realign_armed_ &&
      requested_post_spin_forward &&
      now.nanoseconds() <= recent_pure_spin_until_time_.nanoseconds())
    {
      post_spin_realign_active_ = true;
      post_spin_realign_armed_ = false;
      post_spin_realign_sign_ = (last_pure_spin_sign_ == 0) ? sign(w) : last_pure_spin_sign_;
      post_spin_realign_spin_w_abs_ = last_pure_spin_abs_w_;
      post_spin_realign_start_time_ = now;
      post_spin_realign_until_time_ =
        now + rclcpp::Duration::from_seconds(post_spin_realign_support_duration_s_);
      w = 0.0;
      boost_active_ = false;
      turn_settle_active_ = false;
      RCLCPP_INFO(
        get_logger(),
        "Post-spin support engaged: prior_w=%.2f rad/s, sign=%d, duration=%.2fs, linear_cap=%.2fm/s",
        post_spin_realign_spin_w_abs_, post_spin_realign_sign_,
        post_spin_realign_support_duration_s_, post_spin_realign_linear_cap_m_s_);
    } else if (!requested_post_spin_forward && post_spin_realign_active_) {
      post_spin_realign_active_ = false;
      RCLCPP_INFO(
        get_logger(),
        "Post-spin support released early: DWB swung |w| above target (%.2f rad/s)",
        post_spin_realign_target_w_threshold_rad_s_);
    }

    if (turn_settle_enable_ && requested_straight_motion && !post_spin_realign_active_) {
      const bool recent_turn = now.nanoseconds() <= recent_turn_until_time_.nanoseconds();
      const bool transitioning_from_turn =
        recent_turn ||
        std::abs(target_w_) >= turn_settle_start_w_threshold_rad_s_ ||
        std::abs(current_w_) >= turn_settle_start_w_threshold_rad_s_;

      w = 0.0;

      if (transitioning_from_turn && !turn_settle_active_) {
        turn_settle_active_ = true;
        turn_settle_min_until_time_ =
          now + rclcpp::Duration::from_seconds(turn_settle_min_hold_s_);
        turn_settle_until_time_ =
          now + rclcpp::Duration::from_seconds(turn_settle_timeout_s_);
        boost_active_ = false;
      }
    } else if (turn_settle_active_) {
      turn_settle_active_ = false;
    }

    const bool pure_spin = (std::abs(v) < pure_spin_v_threshold_ && std::abs(w) > 0.0);

    if (pure_spin) {
      const double s = sign(w);

      if (min_pure_spin_enable_ && std::abs(w) < min_pure_spin_angular_rad_s_) {
        w = s * min_pure_spin_angular_rad_s_;
        w = clamp(w, -max_angular_rad_s_, max_angular_rad_s_);
      }

      const bool starting_from_rest = std::abs(current_w_) < 0.05;
      const bool reversing_direction = (target_w_ * w) < 0.0;

      if (pure_spin_start_boost_enable_ && (starting_from_rest || reversing_direction)) {
        boost_active_ = true;
        boost_sign_ = s;
        boost_until_time_ =
          now + rclcpp::Duration::from_seconds(pure_spin_start_boost_duration_s_);
      }
    }

    target_v_ = v;
    target_w_ = w;

    if (debug_) {
      RCLCPP_INFO(
        this->get_logger(),
        "input(v=%.3f,w=%.3f) target(v=%.3f,w=%.3f)",
        msg->linear.x,
        msg->angular.z,
        target_v_,
        target_w_);
    }
  }

  double smoothedTurnSettleTargetV(const rclcpp::Time & now) const
  {
    const double target_abs = std::abs(target_v_);
    if (target_abs <= turn_settle_linear_cap_m_s_) {
      return target_v_;
    }

    double allowed_abs = turn_settle_linear_cap_m_s_;
    if (now.nanoseconds() > turn_settle_min_until_time_.nanoseconds()) {
      const double elapsed_after_hold =
        (now - turn_settle_min_until_time_).nanoseconds() * 1e-9;
      const double blend_duration =
        std::max(0.05, turn_settle_timeout_s_ - turn_settle_min_hold_s_);
      const double blend = clamp(elapsed_after_hold / blend_duration, 0.0, 1.0);
      allowed_abs =
        turn_settle_linear_cap_m_s_ +
        ((target_abs - turn_settle_linear_cap_m_s_) * blend);
    }

    return sign(target_v_) * std::min(target_abs, allowed_abs);
  }

  void timerCallback()
  {
    const auto now = this->now();

    double dt = (now - last_update_time_).nanoseconds() * 1e-9;
    last_update_time_ = now;

    if (dt <= 0.0 || dt > 1.0) {
      dt = 1.0 / output_rate_hz_;
    }

    if (have_cmd_) {
      const double elapsed_cmd = (now - last_cmd_time_).nanoseconds() * 1e-9;
      if (elapsed_cmd > cmd_timeout_s_) {
        target_v_ = 0.0;
        target_w_ = 0.0;
        boost_active_ = false;
        turn_settle_active_ = false;
        post_spin_realign_active_ = false;
        rtfa_active_ = false;
        rtfa_armed_ = false;

        if (immediate_stop_) {
          current_v_ = 0.0;
          current_w_ = 0.0;
          publishCurrent(true);
          return;
        }
      }
    }

    double desired_w = target_w_;
    double desired_v = target_v_;

    if (boost_active_) {
      if (now.nanoseconds() < boost_until_time_.nanoseconds() && std::abs(target_w_) > 0.0) {
        const double boosted_abs =
          std::max(std::abs(target_w_), pure_spin_start_boost_angular_rad_s_);
        desired_w = boost_sign_ * clamp(boosted_abs, 0.0, max_angular_rad_s_);
      } else {
        boost_active_ = false;
      }
    }

    const double max_dv = max_linear_accel_m_s2_ * dt;
    const double max_dw = max_angular_accel_rad_s2_ * dt;

    if (turn_settle_active_) {
      const bool timed_out = now.nanoseconds() >= turn_settle_until_time_.nanoseconds();
      const bool min_hold_done =
        now.nanoseconds() >= turn_settle_min_until_time_.nanoseconds();
      const bool turn_settled =
        min_hold_done && std::abs(current_w_) <= turn_settle_done_w_threshold_rad_s_;
      const bool still_requesting_straight =
        std::abs(target_v_) >= turn_settle_forward_v_threshold_m_s_ &&
        std::abs(target_w_) <= turn_settle_target_w_threshold_rad_s_;

      if (timed_out || !still_requesting_straight) {
        turn_settle_active_ = false;
      } else {
        desired_v = smoothedTurnSettleTargetV(now);
        if (turn_settled) {
          desired_w = 0.0;
        }
      }
    }

    if (post_spin_realign_active_) {
      const bool timed_out = now.nanoseconds() >= post_spin_realign_until_time_.nanoseconds();
      const bool still_requesting_forward =
        std::abs(target_v_) >= post_spin_realign_forward_v_threshold_m_s_ &&
        std::abs(target_w_) <= post_spin_realign_target_w_threshold_rad_s_;

      // Closed-loop overshoot detection: the assist drives chassis in
      // counter-spin direction. Once EKF shows chassis actually rotating
      // that way fast enough, caster has engaged — further open-loop yaw
      // just becomes a new disturbance in the opposite direction.
      bool overshoot_kill = false;
      if (
        post_spin_realign_overshoot_kill_enable_ &&
        have_odom_ &&
        std::abs(current_v_) >= post_spin_realign_overshoot_kill_min_v_m_s_)
      {
        const double odom_age = (now - last_odom_time_).nanoseconds() * 1e-9;
        if (odom_age <= yaw_stabilization_odom_stale_sec_) {
          const double counter_sign = -static_cast<double>(post_spin_realign_sign_);
          if (
            sign(actual_yaw_rate_) == counter_sign &&
            std::abs(actual_yaw_rate_) >= post_spin_realign_overshoot_kill_w_rad_s_)
          {
            overshoot_kill = true;
          }
        }
      }

      if (timed_out || !still_requesting_forward || overshoot_kill) {
        post_spin_realign_active_ = false;
        const double elapsed_s =
          (now - post_spin_realign_start_time_).nanoseconds() * 1e-9;
        const char * reason = "duration_complete";
        if (overshoot_kill) {
          reason = "overshoot_detected";
        } else if (!still_requesting_forward) {
          reason = "forward_request_lost";
        }
        RCLCPP_INFO(
          get_logger(),
          "Post-spin support released: reason=%s, elapsed=%.2fs, actual_w=%.3f",
          reason, elapsed_s, actual_yaw_rate_);
      } else {
        if (std::abs(desired_v) > post_spin_realign_linear_cap_m_s_) {
          desired_v = sign(desired_v) * post_spin_realign_linear_cap_m_s_;
        }

        const double elapsed =
          (now - post_spin_realign_start_time_).nanoseconds() * 1e-9;
        const double progress =
          post_spin_realign_support_duration_s_ > 0.0
            ? clamp(elapsed / post_spin_realign_support_duration_s_, 0.0, 1.0)
            : 1.0;
        const double ratio =
          post_spin_realign_support_start_ratio_ +
          ((post_spin_realign_support_end_ratio_ - post_spin_realign_support_start_ratio_) *
            progress);
        const double correction =
          clamp(post_spin_realign_spin_w_abs_ * ratio, 0.0, max_angular_rad_s_);

        desired_w = -static_cast<double>(post_spin_realign_sign_) * correction;
      }
    }

    // Rotation-to-Forward Assist (RTFA). On the rising-edge fire detected
    // in processIncomingCmd, ramp linear vx from a low floor toward
    // commanded over x_ramp_duration and inject a growing counter-yaw
    // (same sign as last spin) over z_assist_duration. Counter injection
    // only happens when Nav2 is roughly going straight — when Nav2 wants
    // a real turn, the assist aborts and trusts the planner.
    if (rtfa_active_) {
      const double elapsed = (now - rtfa_fire_time_).nanoseconds() * 1e-9;
      const double x_ramp_done = rotation_to_forward_assist_x_ramp_duration_s_;
      const double z_assist_done = rotation_to_forward_assist_z_assist_duration_s_;
      const double max_window = std::max(x_ramp_done, z_assist_done);

      const bool forward_lost =
        std::abs(target_v_) < rotation_to_forward_assist_idle_v_threshold_m_s_;
      const bool nav_turn_committed =
        std::abs(target_w_) >= rotation_to_forward_assist_abort_nav_w_rad_s_;

      bool overshoot_kill = false;
      if (have_odom_) {
        const double odom_age = (now - last_odom_time_).nanoseconds() * 1e-9;
        if (odom_age <= yaw_stabilization_odom_stale_sec_) {
          const double counter_sign = static_cast<double>(rtfa_last_spin_sign_);
          if (
            sign(actual_yaw_rate_) == counter_sign &&
            std::abs(actual_yaw_rate_) >=
              rotation_to_forward_assist_overshoot_kill_w_rad_s_)
          {
            overshoot_kill = true;
          }
        }
      }

      if (elapsed >= max_window || forward_lost || nav_turn_committed || overshoot_kill) {
        rtfa_active_ = false;
        const char * reason = "duration_complete";
        if (overshoot_kill) {
          reason = "overshoot_detected";
        } else if (nav_turn_committed) {
          reason = "nav_turn_committed";
        } else if (forward_lost) {
          reason = "forward_lost";
        }
        RCLCPP_INFO(
          get_logger(),
          "RTFA released: reason=%s, elapsed=%.2fs, actual_w=%.3f",
          reason, elapsed, actual_yaw_rate_);
      } else {
        if (
          elapsed < x_ramp_done &&
          target_v_ > rotation_to_forward_assist_x_ramp_start_m_s_)
        {
          const double progress = clamp(elapsed / x_ramp_done, 0.0, 1.0);
          const double ramp_v =
            rotation_to_forward_assist_x_ramp_start_m_s_ +
            ((target_v_ - rotation_to_forward_assist_x_ramp_start_m_s_) * progress);
          desired_v = std::min(ramp_v, target_v_);
        }

        if (
          elapsed < z_assist_done &&
          std::abs(target_w_) < rotation_to_forward_assist_z_inject_max_nav_w_rad_s_)
        {
          const double progress = clamp(elapsed / z_assist_done, 0.0, 1.0);
          const double ratio =
            rotation_to_forward_assist_z_assist_start_ratio_ +
            ((rotation_to_forward_assist_z_assist_end_ratio_ -
              rotation_to_forward_assist_z_assist_start_ratio_) * progress);
          double counter =
            static_cast<double>(rtfa_last_spin_sign_) * rtfa_last_spin_abs_w_ * ratio;
          counter = clamp(
            counter,
            -rotation_to_forward_assist_z_assist_max_rad_s_,
            rotation_to_forward_assist_z_assist_max_rad_s_);
          desired_w = target_w_ + counter;
          desired_w = clamp(desired_w, -max_angular_rad_s_, max_angular_rad_s_);
        }
      }
    }

    // Soft start: when transitioning from rest to forward motion, cap
    // linear velocity briefly so the caster wheel has time to swivel
    // into alignment without overpowering the yaw stabilizer. Skipped
    // while RTFA is active — RTFA owns the linear ramp in that case.
    if (!rtfa_active_) {
      const bool effectively_at_rest =
        std::abs(current_v_) < soft_start_idle_threshold_m_s_;
      const bool wants_forward =
        std::abs(target_v_) >= soft_start_idle_threshold_m_s_;
      if (
        soft_start_enable_ &&
        !soft_start_active_ &&
        effectively_at_rest &&
        wants_forward)
      {
        soft_start_active_ = true;
        soft_start_until_time_ =
          now + rclcpp::Duration::from_seconds(soft_start_duration_s_);
        RCLCPP_INFO(
          get_logger(),
          "Soft start engaged: cap=%.2fm/s for %.2fs",
          soft_start_linear_cap_m_s_, soft_start_duration_s_);
      }
      if (soft_start_active_) {
        const bool timed_out =
          now.nanoseconds() >= soft_start_until_time_.nanoseconds();
        if (timed_out || !wants_forward) {
          soft_start_active_ = false;
          RCLCPP_INFO(
            get_logger(),
            "Soft start released: reason=%s",
            timed_out ? "duration_complete" : "stopped");
        } else if (std::abs(desired_v) > soft_start_linear_cap_m_s_) {
          desired_v = sign(desired_v) * soft_start_linear_cap_m_s_;
        }
      }
    }

    const double obstacle_scale = scanSpeedScale(now);
    if (desired_v > 0.0) {
      if (shouldStopForwardForScan(now)) {
        desired_v = 0.0;
        if (current_v_ > 0.0) {
          current_v_ = 0.0;
        }
      } else {
        desired_v *= obstacle_scale;
      }
    }
    desired_w *= angularScaleForScan(obstacle_scale);

    // Closed-loop yaw rate stabilization (final layer before ramp).
    // Adds a proportional correction so chassis actually achieves the
    // commanded yaw rate despite caster drag and asymmetric friction.
    // Disabled below min_forward_v threshold: at near-zero motion the
    // actual yaw signal is dominated by sensor noise and corrections
    // can't physically influence the chassis anyway.
    if (
      yaw_stabilization_enable_ &&
      have_odom_ &&
      std::abs(current_v_) >= yaw_stabilization_min_forward_v_m_s_)
    {
      const double odom_age = (now - last_odom_time_).nanoseconds() * 1e-9;
      if (odom_age <= yaw_stabilization_odom_stale_sec_) {
        const double yaw_error = desired_w - actual_yaw_rate_;
        double correction = yaw_stabilization_kp_ * yaw_error;
        if (correction > yaw_stabilization_max_correction_rad_s_) {
          correction = yaw_stabilization_max_correction_rad_s_;
        } else if (correction < -yaw_stabilization_max_correction_rad_s_) {
          correction = -yaw_stabilization_max_correction_rad_s_;
        }
        if (std::abs(correction) >= yaw_stabilization_log_threshold_rad_s_) {
          RCLCPP_DEBUG(
            get_logger(),
            "Yaw stab: cmd_w=%.3f actual_w=%.3f err=%.3f corr=%.3f",
            desired_w, actual_yaw_rate_, yaw_error, correction);
        }
        desired_w += correction;
        if (desired_w > max_angular_rad_s_) {
          desired_w = max_angular_rad_s_;
        } else if (desired_w < -max_angular_rad_s_) {
          desired_w = -max_angular_rad_s_;
        }
      }
    }

    current_v_ = stepToward(current_v_, desired_v, max_dv);
    current_w_ = stepToward(current_w_, desired_w, max_dw);

    publishCurrent(false);
  }

  bool shouldPublishCurrent(const rclcpp::Time & now) const
  {
    if (!have_published_) {
      return true;
    }

    if (std::abs(current_v_ - last_published_v_) >= publish_linear_epsilon_m_s_) {
      return true;
    }

    if (std::abs(current_w_ - last_published_w_) >= publish_angular_epsilon_rad_s_) {
      return true;
    }

    const double elapsed = (now - last_publish_time_).nanoseconds() * 1e-9;
    return elapsed >= publish_keepalive_s_;
  }

  void publishCurrent(bool force)
  {
    const auto now = this->now();
    if (!force && !shouldPublishCurrent(now)) {
      return;
    }

    Twist out;
    out.linear.x = current_v_;
    out.linear.y = 0.0;
    out.linear.z = 0.0;
    out.angular.x = 0.0;
    out.angular.y = 0.0;
    out.angular.z = current_w_;
    pub_->publish(out);

    last_published_v_ = current_v_;
    last_published_w_ = current_w_;
    last_publish_time_ = now;
    have_published_ = true;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string scan_topic_;

  bool scan_speed_scaling_enable_;
  double scan_speed_scaling_stale_sec_;
  double scan_speed_scaling_sector_deg_;
  double scan_speed_scaling_open_clearance_m_;
  double scan_speed_scaling_medium_clearance_m_;
  double scan_speed_scaling_tight_clearance_m_;
  double scan_speed_scaling_danger_clearance_m_;
  double scan_speed_scaling_stop_clearance_m_;
  double scan_speed_scaling_medium_scale_;
  double scan_speed_scaling_tight_scale_;
  double scan_speed_scaling_danger_scale_;
  bool scan_speed_scaling_scale_angular_;
  double scan_speed_scaling_min_angular_scale_;
  bool scan_speed_scaling_stop_forward_;
  double max_linear_m_s_;
  double max_angular_rad_s_;
  double max_linear_accel_m_s2_;
  double max_angular_accel_rad_s2_;
  double linear_deadband_m_s_;
  double angular_deadband_rad_s_;
  double pure_spin_v_threshold_;
  bool min_pure_spin_enable_;
  double min_pure_spin_angular_rad_s_;
  bool pure_spin_start_boost_enable_;
  double pure_spin_start_boost_angular_rad_s_;
  double pure_spin_start_boost_duration_s_;
  bool turn_settle_enable_;
  double turn_settle_forward_v_threshold_m_s_;
  double turn_settle_start_w_threshold_rad_s_;
  double turn_settle_target_w_threshold_rad_s_;
  double turn_settle_done_w_threshold_rad_s_;
  double turn_settle_linear_cap_m_s_;
  double turn_settle_min_hold_s_;
  double turn_settle_timeout_s_;
  bool post_spin_realign_enable_;
  double post_spin_realign_recent_window_s_;
  double post_spin_realign_forward_v_threshold_m_s_;
  double post_spin_realign_spin_w_threshold_rad_s_;
  double post_spin_realign_target_w_threshold_rad_s_;
  double post_spin_realign_linear_cap_m_s_;
  double post_spin_realign_support_start_ratio_;
  double post_spin_realign_support_end_ratio_;
  double post_spin_realign_support_duration_s_;
  bool post_spin_realign_overshoot_kill_enable_{true};
  double post_spin_realign_overshoot_kill_w_rad_s_{0.10};
  double post_spin_realign_overshoot_kill_min_v_m_s_{0.01};
  double cmd_timeout_s_;
  double output_rate_hz_;
  double publish_keepalive_s_;
  double publish_linear_epsilon_m_s_;
  double publish_angular_epsilon_rad_s_;
  bool immediate_stop_;
  bool debug_;

  double target_v_{0.0};
  double target_w_{0.0};
  double current_v_{0.0};
  double current_w_{0.0};

  bool have_cmd_{false};
  bool have_scan_{false};
  bool have_published_{false};
  bool boost_active_{false};
  bool turn_settle_active_{false};
  bool post_spin_realign_active_{false};
  bool post_spin_realign_armed_{false};
  int last_pure_spin_sign_{0};
  int post_spin_realign_sign_{0};
  double last_pure_spin_abs_w_{0.0};
  double post_spin_realign_spin_w_abs_{0.0};
  double boost_sign_{1.0};
  double last_published_v_{0.0};
  double last_published_w_{0.0};
  double latest_front_clearance_m_{std::numeric_limits<double>::infinity()};

  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_scan_time_;
  rclcpp::Time last_update_time_;
  rclcpp::Time last_publish_time_;
  rclcpp::Time boost_until_time_;
  rclcpp::Time recent_turn_until_time_;
  rclcpp::Time turn_settle_min_until_time_;
  rclcpp::Time turn_settle_until_time_;
  rclcpp::Time recent_pure_spin_until_time_;
  rclcpp::Time post_spin_realign_start_time_;
  rclcpp::Time post_spin_realign_until_time_;

  rclcpp::Subscription<Twist>::SharedPtr sub_;
  rclcpp::Subscription<Twist>::SharedPtr supervisor_sub_;
  rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string supervisor_input_topic_;
  bool supervisor_priority_enable_{false};
  double supervisor_priority_timeout_s_{0.20};
  bool have_supervisor_cmd_{false};
  bool supervisor_priority_logged_{false};
  rclcpp::Time last_supervisor_cmd_time_;

  std::string odom_input_topic_;
  bool yaw_stabilization_enable_{false};
  double yaw_stabilization_kp_{1.5};
  double yaw_stabilization_max_correction_rad_s_{0.30};
  double yaw_stabilization_odom_stale_sec_{0.30};
  double yaw_stabilization_log_threshold_rad_s_{0.15};
  double yaw_stabilization_min_forward_v_m_s_{0.03};
  double actual_yaw_rate_{0.0};
  bool have_odom_{false};
  rclcpp::Time last_odom_time_;

  bool soft_start_enable_{true};
  double soft_start_duration_s_{0.6};
  double soft_start_linear_cap_m_s_{0.05};
  double soft_start_idle_threshold_m_s_{0.02};
  bool soft_start_active_{false};
  rclcpp::Time soft_start_until_time_;

  bool rotation_to_forward_assist_enable_{false};
  double rotation_to_forward_assist_spin_w_threshold_rad_s_{0.12};
  double rotation_to_forward_assist_idle_v_threshold_m_s_{0.02};
  double rotation_to_forward_assist_fire_v_threshold_m_s_{0.02};
  double rotation_to_forward_assist_arm_min_duration_s_{0.10};
  double rotation_to_forward_assist_arm_recent_window_s_{5.0};
  double rotation_to_forward_assist_x_ramp_duration_s_{2.0};
  double rotation_to_forward_assist_x_ramp_start_m_s_{0.06};
  double rotation_to_forward_assist_z_assist_duration_s_{1.5};
  double rotation_to_forward_assist_z_assist_start_ratio_{0.4};
  double rotation_to_forward_assist_z_assist_end_ratio_{0.8};
  double rotation_to_forward_assist_z_assist_max_rad_s_{0.25};
  double rotation_to_forward_assist_z_inject_max_nav_w_rad_s_{0.08};
  double rotation_to_forward_assist_abort_nav_w_rad_s_{0.20};
  double rotation_to_forward_assist_overshoot_kill_w_rad_s_{0.10};
  bool rtfa_armed_{false};
  bool rtfa_active_{false};
  int rtfa_last_spin_sign_{0};
  double rtfa_last_spin_abs_w_{0.0};
  rclcpp::Time rtfa_arm_start_time_;
  rclcpp::Time rtfa_fire_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityManager>());
  rclcpp::shutdown();
  return 0;
}
