#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import json
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32, String


DEFAULT_ODOM_TOPIC = "/zj_humanoid/navigation/odom_info"
DEFAULT_JOY_CTRL_TOPIC = "/jzhw/joy_ctrl"
DEFAULT_JOY_TOPIC = "/jzhw/joy"
DEFAULT_SPEED_LEVEL_TOPIC = "/jzhw/joy3_ctrl/speed_level"
DEFAULT_VISION_TOPIC = "/yolo_vision/wall_angle"

# DEFAULT_TARGET_X = 0.37883
# DEFAULT_TARGET_Y = 2.97813
# DEFAULT_TARGET_YAW = 1.69638

DEFAULT_TARGET_X = 2.95848
DEFAULT_TARGET_Y = -0.85498
DEFAULT_TARGET_YAW = 1.38934


def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)


def approach(value, target, max_delta):
    if value < target:
        return min(value + max_delta, target)
    if value > target:
        return max(value - max_delta, target)
    return target


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def make_joy(axis_turn, axis_forward, enable=True):
    msg = Joy()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "joystick"
    msg.axes = [0.0] * 8
    msg.buttons = [0] * 15

    # Keep the mapping consistent with move/cmd.py.
    msg.axes[7] = float(axis_forward)
    msg.axes[0] = float(axis_turn)
    msg.buttons[0] = 1 if enable else 0
    return msg


def make_twist(linear_x, angular_z):
    msg = Twist()
    msg.linear.x = float(linear_x)
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = float(angular_z)
    return msg


class PIDController:
    def __init__(
        self,
        kp,
        ki,
        kd,
        output_limit,
        integral_limit,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = abs(output_limit)
        self.integral_limit = abs(integral_limit)
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_error = None

    def update(self, error, dt):
        if dt <= 0.0:
            dt = 1e-3

        self.integral = clamp(
            self.integral + error * dt,
            -self.integral_limit,
            self.integral_limit,
        )
        derivative = 0.0
        if self.previous_error is not None:
            derivative = (error - self.previous_error) / dt
        self.previous_error = error

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )
        return clamp(output, -self.output_limit, self.output_limit)


class TwiceMoveCorrector:
    ALIGN_TO_PATH = "align_to_path"
    DRIVE_TO_TARGET = "drive_to_target"
    ALIGN_FINAL_YAW = "align_final_yaw"
    VISION_FINAL_CORRECT = "vision_final_correct"
    VISION_NUDGE_TURN_OUT = "turn_out"
    VISION_NUDGE_FORWARD = "forward"
    VISION_NUDGE_TURN_BACK = "turn_back"
    VISION_NUDGE_REVERSE = "reverse"

    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", DEFAULT_ODOM_TOPIC)
        self.joy_ctrl_topic = rospy.get_param(
            "~joy_ctrl_topic", DEFAULT_JOY_CTRL_TOPIC
        )
        self.joy_topic = rospy.get_param("~joy_topic", DEFAULT_JOY_TOPIC)
        self.speed_level_topic = rospy.get_param(
            "~speed_level_topic", DEFAULT_SPEED_LEVEL_TOPIC
        )
        self.vision_topic = rospy.get_param(
            "~vision_topic", DEFAULT_VISION_TOPIC
        )

        self.target_x = float(rospy.get_param("~target_x", DEFAULT_TARGET_X))
        self.target_y = float(rospy.get_param("~target_y", DEFAULT_TARGET_Y))
        self.target_yaw = float(
            rospy.get_param("~target_yaw", DEFAULT_TARGET_YAW)
        )

        self.control_hz = float(rospy.get_param("~control_hz", 50.0))
        self.min_speed_level = int(rospy.get_param("~min_speed_level", 1))
        self.max_speed_level = int(rospy.get_param("~max_speed_level", 5))
        self.speed_level_steps = int(rospy.get_param("~speed_level_steps", 0))
        self.speed_level = int(
            clamp(
                int(rospy.get_param("~speed_level", 2)) + self.speed_level_steps,
                self.min_speed_level,
                self.max_speed_level,
            )
        )
        self.position_tolerance = float(
            rospy.get_param("~position_tolerance", 0.02)
        )
        self.path_heading_tolerance = float(
            rospy.get_param("~path_heading_tolerance", 0.0698)
        )
        self.coarse_heading_tolerance = float(
            rospy.get_param("~coarse_heading_tolerance", 0.6109)
        )
        self.final_heading_tolerance = float(
            rospy.get_param("~final_heading_tolerance", 0.0175)
        )
        self.realign_threshold = float(
            rospy.get_param("~realign_threshold", 0.2618)
        )
        self.coarse_realign_threshold = float(
            rospy.get_param("~coarse_realign_threshold", 1.0472)
        )
        self.near_target_distance = float(
            rospy.get_param("~near_target_distance", 0.15)
        )
        self.near_target_realign_threshold = float(
            rospy.get_param("~near_target_realign_threshold", 0.6109)
        )
        self.final_yaw_blend_distance = float(
            rospy.get_param("~final_yaw_blend_distance", 0.60)
        )
        self.max_final_yaw_drive_weight = float(
            rospy.get_param("~max_final_yaw_drive_weight", 0.85)
        )
        self.min_path_drive_weight = float(
            rospy.get_param("~min_path_drive_weight", 0.25)
        )
        self.direction_switch_margin = float(
            rospy.get_param("~direction_switch_margin", 0.2618)
        )

        self.min_linear_speed = float(rospy.get_param("~min_linear_speed", 0.0))
        self.max_linear_speed = float(rospy.get_param("~max_linear_speed", 0.85))
        self.min_angular_speed = float(rospy.get_param("~min_angular_speed", 0.0))
        self.max_angular_speed = float(rospy.get_param("~max_angular_speed", 0.5))
        self.linear_step = float(rospy.get_param("~linear_step", 0.02))
        self.angular_step = float(rospy.get_param("~angular_step", 0.02))
        self.linear_speed_steps = int(rospy.get_param("~linear_speed_steps", 0))
        self.angular_speed_steps = int(rospy.get_param("~angular_speed_steps", 0))
        self.linear_speed = clamp(
            float(rospy.get_param("~linear_speed", 0.08))
            + self.linear_speed_steps * self.linear_step,
            self.min_linear_speed,
            self.max_linear_speed,
        )
        self.angular_speed = clamp(
            float(rospy.get_param("~angular_speed", 0.15))
            + self.angular_speed_steps * self.angular_step,
            self.min_angular_speed,
            self.max_angular_speed,
        )
        self.linear_acc = abs(float(rospy.get_param("~linear_acc", 0.5)))
        self.angular_acc = abs(float(rospy.get_param("~angular_acc", 0.6)))
        self.soft_start_duration = max(
            0.0,
            float(rospy.get_param("~soft_start_duration", 0.8)),
        )
        self.phase_stop_smooth = bool(
            rospy.get_param("~phase_stop_smooth", True)
        )
        self.publish_enable_joy = bool(
            rospy.get_param("~publish_enable_joy", False)
        )

        self.turn_kp = float(rospy.get_param("~turn_kp", 0.8))
        self.turn_ki = float(rospy.get_param("~turn_ki", 0.0))
        self.turn_kd = float(rospy.get_param("~turn_kd", 0.05))
        self.turn_integral_limit = float(
            rospy.get_param("~turn_integral_limit", 0.6)
        )
        self.max_turn_axis = self.angular_speed
        self.min_turn_axis = min(
            abs(float(rospy.get_param("~min_turn_speed", 0.08))),
            self.max_turn_axis,
        )
        self.drive_turn_kp = float(rospy.get_param("~drive_turn_kp", 0.8))
        self.drive_turn_ki = float(rospy.get_param("~drive_turn_ki", 0.0))
        self.drive_turn_kd = float(rospy.get_param("~drive_turn_kd", 0.03))
        self.drive_turn_integral_limit = float(
            rospy.get_param("~drive_turn_integral_limit", 0.6)
        )
        self.max_drive_turn_axis = min(
            abs(float(rospy.get_param("~max_drive_turn_speed", 0.15))),
            self.angular_speed,
        )
        self.drive_kp = float(rospy.get_param("~drive_kp", 1.2))
        self.drive_ki = float(rospy.get_param("~drive_ki", 0.0))
        self.drive_kd = float(rospy.get_param("~drive_kd", 0.08))
        self.drive_integral_limit = float(
            rospy.get_param("~drive_integral_limit", 0.5)
        )
        self.max_forward_axis = self.linear_speed
        self.min_drive_axis = min(
            abs(float(rospy.get_param("~min_drive_speed", 0.03))),
            self.max_forward_axis,
        )
        self.creep_drive_axis = min(
            abs(float(rospy.get_param("~creep_drive_speed", 0.04))),
            self.max_forward_axis,
        )
        self.creep_drive_axis = max(self.creep_drive_axis, self.min_drive_axis)
        self.allow_reverse = bool(rospy.get_param("~allow_reverse", True))

        self.enable_vision_final = bool(
            rospy.get_param("~enable_vision_final", True)
        )
        self.vision_handoff_distance = float(
            rospy.get_param("~vision_handoff_distance", 0.70)
        )
        self.vision_timeout = float(rospy.get_param("~vision_timeout", 0.5))
        self.vision_min_seen_count = int(
            rospy.get_param("~vision_min_seen_count", 3)
        )
        self.vision_filter_alpha = clamp(
            float(rospy.get_param("~vision_filter_alpha", 0.35)),
            0.0,
            1.0,
        )
        self.vision_target_x = float(
            rospy.get_param("~vision_target_x", 0.9239)
        )
        self.vision_target_y = float(
            rospy.get_param("~vision_target_y", 0.0341)
        )
        self.vision_target_yaw = float(
            rospy.get_param("~vision_target_yaw", 0.226)
        )
        self.vision_x_tolerance = float(
            rospy.get_param("~vision_x_tolerance", 0.005)
        )
        self.vision_y_tolerance = float(
            rospy.get_param("~vision_y_tolerance", 0.005)
        )
        self.vision_yaw_tolerance = float(
            rospy.get_param("~vision_yaw_tolerance", 0.01)
        )
        self.vision_yaw_slow_zone = float(
            rospy.get_param("~vision_yaw_slow_zone", 0.1047)
        )
        self.vision_linear_speed = min(
            abs(float(rospy.get_param("~vision_linear_speed", 0.03))),
            self.linear_speed,
        )
        self.vision_angular_speed = min(
            abs(float(rospy.get_param("~vision_angular_speed", 0.02))),
            self.angular_speed,
        )
        self.min_vision_linear_speed = min(
            abs(float(rospy.get_param("~min_vision_linear_speed", 0.005))),
            self.vision_linear_speed,
        )
        self.min_vision_angular_speed = min(
            abs(float(rospy.get_param("~min_vision_angular_speed", 0.005))),
            self.vision_angular_speed,
        )
        self.vision_x_kp = float(rospy.get_param("~vision_x_kp", 0.35))
        self.vision_x_ki = float(rospy.get_param("~vision_x_ki", 0.0))
        self.vision_x_kd = float(rospy.get_param("~vision_x_kd", 0.03))
        self.vision_y_kp = float(rospy.get_param("~vision_y_kp", 0.8))
        self.vision_y_ki = float(rospy.get_param("~vision_y_ki", 0.0))
        self.vision_y_kd = float(rospy.get_param("~vision_y_kd", 0.02))
        self.vision_yaw_kp = float(rospy.get_param("~vision_yaw_kp", 0.8))
        self.vision_yaw_ki = float(rospy.get_param("~vision_yaw_ki", 0.0))
        self.vision_yaw_kd = float(rospy.get_param("~vision_yaw_kd", 0.02))
        self.vision_x_integral_limit = float(
            rospy.get_param("~vision_x_integral_limit", 0.4)
        )
        self.vision_turn_integral_limit = float(
            rospy.get_param("~vision_turn_integral_limit", 0.5)
        )
        self.vision_linear_sign = float(
            rospy.get_param("~vision_linear_sign", 1.0)
        )
        self.vision_lateral_sign = float(
            rospy.get_param("~vision_lateral_sign", 1.0)
        )
        self.vision_yaw_sign = float(rospy.get_param("~vision_yaw_sign", 1.0))
        self.vision_y_projection_enable = bool(
            rospy.get_param("~vision_y_projection_enable", True)
        )
        self.vision_y_projection_weight = clamp(
            float(rospy.get_param("~vision_y_projection_weight", 1.0)),
            0.0,
            1.0,
        )
        self.vision_y_projection_sign = float(
            rospy.get_param("~vision_y_projection_sign", 1.0)
        )
        self.vision_y_projection_yaw_limit = abs(
            float(rospy.get_param("~vision_y_projection_yaw_limit", 0.35))
        )
        self.vision_lateral_arc_enable = bool(
            rospy.get_param("~vision_lateral_arc_enable", True)
        )
        self.vision_lateral_arc_speed = min(
            abs(float(rospy.get_param("~vision_lateral_arc_speed", 0.025))),
            self.vision_linear_speed,
        )
        self.min_vision_lateral_arc_speed = min(
            abs(float(rospy.get_param("~min_vision_lateral_arc_speed", 0.008))),
            self.vision_lateral_arc_speed,
        )
        self.vision_lateral_arc_slow_zone = max(
            self.vision_y_tolerance,
            abs(float(rospy.get_param("~vision_lateral_arc_slow_zone", 0.12))),
        )
        self.vision_lateral_arc_linear_sign = float(
            rospy.get_param("~vision_lateral_arc_linear_sign", 1.0)
        )
        self.vision_lateral_arc_max_yaw_error = float(
            rospy.get_param("~vision_lateral_arc_max_yaw_error", 0.2094)
        )
        self.vision_lateral_nudge_enable = bool(
            rospy.get_param("~vision_lateral_nudge_enable", True)
        )
        self.vision_lateral_nudge_x_gate = max(
            self.vision_x_tolerance,
            abs(float(rospy.get_param("~vision_lateral_nudge_x_gate", 0.02))),
        )
        self.vision_lateral_nudge_min_angle = abs(
            float(rospy.get_param("~vision_lateral_nudge_min_angle", 0.025))
        )
        self.vision_lateral_nudge_max_angle = abs(
            float(rospy.get_param("~vision_lateral_nudge_max_angle", 0.06))
        )
        self.vision_lateral_nudge_angle_kp = abs(
            float(rospy.get_param("~vision_lateral_nudge_angle_kp", 1.2))
        )
        self.vision_lateral_nudge_angle_tolerance = abs(
            float(
                rospy.get_param("~vision_lateral_nudge_angle_tolerance", 0.01)
            )
        )
        self.vision_lateral_nudge_forward_x = abs(
            float(rospy.get_param("~vision_lateral_nudge_forward_x", 0.015))
        )
        self.vision_lateral_nudge_speed = min(
            abs(float(rospy.get_param("~vision_lateral_nudge_speed", 0.012))),
            self.vision_linear_speed,
        )
        self.vision_lateral_nudge_turn_sign = float(
            rospy.get_param("~vision_lateral_nudge_turn_sign", -1.0)
        )
        self.vision_lateral_nudge_yaw_sign = float(
            rospy.get_param("~vision_lateral_nudge_yaw_sign", -1.0)
        )

        self.settle_cycles = int(rospy.get_param("~settle_cycles", 5))
        self.wait_odom_timeout = float(
            rospy.get_param("~wait_odom_timeout", 10.0)
        )
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 1.0))
        self.total_timeout = float(rospy.get_param("~total_timeout", 240.0))

        self.odom = None
        self.last_odom_time = None
        self.phase = self.ALIGN_TO_PATH
        self.settled_count = 0
        self.control_released = False
        self.current_wz = 0.0
        self.current_vx = 0.0
        self.last_command_time = None
        self.motion_started_time = None
        self.drive_direction = 1.0
        self.vision = None
        self.last_vision_time = None
        self.vision_seen_count = 0
        self.vision_warned_parse_error = False
        self.stop_requested = False
        self.vision_lateral_nudge_step = None
        self.vision_lateral_nudge_direction = 1.0
        self.vision_lateral_nudge_start_x_error = 0.0
        self.vision_lateral_nudge_start_y_error = 0.0
        self.vision_lateral_nudge_target_angle = 0.0

        self.turn_pid = PIDController(
            self.turn_kp,
            self.turn_ki,
            self.turn_kd,
            self.max_turn_axis,
            self.turn_integral_limit,
        )
        self.drive_turn_pid = PIDController(
            self.drive_turn_kp,
            self.drive_turn_ki,
            self.drive_turn_kd,
            self.max_drive_turn_axis,
            self.drive_turn_integral_limit,
        )
        self.drive_pid = PIDController(
            self.drive_kp,
            self.drive_ki,
            self.drive_kd,
            self.max_forward_axis,
            self.drive_integral_limit,
        )
        self.vision_x_pid = PIDController(
            self.vision_x_kp,
            self.vision_x_ki,
            self.vision_x_kd,
            self.vision_linear_speed,
            self.vision_x_integral_limit,
        )
        self.vision_y_pid = PIDController(
            self.vision_y_kp,
            self.vision_y_ki,
            self.vision_y_kd,
            self.vision_angular_speed,
            self.vision_turn_integral_limit,
        )
        self.vision_yaw_pid = PIDController(
            self.vision_yaw_kp,
            self.vision_yaw_ki,
            self.vision_yaw_kd,
            self.vision_angular_speed,
            self.vision_turn_integral_limit,
        )

        self.joy_ctrl_pub = rospy.Publisher(
            self.joy_ctrl_topic,
            Twist,
            queue_size=1,
            latch=False,
            tcp_nodelay=True,
        )
        self.joy_pub = rospy.Publisher(
            self.joy_topic,
            Joy,
            queue_size=1,
            latch=False,
            tcp_nodelay=True,
        )
        self.speed_level_pub = rospy.Publisher(
            self.speed_level_topic,
            Int32,
            queue_size=1,
            latch=True,
        )
        self.odom_sub = rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self.odom_callback,
            queue_size=1,
        )
        self.vision_sub = rospy.Subscriber(
            self.vision_topic,
            String,
            self.vision_callback,
            queue_size=1,
        )

        rospy.on_shutdown(self.release_control)

    def odom_callback(self, msg):
        self.odom = msg
        self.last_odom_time = rospy.Time.now()

    def vision_callback(self, msg):
        try:
            data = json.loads(msg.data)
            vision = {
                "x": float(data["base_x"]),
                "y": float(data["base_y"]),
                "yaw": float(data["yaw_rad"]),
            }
        except (KeyError, TypeError, ValueError) as exc:
            if not self.vision_warned_parse_error:
                rospy.logwarn("Failed to parse vision pose: %s", exc)
                self.vision_warned_parse_error = True
            return

        previous_age = self.vision_age()

        if self.vision is None or self.vision_filter_alpha >= 1.0:
            self.vision = vision
        else:
            alpha = self.vision_filter_alpha
            self.vision = {
                "x": alpha * vision["x"] + (1.0 - alpha) * self.vision["x"],
                "y": alpha * vision["y"] + (1.0 - alpha) * self.vision["y"],
                "yaw": wrap_angle(
                    self.vision["yaw"]
                    + alpha * wrap_angle(vision["yaw"] - self.vision["yaw"])
                ),
            }

        self.last_vision_time = rospy.Time.now()
        if previous_age is None or previous_age > self.vision_timeout:
            self.vision_seen_count = 1
        else:
            self.vision_seen_count = min(self.vision_seen_count + 1, 1000000)

    def current_pose(self):
        pose = self.odom.pose.pose
        return (
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(pose.orientation),
        )

    def set_phase(self, phase):
        if self.phase == phase:
            return
        self.phase = phase
        self.settled_count = 0
        self.reset_pids()
        self.publish_command(0.0, 0.0, smooth=self.phase_stop_smooth)
        rospy.loginfo("Switch phase: %s", phase)

    def reset_pids(self):
        self.turn_pid.reset()
        self.drive_turn_pid.reset()
        self.drive_pid.reset()
        self.vision_x_pid.reset()
        self.vision_y_pid.reset()
        self.vision_yaw_pid.reset()
        self.reset_vision_lateral_nudge()

    def vision_age(self):
        if self.last_vision_time is None:
            return None
        return (rospy.Time.now() - self.last_vision_time).to_sec()

    def vision_is_fresh(self):
        age = self.vision_age()
        return (
            self.enable_vision_final
            and self.vision is not None
            and age is not None
            and age <= self.vision_timeout
            and self.vision_seen_count >= self.vision_min_seen_count
        )

    def should_use_vision(self, distance):
        return (
            self.vision_is_fresh()
            and distance <= self.vision_handoff_distance
        )

    def select_motion_direction(self, path_heading, yaw):
        forward_error = wrap_angle(path_heading - yaw)
        if not self.allow_reverse:
            return 1.0, forward_error

        reverse_error = wrap_angle(path_heading + math.pi - yaw)

        forward_abs = abs(forward_error)
        reverse_abs = abs(reverse_error)
        margin = self.direction_switch_margin

        if (
            self.drive_direction < 0.0
            and forward_abs + margin >= reverse_abs
        ):
            return -1.0, reverse_error
        if (
            self.drive_direction > 0.0
            and reverse_abs + margin >= forward_abs
        ):
            return 1.0, forward_error

        if reverse_abs < forward_abs:
            return -1.0, reverse_error
        return 1.0, forward_error

    def update_motion_direction(self, direction):
        if direction == self.drive_direction:
            return
        self.drive_direction = direction
        self.settled_count = 0
        self.reset_pids()
        rospy.loginfo(
            "Switch drive direction: %s",
            "reverse" if direction < 0.0 else "forward",
        )

    def turn_speed(self, heading_error, dt):
        speed = self.turn_pid.update(heading_error, dt)
        if 0.0 < abs(speed) < self.min_turn_axis:
            speed = math.copysign(self.min_turn_axis, speed)
        return speed

    def publish_command(self, angular_z, linear_x, smooth=True):
        now = rospy.Time.now()
        if self.last_command_time is None:
            dt = 1.0 / max(self.control_hz, 1.0)
        else:
            dt = (now - self.last_command_time).to_sec()
        self.last_command_time = now

        target_wz = clamp(angular_z, -self.angular_speed, self.angular_speed)
        target_vx = clamp(linear_x, -self.linear_speed, self.linear_speed)
        target_is_moving = abs(target_wz) > 0.0 or abs(target_vx) > 0.0

        if target_is_moving:
            if self.motion_started_time is None:
                self.motion_started_time = now
            if self.soft_start_duration > 0.0:
                elapsed = (now - self.motion_started_time).to_sec()
                soft_start_scale = clamp(
                    max(elapsed, dt) / self.soft_start_duration,
                    0.0,
                    1.0,
                )
                target_wz *= soft_start_scale
                target_vx *= soft_start_scale

        if smooth:
            self.current_wz = approach(
                self.current_wz,
                target_wz,
                self.angular_acc * max(dt, 0.0),
            )
            self.current_vx = approach(
                self.current_vx,
                target_vx,
                self.linear_acc * max(dt, 0.0),
            )
        else:
            self.current_wz = target_wz
            self.current_vx = target_vx

        if (
            not target_is_moving
            and abs(self.current_wz) <= 1e-4
            and abs(self.current_vx) <= 1e-4
        ):
            self.motion_started_time = None

        self.speed_level_pub.publish(Int32(data=self.speed_level))
        if self.publish_enable_joy:
            self.joy_pub.publish(make_joy(0.0, 0.0, enable=True))
        self.joy_ctrl_pub.publish(make_twist(self.current_vx, self.current_wz))

    def release_control(self):
        if not hasattr(self, "joy_pub") or self.control_released:
            return

        self.control_released = True

        stop_twist = make_twist(0.0, 0.0)
        for _ in range(10):
            self.joy_ctrl_pub.publish(stop_twist)
            if self.publish_enable_joy:
                self.joy_pub.publish(make_joy(0.0, 0.0, enable=True))
            time.sleep(0.02)

        if self.publish_enable_joy:
            for _ in range(5):
                self.joy_pub.publish(make_joy(0.0, 0.0, enable=False))
                time.sleep(0.02)

    def request_stop(self):
        self.stop_requested = True
        self.publish_command(0.0, 0.0, smooth=False)
        self.release_control()

    def wait_for_odom(self):
        rospy.loginfo("Waiting for odometry: %s", self.odom_topic)
        deadline = rospy.Time.now() + rospy.Duration(self.wait_odom_timeout)
        rate = rospy.Rate(self.control_hz)

        while not rospy.is_shutdown() and self.odom is None:
            if rospy.Time.now() > deadline:
                raise RuntimeError(
                    "Timed out waiting for odometry topic: {}".format(
                        self.odom_topic
                    )
                )
            rate.sleep()

    def check_odom_freshness(self):
        if self.last_odom_time is None:
            raise RuntimeError("Odometry has not been received")

        age = (rospy.Time.now() - self.last_odom_time).to_sec()
        if age > self.odom_timeout:
            raise RuntimeError(
                "Odometry stopped updating for {:.2f} seconds".format(age)
            )

    def align_to_path(self, path_heading_error, distance, dt):
        if distance <= self.position_tolerance:
            self.set_phase(self.ALIGN_FINAL_YAW)
            return

        if (
            distance > self.near_target_distance
            and abs(path_heading_error) <= self.coarse_heading_tolerance
        ):
            self.set_phase(self.DRIVE_TO_TARGET)
            return

        if abs(path_heading_error) <= self.path_heading_tolerance:
            self.settled_count += 1
            self.turn_pid.reset()
            self.publish_command(0.0, 0.0, smooth=False)
            if self.settled_count >= self.settle_cycles:
                self.set_phase(self.DRIVE_TO_TARGET)
            return

        if (
            distance <= self.near_target_distance
            and abs(path_heading_error) <= self.near_target_realign_threshold
        ):
            self.set_phase(self.DRIVE_TO_TARGET)
            return

        self.settled_count = 0
        self.publish_command(self.turn_speed(path_heading_error, dt), 0.0)

    def blended_drive_heading_error(
        self,
        path_heading_error,
        final_heading_error,
        distance,
    ):
        if self.final_yaw_blend_distance <= self.position_tolerance:
            final_weight = self.max_final_yaw_drive_weight
        else:
            final_weight = (
                self.final_yaw_blend_distance - distance
            ) / (
                self.final_yaw_blend_distance - self.position_tolerance
            )

        final_weight = clamp(
            final_weight,
            0.0,
            min(
                self.max_final_yaw_drive_weight,
                1.0 - self.min_path_drive_weight,
            ),
        )
        path_weight = 1.0 - final_weight
        blended_error = (
            path_weight * path_heading_error
            + final_weight * final_heading_error
        )
        return wrap_angle(blended_error), path_weight, final_weight

    def drive_to_target(
        self,
        path_heading_error,
        final_heading_error,
        distance,
        direction,
        dt,
    ):
        if distance <= self.position_tolerance:
            self.set_phase(self.ALIGN_FINAL_YAW)
            return

        realign_threshold = self.coarse_realign_threshold
        if distance <= self.near_target_distance:
            realign_threshold = self.near_target_realign_threshold

        if abs(path_heading_error) > realign_threshold:
            self.set_phase(self.ALIGN_TO_PATH)
            return

        drive_heading_error, _, _ = self.blended_drive_heading_error(
            path_heading_error,
            final_heading_error,
            distance,
        )
        turn_correction = self.drive_turn_pid.update(drive_heading_error, dt)
        forward_speed = abs(self.drive_pid.update(distance, dt))
        max_forward_speed = self.max_forward_axis
        if distance <= self.near_target_distance:
            max_forward_speed = self.creep_drive_axis

        if 0.0 < forward_speed < self.min_drive_axis:
            forward_speed = self.min_drive_axis
        forward_speed = clamp(
            forward_speed,
            self.min_drive_axis,
            max_forward_speed,
        )

        self.publish_command(turn_correction, direction * forward_speed)

    def align_final_yaw(self, final_heading_error, dt):
        if abs(final_heading_error) <= self.final_heading_tolerance:
            self.settled_count += 1
            self.turn_pid.reset()
            self.publish_command(0.0, 0.0, smooth=False)
            return self.settled_count >= self.settle_cycles

        self.settled_count = 0
        self.publish_command(self.turn_speed(final_heading_error, dt), 0.0)
        return False

    def vision_errors(self):
        return (
            self.vision["x"] - self.vision_target_x,
            self.vision["y"] - self.vision_target_y,
            wrap_angle(self.vision["yaw"] - self.vision_target_yaw),
        )

    def projected_vision_y_error(self, error_x, error_y):
        if not self.vision_y_projection_enable:
            return error_y

        # Project lateral error to the target x using the observed QR/wall angle.
        projection_yaw = clamp(
            self.vision["yaw"],
            -self.vision_y_projection_yaw_limit,
            self.vision_y_projection_yaw_limit,
        )
        return (
            error_y
            - self.vision_y_projection_sign
            * self.vision_y_projection_weight
            * math.tan(projection_yaw)
            * error_x
        )

    def reset_vision_lateral_nudge(self):
        self.vision_lateral_nudge_step = None
        self.vision_lateral_nudge_direction = 1.0
        self.vision_lateral_nudge_start_x_error = 0.0
        self.vision_lateral_nudge_start_y_error = 0.0

    def start_vision_lateral_nudge(self, error_x, error_y):
        self.vision_lateral_nudge_step = self.VISION_NUDGE_TURN_OUT
        self.vision_lateral_nudge_direction = math.copysign(
            1.0,
            self.vision_lateral_sign
            * self.vision_lateral_nudge_turn_sign
            * error_y,
        )
        self.vision_lateral_nudge_start_x_error = error_x
        self.vision_lateral_nudge_start_y_error = error_y
        self.vision_lateral_nudge_target_angle = clamp(
            abs(error_y) * self.vision_lateral_nudge_angle_kp,
            self.vision_lateral_nudge_min_angle,
            self.vision_lateral_nudge_max_angle,
        )
        self.vision_x_pid.reset()
        self.vision_y_pid.reset()
        self.vision_yaw_pid.reset()
        rospy.loginfo(
            (
                "Start vision lateral nudge: y_error=%.4f "
                "direction=%+.0f angle=%.3frad"
            ),
            error_y,
            self.vision_lateral_nudge_direction,
            self.vision_lateral_nudge_target_angle,
        )

    def vision_yaw_speed(self, yaw_error, dt):
        if abs(yaw_error) <= self.vision_lateral_nudge_angle_tolerance:
            return 0.0
        return self.vision_lateral_nudge_yaw_sign * self.vision_yaw_pid.update(
            yaw_error,
            dt,
        )

    def vision_lateral_nudge_correct(self, error_x, error_y, dt):
        if self.vision_lateral_nudge_step is None:
            self.start_vision_lateral_nudge(error_x, error_y)

        direction = self.vision_lateral_nudge_direction
        nudge_yaw = wrap_angle(
            self.vision_target_yaw
            + direction * self.vision_lateral_nudge_target_angle
        )
        yaw_to_nudge = wrap_angle(nudge_yaw - self.vision["yaw"])
        yaw_to_target = wrap_angle(
            self.vision_target_yaw - self.vision["yaw"]
        )

        if self.vision_lateral_nudge_step == self.VISION_NUDGE_TURN_OUT:
            if abs(yaw_to_nudge) <= self.vision_lateral_nudge_angle_tolerance:
                self.vision_lateral_nudge_step = self.VISION_NUDGE_FORWARD
                self.vision_yaw_pid.reset()
                rospy.loginfo("Vision lateral nudge: forward")
            else:
                self.publish_command(
                    self.vision_yaw_speed(yaw_to_nudge, dt),
                    0.0,
                )
                return False

        if self.vision_lateral_nudge_step == self.VISION_NUDGE_FORWARD:
            x_moved = abs(
                error_x - self.vision_lateral_nudge_start_x_error
            )
            y_done = abs(error_y) <= self.vision_y_tolerance
            y_crossed = (
                self.vision_lateral_nudge_start_y_error * error_y <= 0.0
            )
            if (
                y_done
                or y_crossed
                or x_moved >= self.vision_lateral_nudge_forward_x
            ):
                self.vision_lateral_nudge_step = self.VISION_NUDGE_TURN_BACK
                self.vision_yaw_pid.reset()
                rospy.loginfo("Vision lateral nudge: turn back")
            else:
                self.publish_command(
                    self.vision_yaw_speed(yaw_to_nudge, dt),
                    self.vision_linear_sign * self.vision_lateral_nudge_speed,
                )
                return False

        if self.vision_lateral_nudge_step == self.VISION_NUDGE_TURN_BACK:
            if abs(yaw_to_target) <= self.vision_lateral_nudge_angle_tolerance:
                self.vision_lateral_nudge_step = self.VISION_NUDGE_REVERSE
                self.vision_yaw_pid.reset()
                rospy.loginfo("Vision lateral nudge: reverse")
            else:
                self.publish_command(
                    self.vision_yaw_speed(yaw_to_target, dt),
                    0.0,
                )
                return False

        if self.vision_lateral_nudge_step == self.VISION_NUDGE_REVERSE:
            if abs(error_x) <= self.vision_lateral_nudge_x_gate:
                self.reset_vision_lateral_nudge()
                self.vision_x_pid.reset()
                self.vision_y_pid.reset()
                self.vision_yaw_pid.reset()
                self.publish_command(0.0, 0.0, smooth=True)
                return False

            self.publish_command(
                self.vision_yaw_speed(yaw_to_target, dt),
                -self.vision_linear_sign * self.vision_lateral_nudge_speed,
            )
            return False

        self.reset_vision_lateral_nudge()
        return False

    def vision_final_correct(self, dt):
        if not self.vision_is_fresh():
            age = self.vision_age()
            rospy.logwarn(
                "Vision lost, fallback to odom correction. age=%s",
                "none" if age is None else "{:.2f}s".format(age),
            )
            self.set_phase(self.DRIVE_TO_TARGET)
            return False

        error_x, error_y, error_yaw = self.vision_errors()
        if (
            abs(error_x) <= self.vision_x_tolerance
            and abs(error_y) <= self.vision_y_tolerance
            and abs(error_yaw) <= self.vision_yaw_tolerance
        ):
            self.reset_vision_lateral_nudge()
            self.settled_count += 1
            self.publish_command(0.0, 0.0, smooth=False)
            return self.settled_count >= self.settle_cycles

        self.settled_count = 0
        if (
            self.vision_lateral_nudge_enable
            and (
                self.vision_lateral_nudge_step is not None
                or (
                    abs(error_x) <= self.vision_lateral_nudge_x_gate
                    and abs(error_y) > self.vision_y_tolerance
                )
            )
        ):
            return self.vision_lateral_nudge_correct(
                error_x,
                error_y,
                dt,
            )

        control_error_y = self.projected_vision_y_error(error_x, error_y)
        if abs(error_x) <= self.vision_x_tolerance:
            self.vision_x_pid.reset()
            linear_x = 0.0
        else:
            linear_x = self.vision_linear_sign * self.vision_x_pid.update(
                error_x,
                dt,
            )

        angular_from_y = self.vision_lateral_sign * self.vision_y_pid.update(
            control_error_y,
            dt,
        )
        angular_from_yaw = self.vision_yaw_sign * self.vision_yaw_pid.update(
            error_yaw,
            dt,
        )
        angular_z = clamp(
            angular_from_y + angular_from_yaw,
            -self.vision_angular_speed,
            self.vision_angular_speed,
        )

        use_lateral_arc = (
            self.vision_lateral_arc_enable
            and abs(control_error_y) > self.vision_y_tolerance
            and abs(error_yaw) <= self.vision_lateral_arc_max_yaw_error
        )
        base_linear_x = linear_x
        if use_lateral_arc:
            y_scale = clamp(
                (
                    abs(control_error_y) - self.vision_y_tolerance
                )
                / max(
                    self.vision_lateral_arc_slow_zone
                    - self.vision_y_tolerance,
                    1e-6,
                ),
                0.0,
                1.0,
            )
            arc_speed = (
                self.min_vision_lateral_arc_speed
                + y_scale
                * (
                    self.vision_lateral_arc_speed
                    - self.min_vision_lateral_arc_speed
                )
            )

            if abs(angular_from_y) > 1e-6:
                turn_sign = math.copysign(1.0, angular_from_y)
            else:
                turn_sign = math.copysign(
                    1.0,
                    self.vision_lateral_sign * control_error_y,
                )

            if abs(angular_z) < self.min_vision_angular_speed:
                angular_z = turn_sign * self.min_vision_angular_speed

            arc_linear_x = (
                self.vision_linear_sign
                * self.vision_lateral_arc_linear_sign
                * arc_speed
            )
            linear_x = (
                (1.0 - y_scale) * base_linear_x
                + y_scale * arc_linear_x
            )

        if (
            abs(error_y) <= self.vision_y_tolerance
            and abs(error_yaw) <= self.vision_yaw_tolerance
        ):
            angular_z = 0.0
        elif (
            abs(control_error_y) <= self.vision_y_tolerance * 2.0
            and abs(error_yaw) <= self.vision_yaw_slow_zone
        ):
            slow_scale = clamp(
                abs(error_yaw) / max(self.vision_yaw_slow_zone, 1e-6),
                0.0,
                1.0,
            )
            slow_angular_limit = (
                self.min_vision_angular_speed
                + slow_scale
                * (self.vision_angular_speed - self.min_vision_angular_speed)
            )
            angular_z = clamp(
                angular_z,
                -slow_angular_limit,
                slow_angular_limit,
            )

        if 0.0 < abs(linear_x) < self.min_vision_linear_speed:
            linear_x = math.copysign(self.min_vision_linear_speed, linear_x)
        if 0.0 < abs(angular_z) < self.min_vision_angular_speed:
            angular_z = math.copysign(self.min_vision_angular_speed, angular_z)

        self.publish_command(angular_z, linear_x)
        return False

    def run(self):
        rospy.loginfo(
            "Target pose: x=%.5f, y=%.5f, yaw=%.5f rad",
            self.target_x,
            self.target_y,
            self.target_yaw,
        )
        rospy.loginfo(
            (
                "Twist topic: %s, Joy enable topic: %s, "
                "speed level topic: %s, speed level: %d (%+d steps)"
            ),
            self.joy_ctrl_topic,
            self.joy_topic,
            self.speed_level_topic,
            self.speed_level,
            self.speed_level_steps,
        )
        rospy.loginfo(
            (
                "Velocity limits: linear %.2fm/s (%+d steps), "
                "angular %.2frad/s (%+d steps), acc=(%.2f, %.2f), "
                "soft_start=%.2fs"
            ),
            self.linear_speed,
            self.linear_speed_steps,
            self.angular_speed,
            self.angular_speed_steps,
            self.linear_acc,
            self.angular_acc,
            self.soft_start_duration,
        )
        rospy.loginfo(
            "Reverse correction: %s, publish Joy enable: %s",
            "enabled" if self.allow_reverse else "disabled",
            self.publish_enable_joy,
        )
        rospy.loginfo(
            (
                "Vision final: %s topic=%s handoff=%.2fm timeout=%.2fs "
                "target=(%.4f, %.4f, %.3f)"
            ),
            "enabled" if self.enable_vision_final else "disabled",
            self.vision_topic,
            self.vision_handoff_distance,
            self.vision_timeout,
            self.vision_target_x,
            self.vision_target_y,
            self.vision_target_yaw,
        )
        rospy.loginfo(
            (
                "Vision lateral arc: %s speed=%.3fm/s min=%.3fm/s "
                "slow_zone=%.3fm yaw_gate=%.3frad linear_sign=%.1f"
            ),
            "enabled" if self.vision_lateral_arc_enable else "disabled",
            self.vision_lateral_arc_speed,
            self.min_vision_lateral_arc_speed,
            self.vision_lateral_arc_slow_zone,
            self.vision_lateral_arc_max_yaw_error,
            self.vision_lateral_arc_linear_sign,
        )
        rospy.loginfo(
            (
                "Vision y projection: %s weight=%.2f sign=%.1f yaw_limit=%.3frad"
            ),
            "enabled" if self.vision_y_projection_enable else "disabled",
            self.vision_y_projection_weight,
            self.vision_y_projection_sign,
            self.vision_y_projection_yaw_limit,
        )
        rospy.loginfo(
            (
                "Vision lateral nudge: %s angle=(%.3f..%.3f)rad "
                "angle_kp=%.2f x_gate=%.3fm forward_x=%.3fm "
                "speed=%.3fm/s turn_sign=%.1f yaw_sign=%.1f"
            ),
            "enabled" if self.vision_lateral_nudge_enable else "disabled",
            self.vision_lateral_nudge_min_angle,
            self.vision_lateral_nudge_max_angle,
            self.vision_lateral_nudge_angle_kp,
            self.vision_lateral_nudge_x_gate,
            self.vision_lateral_nudge_forward_x,
            self.vision_lateral_nudge_speed,
            self.vision_lateral_nudge_turn_sign,
            self.vision_lateral_nudge_yaw_sign,
        )
        rospy.loginfo(
            (
                "Coarse/fine correction: coarse heading %.3frad, "
                "coarse realign %.3frad, near %.2fm, near realign %.3frad, "
                "yaw blend %.2fm max weight %.2f"
            ),
            self.coarse_heading_tolerance,
            self.coarse_realign_threshold,
            self.near_target_distance,
            self.near_target_realign_threshold,
            self.final_yaw_blend_distance,
            min(
                self.max_final_yaw_drive_weight,
                1.0 - self.min_path_drive_weight,
            ),
        )

        self.wait_for_odom()
        self.speed_level_pub.publish(Int32(data=self.speed_level))
        rospy.sleep(0.3)

        started_at = rospy.Time.now()
        last_loop_time = rospy.Time.now()
        rate = rospy.Rate(self.control_hz)

        while not rospy.is_shutdown():
            if self.stop_requested:
                raise rospy.ROSInterruptException("twice_move stop requested")

            now = rospy.Time.now()
            dt = (now - last_loop_time).to_sec()
            last_loop_time = now

            if (rospy.Time.now() - started_at).to_sec() > self.total_timeout:
                raise RuntimeError(
                    "Correction timed out after {:.1f} seconds".format(
                        self.total_timeout
                    )
                )

            self.check_odom_freshness()
            x, y, yaw = self.current_pose()
            dx = self.target_x - x
            dy = self.target_y - y
            distance = math.hypot(dx, dy)
            path_heading = math.atan2(dy, dx)
            drive_direction, path_heading_error = self.select_motion_direction(
                path_heading,
                yaw,
            )
            self.update_motion_direction(drive_direction)
            final_heading_error = wrap_angle(self.target_yaw - yaw)
            drive_heading_error, path_weight, final_weight = (
                self.blended_drive_heading_error(
                    path_heading_error,
                    final_heading_error,
                    distance,
                )
            )
            vision_age = self.vision_age()
            vision_ready = self.should_use_vision(distance)
            vision_text = "none"
            if self.vision is not None:
                vision_text = (
                    "x={:.3f} y={:.3f} yaw={:.3f}rad age={}"
                ).format(
                    self.vision["x"],
                    self.vision["y"],
                    self.vision["yaw"],
                    "none"
                    if vision_age is None
                    else "{:.2f}s".format(vision_age),
                )

            rospy.loginfo_throttle(
                1.0,
                (
                    "phase={} pose=({:.4f}, {:.4f}, {:.3f}) "
                    "distance={:.4f} drive={} path_error={:.3f}rad "
                    "final_error={:.3f}rad blend_error={:.3f}rad "
                    "weight=({:.2f},{:.2f}) vision_ready={} vision={} "
                    "cmd_wz_vx=({:.3f}, {:.3f})"
                ).format(
                    self.phase,
                    x,
                    y,
                    yaw,
                    distance,
                    "reverse" if drive_direction < 0.0 else "forward",
                    path_heading_error,
                    final_heading_error,
                    drive_heading_error,
                    path_weight,
                    final_weight,
                    vision_ready,
                    vision_text,
                    self.current_wz,
                    self.current_vx,
                ),
            )

            if (
                vision_ready
                and self.phase != self.VISION_FINAL_CORRECT
            ):
                self.set_phase(self.VISION_FINAL_CORRECT)

            if self.phase == self.VISION_FINAL_CORRECT:
                if self.vision_final_correct(dt):
                    self.publish_command(0.0, 0.0, smooth=False)
                    error_x, error_y, error_yaw = self.vision_errors()
                    rospy.loginfo(
                        (
                            "Vision correction complete: "
                            "error=(%.4f, %.4f, %.3frad)"
                        ),
                        error_x,
                        error_y,
                        error_yaw,
                    )
                    return
            elif self.phase == self.ALIGN_TO_PATH:
                self.align_to_path(path_heading_error, distance, dt)
            elif self.phase == self.DRIVE_TO_TARGET:
                self.drive_to_target(
                    path_heading_error,
                    final_heading_error,
                    distance,
                    drive_direction,
                    dt,
                )
            elif self.phase == self.ALIGN_FINAL_YAW:
                if self.align_final_yaw(final_heading_error, dt):
                    self.publish_command(0.0, 0.0, smooth=False)
                    rospy.loginfo(
                        "Correction complete: distance=%.4f m, yaw_error=%.3f rad",
                        distance,
                        final_heading_error,
                    )
                    return
            else:
                raise RuntimeError("Unknown phase: {}".format(self.phase))

            rate.sleep()


def main():
    rospy.init_node("twice_move_corrector", anonymous=True)
    controller = TwiceMoveCorrector()

    try:
        controller.run()
    finally:
        controller.release_control()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("twice_move failed: %s", exc)
