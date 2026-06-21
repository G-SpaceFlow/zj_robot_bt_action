#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import time

import rospy

from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from upperlimb.msg import Joints
from std_msgs.msg import Int32, String
from upperlimb.srv import IK, IKRequest
from upperlimb.srv import MoveJ, MoveJRequest
from upperlimb.srv import MoveJByPath, MoveJByPathRequest
from upperlimb.srv import MoveL, MoveLRequest
from hand.srv import HandJoint, HandJointRequest
from robot_bt_action.srv import GetTarget, GetTargetRequest
from robot_bt_action.srv import NavigateToPose, NavigateToPoseRequest
from robot_bt_action.srv import SetOffset, SetOffsetRequest


IK_LEFT_SERVICE = "/zj_humanoid/upperlimb/IK/left_arm"
IK_RIGHT_SERVICE = "/zj_humanoid/upperlimb/IK/right_arm"
SET_OFFSET_SERVICE = "/bt_target_server/set_offset"
GET_TARGET_SERVICE = "/bt_target_server/get_target"
TRIGGER_DETECT_SERVICE = "/bt_target_server/trigger_detect"
NAVIGATION_SERVICE = "/bt_navigation_server/navigate_to_pose"
NAVIGATION_ACTION_NAME = "/zj_humanoid/navigation/navigation"
NAVIGATION_CANCEL_TOPIC = NAVIGATION_ACTION_NAME + "/cancel"
NAVIGATION_STATE_TOPIC = "/bt_navigation_server/state"
BT_NAVIGATION_CANCEL_TOPIC = "/bt_navigation_server/cancel"
NAVIGATION_STATE_NAVIGATION = "navigation"
NAVIGATION_STATE_TWICE_MOVE = "twice_move"
DEFAULT_BASE_MOTION_TOPIC = "/jzhw/joy_ctrl"
DEFAULT_BASE_JOY_TOPIC = "/jzhw/joy"
DEFAULT_BASE_SPEED_LEVEL_TOPIC = "/jzhw/joy3_ctrl/speed_level"


class ControlServiceClient(object):
    """集中封装行为树执行过程中用到的 ROS 控制服务调用。"""

    def __init__(self,
                 ik_left_service=IK_LEFT_SERVICE,
                 ik_right_service=IK_RIGHT_SERVICE):
        self.ik_left_service = ik_left_service
        self.ik_right_service = ik_right_service
        self.navigation_request_active = False
        self.navigation_cancel_pub = rospy.Publisher(
            NAVIGATION_CANCEL_TOPIC,
            GoalID,
            queue_size=10,
        )
        self.bt_navigation_cancel_pub = rospy.Publisher(
            BT_NAVIGATION_CANCEL_TOPIC,
            String,
            queue_size=10,
        )
        self.navigation_server_state = ""
        self.navigation_state_sub = rospy.Subscriber(
            NAVIGATION_STATE_TOPIC,
            String,
            self.navigation_state_callback,
            queue_size=1,
        )
        self.base_motion_pubs = {}
        self.base_joy_pubs = {}
        self.base_speed_level_pubs = {}

    def navigation_state_callback(self, msg):
        self.navigation_server_state = msg.data

    def cancel_bt_navigation_server(self, reason="cancel"):
        deadline = time.time() + 1.0
        while (
            self.bt_navigation_cancel_pub.get_num_connections() == 0
            and time.time() < deadline
            and not rospy.is_shutdown()
        ):
            rospy.sleep(0.02)

        rospy.logwarn(
            "发送行为树导航服务取消话题: %s reason=%s",
            BT_NAVIGATION_CANCEL_TOPIC,
            reason,
        )
        cancel_msg = String(data=str(reason))
        for _ in range(10):
            self.bt_navigation_cancel_pub.publish(cancel_msg)
            rospy.sleep(0.03)

    def cancel_navigation(self):
        if not self.navigation_request_active:
            return

        if self.navigation_server_state == NAVIGATION_STATE_TWICE_MOVE:
            rospy.logwarn("二次矫正阶段退出，发送二次矫正取消请求")
            self.cancel_bt_navigation_server("twice_move_cancel")
            return

        if (
            self.navigation_server_state
            and self.navigation_server_state != NAVIGATION_STATE_NAVIGATION
        ):
            rospy.loginfo(
                "当前导航服务状态为 %s，退出时不发送导航取消话题",
                self.navigation_server_state,
            )
            return

        deadline = time.time() + 1.0
        while (
            self.navigation_cancel_pub.get_num_connections() == 0
            and time.time() < deadline
            and not rospy.is_shutdown()
        ):
            rospy.sleep(0.02)

        rospy.logwarn("地图导航阶段退出，发送导航取消话题: %s", NAVIGATION_CANCEL_TOPIC)
        cancel_msg = GoalID()
        for _ in range(10):
            self.navigation_cancel_pub.publish(cancel_msg)
            rospy.sleep(0.03)

    @staticmethod
    def list_to_joints(joint_list):
        msg = Joints()
        msg.joint = [float(x) for x in joint_list]
        return msg

    @staticmethod
    def flatten_joint_group(joint_group):
        if not isinstance(joint_group, list):
            raise ValueError("关节数据必须是 list")

        full_joints = []
        for part in joint_group:
            if not isinstance(part, list):
                raise ValueError("关节分组里的每一项必须是 list")
            full_joints.extend(part)

        return full_joints

    def call_ik(self, ik_service_name, target_pose, q7_ref):
        rospy.loginfo("等待 IK 服务: {}".format(ik_service_name))
        rospy.wait_for_service(ik_service_name)

        try:
            ik_client = rospy.ServiceProxy(ik_service_name, IK)

            req = IKRequest()
            req.pose = target_pose
            req.q7 = float(q7_ref)

            resp = ik_client(req)

            if hasattr(resp, "success") and not resp.success:
                rospy.logwarn("IK 求解失败: {}".format(getattr(resp, "message", "")))
                return None

            if hasattr(resp, "nums") and resp.nums <= 0:
                rospy.logwarn("IK 无解")
                return None

            return resp.joints

        except rospy.ServiceException as exc:
            rospy.logerr("调用 IK 服务失败: {}".format(exc))
            return None

    def call_left_ik(self, target_pose, q7_ref):
        return self.call_ik(self.ik_left_service, target_pose, q7_ref)

    def call_right_ik(self, target_pose, q7_ref):
        return self.call_ik(self.ik_right_service, target_pose, q7_ref)

    def set_offset(self, offset_entry, offset_id=None):
        try:
            rospy.wait_for_service(SET_OFFSET_SERVICE, timeout=2.0)
            set_offset = rospy.ServiceProxy(SET_OFFSET_SERVICE, SetOffset)
            req = SetOffsetRequest()
            for field in [
                "left_dx", "left_dy", "left_dz",
                "right_dx", "right_dy", "right_dz",
                "left_ox", "left_oy", "left_oz", "left_ow",
                "right_ox", "right_oy", "right_oz", "right_ow",
            ]:
                setattr(req, field, offset_entry.get(field, 0.0))

            resp = set_offset(req)
            if resp.success:
                if offset_id is None:
                    rospy.loginfo("补偿参数已切换")
                else:
                    rospy.loginfo("补偿参数已切换: %s", offset_id)
                return True

            rospy.logwarn("set_offset 返回失败: %s", resp.message)
            return False
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("set_offset 服务异常: %s", str(e))
            return False

    def call_movej_whole_body(self, service_name, joints, motion):
        rospy.loginfo("等待 movej 服务: {}".format(service_name))
        rospy.wait_for_service(service_name)

        req = MoveJRequest()
        req.joints = joints
        req.v = float(motion.get("v", 0.5))
        req.acc = float(motion.get("acc", 0.05))
        req.t = float(motion.get("t", 0.0))
        req.is_async = bool(motion.get("is_async", False))
        req.arm_type = int(motion.get("arm_type", 31))

        try:
            client = rospy.ServiceProxy(service_name, MoveJ)
            resp = client(req)

            if resp.success:
                rospy.loginfo("movej 执行成功: {}".format(service_name))
                return True

            rospy.logwarn("movej 执行失败: {}".format(resp.message))
            return False

        except rospy.ServiceException as exc:
            rospy.logerr("调用 movej 服务失败: {}".format(exc))
            return False

    def call_movej_by_path_whole_body(self, service_name, path, motion):
        rospy.loginfo("等待 movej_by_path 服务: {}".format(service_name))
        rospy.wait_for_service(service_name)

        req = MoveJByPathRequest()
        req.path = []

        for point in path:
            full_joints = self.flatten_joint_group(point)
            rospy.loginfo("添加轨迹点，关节数量: {}".format(len(full_joints)))
            req.path.append(self.list_to_joints(full_joints))

        req.time = float(motion.get("time", 0.0))
        req.timestamp = [float(x) for x in motion.get("timestamp", [])]
        req.is_async = bool(motion.get("is_async", False))
        req.arm_type = int(motion.get("arm_type", 31))

        try:
            client = rospy.ServiceProxy(service_name, MoveJByPath)
            resp = client(req)

            if resp.success:
                rospy.loginfo("movej_by_path 执行成功: {}".format(service_name))
                return True

            rospy.logwarn("movej_by_path 执行失败: {}".format(resp.message))
            return False

        except rospy.ServiceException as exc:
            rospy.logerr("调用 movej_by_path 服务失败: {}".format(exc))
            return False

    def call_dual_arm_movel(self, service_name, left_pose, right_pose, motion):
        req = MoveLRequest()
        req.pose = [left_pose, right_pose]
        req.v = float(motion.get("v", 0.5))
        req.acc = float(motion.get("acc", 0.05))
        req.is_async = bool(motion.get("is_async", False))

        rospy.loginfo(
            "MoveL请求位姿 left_pos=(%.3f, %.3f, %.3f), left_quat=(%.4f, %.4f, %.4f, %.4f), "
            "right_pos=(%.3f, %.3f, %.3f), right_quat=(%.4f, %.4f, %.4f, %.4f)",
            left_pose.position.x, left_pose.position.y, left_pose.position.z,
            left_pose.orientation.x, left_pose.orientation.y,
            left_pose.orientation.z, left_pose.orientation.w,
            right_pose.position.x, right_pose.position.y, right_pose.position.z,
            right_pose.orientation.x, right_pose.orientation.y,
            right_pose.orientation.z, right_pose.orientation.w
        )
        rospy.loginfo("调用双臂 MoveL: v=%.2f, acc=%.2f, async=%d",
                      req.v, req.acc, req.is_async)

        try:
            rospy.wait_for_service(service_name)
            client = rospy.ServiceProxy(service_name, MoveL)
            resp = client(req)
            if resp.success:
                rospy.loginfo("dual_arm_moveL 执行成功")
                return True
            rospy.logwarn("dual_arm_moveL 执行失败: %s", resp.message)
            return False
        except rospy.ServiceException as exc:
            rospy.logerr("调用 dual_arm_moveL 失败: %s", str(exc))
            return False

    def call_hand_joint(self, service_name, q_values):
        try:
            rospy.wait_for_service(service_name, timeout=2.0)
            client = rospy.ServiceProxy(service_name, HandJoint)
            req = HandJointRequest()
            req.q = [float(v) for v in q_values]
            resp = client(req)
            if resp.success:
                rospy.loginfo("手掌关节动作成功: %s", service_name)
                return True
            rospy.logwarn("手掌关节动作失败: %s", resp.message)
            return False
        except rospy.ServiceException as e:
            rospy.logerr("手掌关节动作调用失败: %s", str(e))
            return False

    def get_target(self, key, timeout=1.0, motion_mode=1, point_names=None):
        try:
            rospy.wait_for_service(GET_TARGET_SERVICE, timeout=timeout)
            get_target = rospy.ServiceProxy(GET_TARGET_SERVICE, GetTarget)
            req = GetTargetRequest()
            req.key = key
            if hasattr(req, "motion_mode"):
                req.motion_mode = int(motion_mode)
            if hasattr(req, "point_names"):
                req.point_names = [str(name) for name in (point_names or [])]
            return get_target(req)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("获取缓存位姿异常: %s", str(e))
            return None

    def trigger_detect(self, key, trigger_value=2, labels=None, point_names=None, motion_mode=1, timeout=1.0):
        try:
            rospy.wait_for_service(TRIGGER_DETECT_SERVICE, timeout=timeout)
            trigger = rospy.ServiceProxy(TRIGGER_DETECT_SERVICE, GetTarget)
            req = GetTargetRequest()
            req.key = key
            req.trigger_value = int(trigger_value)
            if hasattr(req, "labels"):
                req.labels = [str(label) for label in (labels or [])]
            if hasattr(req, "motion_mode"):
                req.motion_mode = int(motion_mode)
            if hasattr(req, "point_names"):
                req.point_names = [str(name) for name in (point_names or [])]
            return trigger(req)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logwarn("动态位姿服务异常: %s", str(e))
            return None

    @staticmethod
    def _read_float(data, key, default=float("nan")):
        if not isinstance(data, dict):
            return default
        value = data.get(key, default)
        if value is None:
            return default
        return float(value)

    @staticmethod
    def _read_int(data, key, default=0):
        if not isinstance(data, dict):
            return default
        value = data.get(key, default)
        if value is None:
            return default
        return int(value)

    def _get_base_motion_pub(self, topic):
        if topic not in self.base_motion_pubs:
            self.base_motion_pubs[topic] = rospy.Publisher(
                topic,
                Twist,
                queue_size=1,
                latch=False,
                tcp_nodelay=True,
            )
        return self.base_motion_pubs[topic]

    def _get_base_joy_pub(self, topic):
        if topic not in self.base_joy_pubs:
            self.base_joy_pubs[topic] = rospy.Publisher(
                topic,
                Joy,
                queue_size=1,
                latch=False,
                tcp_nodelay=True,
            )
        return self.base_joy_pubs[topic]

    def _get_base_speed_level_pub(self, topic):
        if topic not in self.base_speed_level_pubs:
            self.base_speed_level_pubs[topic] = rospy.Publisher(
                topic,
                Int32,
                queue_size=1,
                latch=True,
            )
        return self.base_speed_level_pubs[topic]

    @staticmethod
    def _read_motion_float(action, motion, names, default=0.0):
        for owner in (motion, action):
            if not isinstance(owner, dict):
                continue
            for name in names:
                if name not in owner:
                    continue
                value = owner.get(name)
                if value is None:
                    continue
                return float(value)
        return float(default)

    @staticmethod
    def _read_motion_bool(action, motion, names, default=False):
        for owner in (motion, action):
            if not isinstance(owner, dict):
                continue
            for name in names:
                if name not in owner:
                    continue
                value = owner.get(name)
                if isinstance(value, bool):
                    return value
                if isinstance(value, str):
                    return value.strip().lower() not in ("0", "false", "no", "off")
                return bool(value)
        return bool(default)

    @staticmethod
    def _motion_profile_duration(distance, max_speed, acceleration):
        distance = abs(float(distance))
        max_speed = abs(float(max_speed))
        acceleration = abs(float(acceleration))
        if distance <= 1e-9:
            return 0.0
        if max_speed <= 1e-9:
            return 0.0
        if acceleration <= 1e-9:
            return distance / max_speed

        ramp_distance = max_speed * max_speed / acceleration
        if distance <= ramp_distance:
            return 2.0 * math.sqrt(distance / acceleration)
        return 2.0 * max_speed / acceleration + (distance - ramp_distance) / max_speed

    @staticmethod
    def _motion_profile_speed(elapsed, distance, max_speed, acceleration):
        distance = abs(float(distance))
        elapsed = max(0.0, float(elapsed))
        max_speed = abs(float(max_speed))
        acceleration = abs(float(acceleration))
        if distance <= 1e-9 or max_speed <= 1e-9:
            return 0.0
        if acceleration <= 1e-9:
            duration = distance / max_speed
            return max_speed if elapsed < duration else 0.0

        ramp_distance = max_speed * max_speed / acceleration
        if distance <= ramp_distance:
            duration = 2.0 * math.sqrt(distance / acceleration)
            if elapsed >= duration:
                return 0.0
            return acceleration * min(elapsed, duration - elapsed)

        ramp_time = max_speed / acceleration
        cruise_time = (distance - ramp_distance) / max_speed
        duration = 2.0 * ramp_time + cruise_time
        if elapsed >= duration:
            return 0.0
        if elapsed < ramp_time:
            return acceleration * elapsed
        if elapsed < ramp_time + cruise_time:
            return max_speed
        return acceleration * (duration - elapsed)

    @staticmethod
    def _publish_stop(pub, repeat_count=8, interval=0.03):
        stop_msg = Twist()
        for _ in range(max(1, int(repeat_count))):
            pub.publish(stop_msg)
            rospy.sleep(interval)

    @staticmethod
    def _make_base_joy(axis_turn=0.0, axis_forward=0.0, enable=True):
        msg = Joy()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "joystick"
        msg.axes = [0.0] * 8
        msg.buttons = [0] * 15
        msg.axes[7] = float(axis_forward)
        msg.axes[0] = float(axis_turn)
        msg.buttons[0] = 1 if enable else 0
        return msg

    def call_open_loop_motion(self, action):
        motion = action.get("motion", {}) or {}
        topic = str(motion.get("topic", action.get("topic", DEFAULT_BASE_MOTION_TOPIC)))
        pub = self._get_base_motion_pub(topic)
        joy_topic = str(motion.get("joy_topic", action.get("joy_topic", DEFAULT_BASE_JOY_TOPIC)))
        joy_pub = self._get_base_joy_pub(joy_topic)
        speed_level_topic = str(
            motion.get(
                "speed_level_topic",
                action.get("speed_level_topic", DEFAULT_BASE_SPEED_LEVEL_TOPIC),
            )
        )
        speed_level_pub = self._get_base_speed_level_pub(speed_level_topic)

        dx = self._read_motion_float(action, motion, ("dx", "x", "linear_x"), 0.0)
        dy = self._read_motion_float(action, motion, ("dy", "y", "linear_y"), 0.0)
        dyaw = self._read_motion_float(action, motion, ("dyaw", "yaw", "angular_z"), 0.0)

        if abs(dx) < 1e-9 and abs(dy) < 1e-9 and abs(dyaw) < 1e-9:
            rospy.logwarn("开环运动没有配置 dx/dy/dyaw，跳过: %s", action.get("name", ""))
            return True

        linear_speed = abs(self._read_motion_float(action, motion, ("linear_speed", "speed"), 0.08))
        angular_speed = abs(self._read_motion_float(action, motion, ("angular_speed", "yaw_speed"), 0.20))
        linear_acc = abs(self._read_motion_float(action, motion, ("linear_acc", "acc"), 0.5))
        angular_acc = abs(self._read_motion_float(action, motion, ("angular_acc",), 0.6))
        rate_hz = max(5.0, self._read_motion_float(action, motion, ("rate", "rate_hz"), 30.0))
        min_duration = max(0.05, self._read_motion_float(action, motion, ("min_duration",), 0.3))
        stop_repeat = int(self._read_motion_float(action, motion, ("stop_repeat",), 8))
        speed_level = int(self._read_motion_float(action, motion, ("speed_level", "init_speed_level"), 2))
        publisher_warmup = max(
            0.0,
            self._read_motion_float(action, motion, ("publisher_warmup", "warmup"), 0.3),
        )
        publish_enable_joy = self._read_motion_bool(
            action,
            motion,
            ("publish_enable_joy", "enable_joy"),
            False,
        )
        wait_for_connection = self._read_motion_bool(
            action,
            motion,
            ("wait_for_connection",),
            False,
        )

        configured_duration = self._read_motion_float(action, motion, ("duration", "time", "t"), 0.0)
        if configured_duration > 0.0:
            duration = configured_duration
        else:
            linear_distance = math.sqrt(dx * dx + dy * dy)
            linear_duration = self._motion_profile_duration(
                linear_distance,
                linear_speed,
                linear_acc,
            )
            angular_duration = self._motion_profile_duration(
                abs(dyaw),
                angular_speed,
                angular_acc,
            )
            duration = max(min_duration, linear_duration, angular_duration)

        if wait_for_connection:
            deadline = time.time() + 1.0
            while pub.get_num_connections() == 0 and time.time() < deadline and not rospy.is_shutdown():
                rospy.sleep(0.02)

        rospy.loginfo(
            "执行开环运动: topic=%s, joy_topic=%s, speed_level_topic=%s, speed_level=%d, publish_enable_joy=%s, dx=%.4f, dy=%.4f, dyaw=%.4f, duration=%.3fs, linear_speed=%.3f, angular_speed=%.3f",
            topic,
            joy_topic,
            speed_level_topic,
            speed_level,
            publish_enable_joy,
            dx,
            dy,
            dyaw,
            duration,
            linear_speed,
            angular_speed,
        )

        rate = rospy.Rate(rate_hz)
        start_time = rospy.Time.now()
        linear_distance = math.sqrt(dx * dx + dy * dy)
        linear_unit_x = dx / linear_distance if linear_distance > 1e-9 else 0.0
        linear_unit_y = dy / linear_distance if linear_distance > 1e-9 else 0.0
        yaw_sign = math.copysign(1.0, dyaw) if abs(dyaw) > 1e-9 else 0.0
        max_published_linear = 0.0
        max_published_angular = 0.0
        try:
            if publisher_warmup > 0.0:
                rospy.sleep(publisher_warmup)
            speed_level_pub.publish(Int32(data=speed_level))
            if publish_enable_joy:
                joy_pub.publish(self._make_base_joy(0.0, 0.0, enable=True))
            while not rospy.is_shutdown():
                elapsed = (rospy.Time.now() - start_time).to_sec()
                if elapsed >= duration:
                    break
                linear_axis_speed = self._motion_profile_speed(
                    elapsed,
                    linear_distance,
                    linear_speed,
                    linear_acc,
                )
                angular_axis_speed = self._motion_profile_speed(
                    elapsed,
                    abs(dyaw),
                    angular_speed,
                    angular_acc,
                )

                cmd = Twist()
                cmd.linear.x = linear_unit_x * linear_axis_speed
                cmd.linear.y = linear_unit_y * linear_axis_speed
                cmd.angular.z = yaw_sign * angular_axis_speed
                max_published_linear = max(max_published_linear, abs(cmd.linear.x), abs(cmd.linear.y))
                max_published_angular = max(max_published_angular, abs(cmd.angular.z))
                speed_level_pub.publish(Int32(data=speed_level))
                if publish_enable_joy:
                    joy_pub.publish(self._make_base_joy(0.0, 0.0, enable=True))
                pub.publish(cmd)
                rate.sleep()
        except (KeyboardInterrupt, rospy.ROSInterruptException):
            self._publish_stop(pub, repeat_count=stop_repeat)
            raise
        finally:
            self._publish_stop(pub, repeat_count=stop_repeat)
            if publish_enable_joy:
                for _ in range(5):
                    joy_pub.publish(self._make_base_joy(0.0, 0.0, enable=False))
                    rospy.sleep(0.02)

        rospy.loginfo(
            "开环运动完成: %s, max_linear=%.3f, max_angular=%.3f",
            action.get("name", ""),
            max_published_linear,
            max_published_angular,
        )
        return True

    def call_navigation(self, nav_pose):
        service_name = nav_pose.get("service", NAVIGATION_SERVICE)
        wait_timeout = float(nav_pose.get("service_timeout", nav_pose.get("timeout", 5.0)))
        rospy.loginfo("等待导航服务: %s", service_name)
        rospy.wait_for_service(service_name, timeout=wait_timeout)

        twice_move = nav_pose.get("twice_move", {}) or {}
        target = twice_move.get("target", {}) or {}
        vision_target = twice_move.get("vision_target", {}) or {}
        mode = str(nav_pose.get("mode", nav_pose.get("type", ""))).strip().lower()
        skip_navigation = bool(nav_pose.get("skip_navigation", False)) or mode in (
            "twice_move",
            "twice_move_only",
            "correction",
            "correct_only",
        )

        req = NavigateToPoseRequest()
        if skip_navigation:
            req.x = self._read_float(nav_pose, "x", float("nan"))
            req.y = self._read_float(nav_pose, "y", float("nan"))
            req.yaw = self._read_float(nav_pose, "yaw", float("nan"))
        else:
            req.x = float(nav_pose["x"])
            req.y = float(nav_pose["y"])
            req.yaw = float(nav_pose["yaw"])
        req.waypoint_id = self._read_int(nav_pose, "id", 1)
        req.action = self._read_int(nav_pose, "action", 0)
        req.audio = self._read_int(nav_pose, "audio", 0)
        req.distance_tolerance = self._read_float(nav_pose, "distance_tolerance", 0.04)
        req.heading_tolerance = self._read_float(nav_pose, "heading_tolerance", 0.04)
        req.timeout = self._read_float(nav_pose, "navigation_timeout", nav_pose.get("action_timeout", 180.0))
        req.frame_id = str(nav_pose.get("frame_id", "map"))
        req.skip_navigation = skip_navigation

        req.enable_twice_move = bool(twice_move.get("enabled", bool(twice_move)))
        req.vision_trigger_value = self._read_int(twice_move, "trigger_value", 4)
        req.vision_stop_value = self._read_int(twice_move, "stop_value", 0)
        req.vision_control_topic = str(twice_move.get("vision_control_topic", "/yolo_vision/control"))
        req.target_x = self._read_float(target, "x", self._read_float(twice_move, "target_x", float("nan")))
        req.target_y = self._read_float(target, "y", self._read_float(twice_move, "target_y", float("nan")))
        req.target_yaw = self._read_float(target, "yaw", self._read_float(twice_move, "target_yaw", float("nan")))
        req.vision_target_x = self._read_float(
            vision_target,
            "x",
            self._read_float(twice_move, "vision_target_x", float("nan")),
        )
        req.vision_target_y = self._read_float(
            vision_target,
            "y",
            self._read_float(twice_move, "vision_target_y", float("nan")),
        )
        req.vision_target_yaw = self._read_float(
            vision_target,
            "yaw",
            self._read_float(twice_move, "vision_target_yaw", float("nan")),
        )
        req.twice_move_params_json = json.dumps(
            twice_move.get("params", {}) or {},
            ensure_ascii=False,
        )

        try:
            self.navigation_server_state = (
                NAVIGATION_STATE_TWICE_MOVE if skip_navigation else NAVIGATION_STATE_NAVIGATION
            )
            self.navigation_request_active = True
            client = rospy.ServiceProxy(service_name, NavigateToPose)
            resp = client(req)
            if resp.success:
                rospy.loginfo("导航/二次矫正执行成功")
                return True

            rospy.logwarn("导航/二次矫正失败: %s", resp.message)
            return False
        except (KeyboardInterrupt, rospy.ROSInterruptException):
            self.cancel_navigation()
            raise
        except rospy.ServiceException as exc:
            rospy.logerr("调用导航服务失败: %s", str(exc))
            return False
        finally:
            self.navigation_request_active = False

    def call_twice_move(self, twice_move):
        twice_move = dict(twice_move or {})
        service_name = twice_move.pop("service", NAVIGATION_SERVICE)
        twice_move.setdefault("enabled", True)
        nav_pose = {
            "service": service_name,
            "mode": "twice_move",
            "skip_navigation": True,
            "twice_move": twice_move,
        }
        return self.call_navigation(nav_pose)
