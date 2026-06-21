#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import copy
import math
import rospy
import yaml
import tf
import tf.transformations as tfm

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
PROJECT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, os.pardir))

try:
    from scripts.executor_services import ControlServiceClient
    from scripts.executor_services import IK_LEFT_SERVICE, IK_RIGHT_SERVICE
    from scripts.bt_runner_config import build_exec_config
except ImportError:
    from executor_services import ControlServiceClient
    from executor_services import IK_LEFT_SERVICE, IK_RIGHT_SERVICE
    from bt_runner_config import build_exec_config


DEFAULT_TF_BASE_FRAME = "BASE"
DEFAULT_TF_LEFT_HAND_FRAME = "HAND_L"
DEFAULT_TF_RIGHT_HAND_FRAME = "HAND_R"
OFFSET_PARAMS_PATH = os.path.join(PROJECT_DIR, "offset_params.yaml")
CENTER_CACHE_PREFIX = "__center__:"

def load_yaml(file_path):
    with open(file_path, "r", encoding="utf-8") as file_obj:
        return yaml.safe_load(file_obj)


class BehaviorTreeExecutor(object):
    def __init__(self,
                 config=None,
                 ik_left_service=IK_LEFT_SERVICE,
                 ik_right_service=IK_RIGHT_SERVICE):
        self.config = build_exec_config(config)
        self.services = ControlServiceClient(
            ik_left_service=ik_left_service,
            ik_right_service=ik_right_service,
        )
        self.ik_left_service = self.services.ik_left_service
        self.ik_right_service = self.services.ik_right_service
        self._pose_cache = {} 
        self._relative_offset_indices = {}
        self._relative_offset_pending = {}
        self.tf_listener = tf.TransformListener()
        self.offset_map = self._load_offset_map()

    def _load_offset_map(self):
        try:
            with open(OFFSET_PARAMS_PATH, "r", encoding="utf-8") as f:
                offset_data = yaml.safe_load(f) or {}
            offset_map = {}
            for entry in offset_data.get("offsets", []):
                offset_map[entry["id"]] = entry
            rospy.loginfo("成功加载偏移参数, 共 %d 组", len(offset_map))
            return offset_map
        except Exception as e:
            rospy.logwarn("加载偏移参数失败: %s", str(e))
            return {}


    def load_tree(self, file_path):
        return load_yaml(file_path)

    def get_current_waist_angle(self, timeout=0.5):
        """
        从 /zj_humanoid/upperlimb/joint_states 获取当前 Waist_Z 角度。
        获取失败时返回 0.0，保持静态偏移逻辑可继续执行。
        """
        try:
            msg = rospy.wait_for_message(
                "/zj_humanoid/upperlimb/joint_states",
                JointState,
                timeout=timeout
            )
            idx = msg.name.index("Waist_Z")
            return float(msg.position[idx])
        except (rospy.ROSException, ValueError, IndexError) as e:
            rospy.logwarn("获取 Waist_Z 角度失败: %s，relative_offset 使用未旋转偏移", str(e))
            return 0.0

    def lookup_pose_from_tf(self, base_frame, child_frame, timeout=1.0):
        try:
            self.tf_listener.waitForTransform(
                base_frame,
                child_frame,
                rospy.Time(0),
                rospy.Duration(timeout)
            )
            trans, quat = self.tf_listener.lookupTransform(
                base_frame,
                child_frame,
                rospy.Time(0)
            )
            stamp = self.tf_listener.getLatestCommonTime(base_frame, child_frame)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("获取 TF %s -> %s 失败: %s", base_frame, child_frame, str(e))
            return None, None

        pose = Pose()
        pose.position.x = float(trans[0])
        pose.position.y = float(trans[1])
        pose.position.z = float(trans[2])
        pose.orientation.x = -float(quat[2])
        pose.orientation.y = float(quat[3])
        pose.orientation.z = float(quat[0])
        pose.orientation.w = -float(quat[1])

        return pose, stamp

    def get_current_hand_poses(self, action):
        base_frame = action.get("tf_base_frame", DEFAULT_TF_BASE_FRAME)
        left_frame = action.get("left_hand_frame", DEFAULT_TF_LEFT_HAND_FRAME)
        right_frame = action.get("right_hand_frame", DEFAULT_TF_RIGHT_HAND_FRAME)
        timeout = float(action.get("tf_timeout", 1.0))
        settle_time = float(action.get("tf_settle_time", 0.2))

        if settle_time > 0.0:
            rospy.sleep(settle_time)

        left_pose, left_stamp = self.lookup_pose_from_tf(base_frame, left_frame, timeout=timeout)
        right_pose, right_stamp = self.lookup_pose_from_tf(base_frame, right_frame, timeout=timeout)
        if left_pose is None or right_pose is None:
            return None, None

        rospy.logdebug(
            "当前末端TF left_pos=(%.3f, %.3f, %.3f), left_quat_adjusted=(%.4f, %.4f, %.4f, %.4f), "
            "left_stamp=%.3f, right_pos=(%.3f, %.3f, %.3f), right_quat_adjusted=(%.4f, %.4f, %.4f, %.4f), "
            "right_stamp=%.3f, base=%s",
            left_pose.position.x, left_pose.position.y, left_pose.position.z,
            left_pose.orientation.x, left_pose.orientation.y,
            left_pose.orientation.z, left_pose.orientation.w,
            left_stamp.to_sec(),
            right_pose.position.x, right_pose.position.y, right_pose.position.z,
            right_pose.orientation.x, right_pose.orientation.y,
            right_pose.orientation.z, right_pose.orientation.w,
            right_stamp.to_sec(),
            base_frame
        )
        return left_pose, right_pose

    @staticmethod
    def pose_from_dict(pose_dict):
        pose = Pose()

        pose.position.x = float(pose_dict["position"]["x"])
        pose.position.y = float(pose_dict["position"]["y"])
        pose.position.z = float(pose_dict["position"]["z"])

        pose.orientation.x = float(pose_dict["orientation"]["x"])
        pose.orientation.y = float(pose_dict["orientation"]["y"])
        pose.orientation.z = float(pose_dict["orientation"]["z"])
        pose.orientation.w = float(pose_dict["orientation"]["w"])

        return pose

    @staticmethod
    def pose_position_distance(pose_a, pose_b):
        dx = float(pose_a.position.x) - float(pose_b.position.x)
        dy = float(pose_a.position.y) - float(pose_b.position.y)
        dz = float(pose_a.position.z) - float(pose_b.position.z)
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def pose_orientation_distance(pose_a, pose_b):
        quat_a = [
            float(pose_a.orientation.x),
            float(pose_a.orientation.y),
            float(pose_a.orientation.z),
            float(pose_a.orientation.w),
        ]
        quat_b = [
            float(pose_b.orientation.x),
            float(pose_b.orientation.y),
            float(pose_b.orientation.z),
            float(pose_b.orientation.w),
        ]
        norm_a = math.sqrt(sum(v * v for v in quat_a))
        norm_b = math.sqrt(sum(v * v for v in quat_b))
        if norm_a <= 1e-9 or norm_b <= 1e-9:
            return math.pi

        dot = sum((a / norm_a) * (b / norm_b) for a, b in zip(quat_a, quat_b))
        dot = max(-1.0, min(1.0, abs(dot)))
        return 2.0 * math.acos(dot)

    @staticmethod
    def read_bool_config(action, motion, names, default):
        for owner in (action, motion):
            for name in names:
                if name not in owner:
                    continue
                value = owner.get(name)
                if isinstance(value, bool):
                    return value
                if isinstance(value, str):
                    return value.strip().lower() not in ("0", "false", "no", "off")
                return bool(value)
        return default

    @staticmethod
    def read_float_config(action, motion, names, default):
        for owner in (action, motion):
            for name in names:
                if name in owner:
                    value = owner.get(name)
                    if value is None:
                        continue
                    return float(value)
        return default

    def verify_dual_arm_movel_result(self, action, left_target, right_target, motion):
        verify_enabled = self.read_bool_config(
            action,
            motion,
            ("verify_movel_result", "verify_movel", "verify_execution"),
            True,
        )
        if not verify_enabled:
            rospy.logdebug("MoveL 到位校验已关闭 action_id=%s", action.get("id", ""))
            return True

        is_async = self.read_bool_config(action, motion, ("is_async",), False)
        verify_async = self.read_bool_config(
            action,
            motion,
            ("verify_movel_async", "verify_async"),
            False,
        )
        if is_async and not verify_async:
            rospy.logwarn("MoveL 为异步执行，跳过到位校验 action_id=%s", action.get("id", ""))
            return True

        position_tolerance = self.read_float_config(
            action,
            motion,
            ("movel_position_tolerance", "position_tolerance", "pose_position_tolerance"),
            0.05,
        )
        orientation_tolerance = self.read_float_config(
            action,
            motion,
            ("movel_orientation_tolerance", "orientation_tolerance", "pose_orientation_tolerance"),
            0.35,
        )
        check_orientation = self.read_bool_config(
            action,
            motion,
            ("movel_check_orientation", "check_orientation"),
            False,
        )
        fail_on_unavailable = self.read_bool_config(
            action,
            motion,
            ("movel_fail_on_verify_unavailable", "fail_on_verify_unavailable"),
            True,
        )

        current_left, current_right = self.get_current_hand_poses(action)
        if current_left is None or current_right is None:
            msg = "MoveL 到位校验失败: 无法获取当前末端 TF action_id={}".format(action.get("id", ""))
            if fail_on_unavailable:
                rospy.logerr(msg)
                return False
            rospy.logwarn(msg + "，按配置继续")
            return True

        left_pos_err = self.pose_position_distance(current_left, left_target)
        right_pos_err = self.pose_position_distance(current_right, right_target)
        left_ori_err = self.pose_orientation_distance(current_left, left_target)
        right_ori_err = self.pose_orientation_distance(current_right, right_target)

        rospy.logdebug(
            "MoveL到位校验 action_id=%s: left_pos_err=%.4fm, right_pos_err=%.4fm, "
            "left_ori_err=%.3frad, right_ori_err=%.3frad, pos_tol=%.4fm, ori_tol=%.3frad, check_ori=%d",
            action.get("id", ""),
            left_pos_err,
            right_pos_err,
            left_ori_err,
            right_ori_err,
            position_tolerance,
            orientation_tolerance,
            check_orientation,
        )

        position_ok = left_pos_err <= position_tolerance and right_pos_err <= position_tolerance
        orientation_ok = (
            not check_orientation
            or (left_ori_err <= orientation_tolerance and right_ori_err <= orientation_tolerance)
        )
        if position_ok and orientation_ok:
            return True

        rospy.logerr(
            "MoveL 到位校验失败 action_id=%s: 当前位置与目标不接近，停止后续动作 "
            "(left_pos_err=%.4fm, right_pos_err=%.4fm, left_ori_err=%.3frad, right_ori_err=%.3frad)",
            action.get("id", ""),
            left_pos_err,
            right_pos_err,
            left_ori_err,
            right_ori_err,
        )
        return False

    @staticmethod
    def list_to_joints(joint_list):
        return ControlServiceClient.list_to_joints(joint_list)

    @staticmethod
    def read_relative_position(offset_data):
        offset_data = offset_data or {}
        pose_data = offset_data.get("pose", {}) or {}
        pos_data = offset_data.get("position", pose_data.get("position", {})) or {}

        if pos_data:
            return (
                float(pos_data.get("dx", pos_data.get("x", 0.0))),
                float(pos_data.get("dy", pos_data.get("y", 0.0))),
                float(pos_data.get("dz", pos_data.get("z", 0.0))),
            )

        return (
            float(offset_data.get("dx", 0.0)),
            float(offset_data.get("dy", 0.0)),
            float(offset_data.get("dz", 0.0)),
        )

    @staticmethod
    def parse_relative_offsets(relative_offset):
        """
        支持两种 relative_offset 写法：
        1. 旧格式: {dx: 0.0, dy: 0.0, dz: 0.05}
        2. 新格式: {left: {dx: ...}, right: {dx: ...}}
        """
        if "left" in relative_offset or "right" in relative_offset:
            left_offset = BehaviorTreeExecutor.read_relative_position(relative_offset.get("left", {}))
            right_offset = BehaviorTreeExecutor.read_relative_position(relative_offset.get("right", {}))
        elif "left_arm" in relative_offset or "right_arm" in relative_offset:
            left_offset = BehaviorTreeExecutor.read_relative_position(relative_offset.get("left_arm", {}))
            right_offset = BehaviorTreeExecutor.read_relative_position(relative_offset.get("right_arm", {}))
        else:
            left_offset = BehaviorTreeExecutor.read_relative_position(relative_offset)
            right_offset = left_offset

        return left_offset, right_offset

    @staticmethod
    def read_relative_position_overrides(offset_data):
        """
        读取显式写出的位移轴。区别于 read_relative_position：
        未写的轴不返回，用于 motion_mode=4 让未写轴继续使用自动计算值。
        """
        if not isinstance(offset_data, dict):
            return {}

        pose_data = offset_data.get("pose", {}) or {}
        pos_data = offset_data.get("position", pose_data.get("position", {})) or {}
        overrides = {}

        if pos_data:
            source = pos_data
            key_pairs = (("x", ("dx", "x")), ("y", ("dy", "y")), ("z", ("dz", "z")))
        else:
            source = offset_data
            key_pairs = (("x", ("dx",)), ("y", ("dy",)), ("z", ("dz",)))

        for axis, keys in key_pairs:
            for key in keys:
                if key in source:
                    overrides[axis] = float(source[key])
                    break

        return overrides

    @classmethod
    def parse_mode4_delta_overrides(cls, relative_offset, computed_delta):
        """
        motion_mode=4 中 relative_offset 是轴级覆盖：
        未写轴使用 target_point - current_point，写出的轴使用手动值。
        """
        return cls.parse_mode5_delta_overrides(relative_offset, computed_delta, computed_delta)

    @classmethod
    def parse_mode5_delta_overrides(cls, relative_offset, left_computed_delta, right_computed_delta):
        """
        motion_mode=5 中左右手分别有自动计算位移，relative_offset 只覆盖写出的轴。
        """
        relative_offset = relative_offset or {}
        common_overrides = cls.read_relative_position_overrides(relative_offset)

        def merge_hand_overrides(*keys):
            overrides = dict(common_overrides)
            for key in keys:
                if key in relative_offset:
                    overrides.update(cls.read_relative_position_overrides(relative_offset.get(key, {})))
            return overrides

        def make_delta(computed_delta, overrides):
            values = {
                "x": float(computed_delta[0]),
                "y": float(computed_delta[1]),
                "z": float(computed_delta[2]),
            }
            values.update(overrides)
            return values["x"], values["y"], values["z"]

        left_overrides = merge_hand_overrides("left", "left_arm")
        right_overrides = merge_hand_overrides("right", "right_arm")
        return (
            make_delta(left_computed_delta, left_overrides),
            make_delta(right_computed_delta, right_overrides),
            left_overrides,
            right_overrides,
        )

    @staticmethod
    def normalize_distance_axes(axes):
        if axes is None:
            return ["x", "y"]
        if isinstance(axes, str):
            axes = [axes]

        result = []
        for axis in axes:
            axis_name = str(axis).strip().lower()
            if axis_name in ("x", "y", "z") and axis_name not in result:
                result.append(axis_name)
        return result or ["x", "y"]

    @classmethod
    def read_mode4_skip_threshold(cls, action, pose_source=None, relative_offset=None):
        pose_source = pose_source or {}
        relative_offset = relative_offset or {}
        for owner in (action, pose_source, relative_offset):
            if not isinstance(owner, dict):
                continue
            config = owner.get(
                "skip_if_distance_less_than",
                owner.get("distance_threshold", owner.get("skip_threshold")),
            )
            if config is None:
                continue
            if isinstance(config, dict):
                threshold = config.get("threshold", config.get("distance", config.get("value")))
                if threshold is None:
                    continue
                return float(threshold), cls.normalize_distance_axes(config.get("axes", config.get("axis")))
            return float(config), ["x", "y"]
        return None, []

    @classmethod
    def mode4_distance_for_axes(cls, delta, axes):
        axis_index = {"x": 0, "y": 1, "z": 2}
        return math.sqrt(sum(float(delta[axis_index[axis]]) ** 2 for axis in axes))

    def relative_offset_action_key(self, action):
        return "action_obj:{}".format(id(action))

    @staticmethod
    def relative_offset_sequence(relative_offset):
        if isinstance(relative_offset, list):
            return relative_offset, {}

        if not isinstance(relative_offset, dict):
            return None, {}

        for key in ("actions", "steps", "sequence", "offsets"):
            entries = relative_offset.get(key)
            if isinstance(entries, list):
                common = copy.deepcopy(relative_offset)
                common.pop(key, None)
                return entries, common

        return None, {}

    def select_relative_offset(self, action, relative_offset):
        """
        relative_offset 支持旧的单个偏移字典，也支持列表/steps 多偏移轮询。
        轮询索引按 action 记录，只要选中过当前项就会在动作结束后前进。
        """
        entries, common = self.relative_offset_sequence(relative_offset)
        if entries is None:
            if not isinstance(relative_offset, dict):
                rospy.logerr("relative_offset 必须是 dict 或 list，当前类型: %s", type(relative_offset).__name__)
                return None
            return relative_offset

        if not entries:
            rospy.logerr("relative_offset 多动作列表为空: action_id=%s", action.get("id", ""))
            return None

        action_key = self.relative_offset_action_key(action)
        current = self._relative_offset_indices.get(action_key, 0)
        selected_index = current % len(entries)
        selected = entries[selected_index]
        if not isinstance(selected, dict):
            rospy.logerr(
                "relative_offset 第 %d 项必须是 dict，当前类型: %s",
                selected_index + 1,
                type(selected).__name__,
            )
            return None

        merged = copy.deepcopy(common)
        merged.update(copy.deepcopy(selected))
        self._relative_offset_pending[action_key] = (current + 1, selected_index, len(entries))
        rospy.logdebug(
            "relative_offset 多动作选择 action_id=%s, index=%d/%d",
            action.get("id", ""),
            selected_index + 1,
            len(entries),
        )
        return merged

    def finish_relative_offset_selection(self, action, success):
        action_key = self.relative_offset_action_key(action)
        pending = self._relative_offset_pending.pop(action_key, None)
        if pending is None:
            return

        next_index, selected_index, total = pending
        self._relative_offset_indices[action_key] = next_index
        if success:
            rospy.logdebug(
                "relative_offset 多动作完成 action_id=%s, 已执行=%d/%d, 下次index=%d/%d",
                action.get("id", ""),
                selected_index + 1,
                total,
                (next_index % total) + 1,
                total,
            )
        else:
            rospy.logwarn(
                "relative_offset 多动作执行失败但仍切换 action_id=%s, 已执行=%d/%d, 下次index=%d/%d",
                action.get("id", ""),
                selected_index + 1,
                total,
                (next_index % total) + 1,
                total,
            )

    @staticmethod
    def normalize_hand_name(hand_name):
        hand_name = str(hand_name or "left").strip().lower()
        if hand_name in ("right", "right_arm", "r"):
            return "right"
        return "left"

    @classmethod
    def parse_mode3_hand_offsets(cls, relative_offset):
        """
        motion_mode=3 使用：
        - relative_offset.left/right 同时存在时，左右手分别到达单点加各自偏移。
        - 只存在 left 或 right 时，只移动对应手，另一只手保持当前位姿。
        - 直接写 dx/dy/dz 时，默认移动 left，可用 hand/arm: right 改为右手。
        """
        targets = {}
        if "left" in relative_offset:
            targets["left"] = cls.read_relative_position(relative_offset.get("left", {}))
        if "left_arm" in relative_offset:
            targets["left"] = cls.read_relative_position(relative_offset.get("left_arm", {}))
        if "right" in relative_offset:
            targets["right"] = cls.read_relative_position(relative_offset.get("right", {}))
        if "right_arm" in relative_offset:
            targets["right"] = cls.read_relative_position(relative_offset.get("right_arm", {}))

        if targets:
            return targets

        if (
                "dx" in relative_offset
                or "dy" in relative_offset
                or "dz" in relative_offset
                or "position" in relative_offset
                or "pose" in relative_offset):
            hand_name = cls.normalize_hand_name(
                relative_offset.get("hand", relative_offset.get("arm", "left"))
            )
            return {hand_name: cls.read_relative_position(relative_offset)}

        return {}

    @staticmethod
    def parse_relative_orientations(relative_offset):
        """
        可选读取 relative_offset 中的目标姿态。
        支持:
        left: {orientation: {x: ..., y: ..., z: ..., w: ...}}
        left: {ox: ..., oy: ..., oz: ..., ow: ...}
        未配置时返回 None，表示保持当前姿态。
        """
        def read_orientation(offset_data):
            offset_data = offset_data or {}
            pose_data = offset_data.get("pose", {}) or {}
            orient_data = offset_data.get("orientation", pose_data.get("orientation", offset_data))

            if all(key in orient_data for key in ("x", "y", "z", "w")):
                return (
                    float(orient_data["x"]),
                    float(orient_data["y"]),
                    float(orient_data["z"]),
                    float(orient_data["w"]),
                )

            if all(key in orient_data for key in ("ox", "oy", "oz", "ow")):
                return (
                    float(orient_data["ox"]),
                    float(orient_data["oy"]),
                    float(orient_data["oz"]),
                    float(orient_data["ow"]),
                )

            return None

        if "left" in relative_offset or "right" in relative_offset:
            left_orientation = read_orientation(relative_offset.get("left", {}))
            right_orientation = read_orientation(relative_offset.get("right", {}))
        elif "left_arm" in relative_offset or "right_arm" in relative_offset:
            left_orientation = read_orientation(relative_offset.get("left_arm", {}))
            right_orientation = read_orientation(relative_offset.get("right_arm", {}))
        else:
            orientation = read_orientation(relative_offset)
            left_orientation = orientation
            right_orientation = orientation

        return left_orientation, right_orientation

    @staticmethod
    def rotate_orientation_with_waist(orientation, waist_angle):
        q_waist = tfm.quaternion_about_axis(waist_angle, (0, 0, 1))
        q_rot = tfm.unit_vector(tfm.quaternion_multiply(q_waist, orientation))
        return (
            float(q_rot[0]),
            float(q_rot[1]),
            float(q_rot[2]),
            float(q_rot[3]),
        )

    @staticmethod
    def apply_pose_orientation(pose, orientation, waist_angle=None):
        if orientation is None:
            return False

        if waist_angle is None:
            target_orientation = orientation
        else:
            target_orientation = BehaviorTreeExecutor.rotate_orientation_with_waist(
                orientation,
                waist_angle,
            )

        pose.orientation.x = target_orientation[0]
        pose.orientation.y = target_orientation[1]
        pose.orientation.z = target_orientation[2]
        pose.orientation.w = target_orientation[3]
        return True

    @staticmethod
    def parse_midpoint_offset(relative_offset):
        """
        读取双手中点相对目标点的偏移。
        优先支持 midpoint/center；兼容现有 YAML，默认使用 left。
        """
        if "midpoint" in relative_offset:
            return BehaviorTreeExecutor.read_relative_position(relative_offset.get("midpoint", {})), "midpoint"
        if "center" in relative_offset:
            return BehaviorTreeExecutor.read_relative_position(relative_offset.get("center", {})), "center"
        if "left" in relative_offset:
            return BehaviorTreeExecutor.read_relative_position(relative_offset.get("left", {})), "left"
        if "left_arm" in relative_offset:
            return BehaviorTreeExecutor.read_relative_position(relative_offset.get("left_arm", {})), "left_arm"
        if (
                "dx" in relative_offset
                or "dy" in relative_offset
                or "dz" in relative_offset
                or "position" in relative_offset
                or "pose" in relative_offset):
            return BehaviorTreeExecutor.read_relative_position(relative_offset), "direct"
        if "right" in relative_offset:
            return BehaviorTreeExecutor.read_relative_position(relative_offset.get("right", {})), "right"
        if "right_arm" in relative_offset:
            return BehaviorTreeExecutor.read_relative_position(relative_offset.get("right_arm", {})), "right_arm"
        return (0.0, 0.0, 0.0), "default"

    @staticmethod
    def rotate_xy_offset(offset, theta):
        dx, dy, dz = offset
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        return (
            cos_t * dx - sin_t * dy,
            sin_t * dx + cos_t * dy,
            dz,
        )

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

    @staticmethod
    def normalize_labels(labels):
        if labels is None:
            return []
        if isinstance(labels, str):
            labels = [labels]
        return [str(label).strip() for label in labels if str(label).strip()]

    @staticmethod
    def normalize_point_names(point_names):
        if point_names is None:
            return []
        if isinstance(point_names, str):
            point_names = [point_names]
        return [str(name).strip() for name in point_names if str(name).strip()]

    @staticmethod
    def normalize_motion_mode(motion_mode):
        try:
            mode = int(motion_mode)
        except (TypeError, ValueError):
            mode = 1
        if mode == 0:
            return 1
        return mode

    def apply_joint_sources(self, action, full_joints):
        sources = action.get("joint_sources", action.get("joint_source", []))
        if isinstance(sources, dict):
            sources = [sources]
        if not sources:
            return full_joints
        if not isinstance(sources, list):
            rospy.logerr("joint_sources 格式错误，必须是 dict 或 list")
            return None

        updated = list(full_joints)
        for source in sources:
            if not isinstance(source, dict):
                rospy.logerr("joint_sources 每一项必须是 dict，当前=%s", type(source).__name__)
                return None

            try:
                index = int(source.get("index", source.get("joint_index")))
            except (TypeError, ValueError):
                rospy.logerr("joint_source 缺少有效 index: %s", source)
                return None
            if index < 0:
                index += len(updated)
            if index < 0 or index >= len(updated):
                rospy.logerr("joint_source index=%d 超出 joints 长度=%d", index, len(updated))
                return None

            key = source.get("base_key", source.get("key"))
            point = source.get("point", source.get("field", source.get("name")))
            if not key or not point:
                rospy.logerr("joint_source 需要 base_key/key 和 point/field/name: %s", source)
                return None

            motion_mode = self.normalize_motion_mode(source.get("motion_mode", source.get("mode", 4)))
            left_pose, right_pose = self.get_cached_arm_poses(
                key,
                log_missing_as_error=not bool(source.get("optional", False)),
                point_names=[point],
                motion_mode=motion_mode,
            )
            if left_pose is None or right_pose is None:
                if source.get("optional", False):
                    rospy.logwarn("joint_source 缺少缓存 key=%s point=%s，保留静态关节值", key, point)
                    continue
                return None

            axis = str(source.get("axis", "x")).lower()
            if axis == "x":
                raw_value = left_pose.position.x
            elif axis == "y":
                raw_value = left_pose.position.y
            elif axis == "z":
                raw_value = left_pose.position.z
            else:
                rospy.logerr("joint_source axis 仅支持 x/y/z，当前=%s", axis)
                return None

            value = float(raw_value) * float(source.get("scale", 1.0)) + float(source.get("offset", 0.0))
            if "min" in source:
                value = max(value, float(source["min"]))
            if "max" in source:
                value = min(value, float(source["max"]))

            old_value = updated[index]
            updated[index] = value
            rospy.logdebug(
                "joint_source 应用: index=%d, key=%s, point=%s.%s, raw=%.4f, old=%.4f, new=%.4f",
                index,
                key,
                point,
                axis,
                raw_value,
                old_value,
                value,
            )

        return updated

    @classmethod
    def read_action_point_names(cls, action, pose_source=None):
        pose_source = pose_source or {}
        return cls.normalize_point_names(
            action.get(
                "points",
                action.get("point_names", pose_source.get("points", pose_source.get("point_names", []))),
            )
        )

    @classmethod
    def read_action_motion_mode(cls, action, pose_source=None):
        pose_source = pose_source or {}
        return cls.normalize_motion_mode(
            action.get("motion_mode", action.get("mode", pose_source.get("motion_mode", pose_source.get("mode", 1))))
        )

    @staticmethod
    def _first_present_mapping(source, keys):
        if not isinstance(source, dict):
            return None
        for key in keys:
            value = source.get(key)
            if isinstance(value, dict):
                return value
        return None

    @staticmethod
    def _point_delta_containers(action, pose_source=None):
        pose_source = pose_source or {}
        containers = []
        for owner in (action, pose_source):
            point_delta = owner.get("point_delta") if isinstance(owner, dict) else None
            if isinstance(point_delta, dict):
                containers.append(point_delta)
        containers.extend([action, pose_source])
        return containers

    @classmethod
    def read_mode4_point_sources(cls, action, pose_source=None):
        pose_source = pose_source or {}
        containers = cls._point_delta_containers(action, pose_source)

        target_source = None
        current_source = None
        for container in containers:
            target_source = target_source or cls._first_present_mapping(
                container,
                ("target", "target_point", "target_source"),
            )
            current_source = current_source or cls._first_present_mapping(
                container,
                ("current", "current_point", "current_source", "from", "from_point"),
            )

        if target_source is not None and current_source is not None:
            return target_source, current_source

        for owner in (action, pose_source):
            if not isinstance(owner, dict):
                continue
            for key in ("point_pairs", "point_sources", "mode4_points", "delta_points"):
                entries = owner.get(key)
                if isinstance(entries, list) and len(entries) >= 2:
                    if isinstance(entries[0], dict) and isinstance(entries[1], dict):
                        return entries[0], entries[1]

        return None, None

    @classmethod
    def read_mode5_point_sources(cls, action, pose_source=None):
        pose_source = pose_source or {}
        containers = cls._point_delta_containers(action, pose_source)

        left_current = None
        right_current = None
        left_target = None
        right_target = None
        for container in containers:
            left_current = left_current or cls._first_present_mapping(
                container,
                ("left_current", "current_left", "left_current_point", "current_left_point", "left_from", "from_left"),
            )
            right_current = right_current or cls._first_present_mapping(
                container,
                ("right_current", "current_right", "right_current_point", "current_right_point", "right_from", "from_right"),
            )
            left_target = left_target or cls._first_present_mapping(
                container,
                ("left_target", "target_left", "left_target_point", "target_left_point", "left_to", "to_left"),
            )
            right_target = right_target or cls._first_present_mapping(
                container,
                ("right_target", "target_right", "right_target_point", "target_right_point", "right_to", "to_right"),
            )

        if all(source is not None for source in (left_current, right_current, left_target, right_target)):
            return left_current, right_current, left_target, right_target

        for owner in (action, pose_source):
            if not isinstance(owner, dict):
                continue
            for key in ("point_pairs", "mode5_points"):
                entries = owner.get(key)
                if isinstance(entries, list) and len(entries) >= 4:
                    if all(isinstance(entry, dict) for entry in entries[:4]):
                        return entries[0], entries[1], entries[2], entries[3]

        return None, None, None, None

    @staticmethod
    def target_cache_key(key, point_names=None, motion_mode=1):
        point_names = point_names or []
        if not point_names:
            return key
        return "{}|mode={}|points={}".format(
            key,
            int(motion_mode),
            ",".join(point_names),
        )

    def clear_pose_cache_for_key(self, key):
        prefix = "{}|".format(key)
        keys_to_remove = [
            cache_key
            for cache_key in self._pose_cache
            if cache_key == key
            or cache_key.startswith(prefix)
            or cache_key == CENTER_CACHE_PREFIX + key
        ]
        for cache_key in keys_to_remove:
            self._pose_cache.pop(cache_key, None)

        if keys_to_remove:
            rospy.logdebug(
                "视觉重新识别前已清理旧位姿缓存 key=%s, count=%d",
                key,
                len(keys_to_remove),
            )

    @staticmethod
    def get_action_ref_joints(action):
        joints_data = action.get("joints", [])

        if not joints_data:
            raise ValueError("IK 动作缺少 joints 参考关节角")

        ref_group = joints_data[0]
        if len(ref_group) < 3:
            raise ValueError("joints 格式错误，应包含左臂、右臂、其他关节三组")

        left_ref = ref_group[0]
        right_ref = ref_group[1]
        other_ref = ref_group[2]

        if len(left_ref) != 7:
            raise ValueError("左臂参考关节角不是7维")

        if len(right_ref) != 7:
            raise ValueError("右臂参考关节角不是7维")

        return left_ref, right_ref, other_ref

    def call_ik(self, ik_service_name, target_pose, q7_ref):
        return self.services.call_ik(ik_service_name, target_pose, q7_ref)

    def set_offset_by_id(self, offset_id):
        if offset_id not in self.offset_map:
            rospy.logerr("未知 offset_id: %s, 可用: %s", offset_id, list(self.offset_map.keys()))
            return False

        entry = self.offset_map[offset_id]
        return self.services.set_offset(entry, offset_id=offset_id)

    def call_movej_whole_body(self, service_name, joints, motion):
        return self.services.call_movej_whole_body(service_name, joints, motion)

    def call_movej_by_path_whole_body(self, service_name, path, motion):
        return self.services.call_movej_by_path_whole_body(service_name, path, motion)

    def execute_movej_static_action(self, action):
        """使用 YAML 中的静态关节角度调用 MoveJ，不进行 IK"""
        service_name = action["service"]
        motion = action.get("motion", {})
        joints_data = action.get("joints", None)

        if joints_data is None:
            rospy.logerr("静态关节动作缺少 'joints' 字段")
            return False

        # ---- 规范化：支持两种写法 ----
        if isinstance(joints_data, list):
            # 写法1：[[左臂], [右臂], [其他]] -> 展平为一维
            if all(isinstance(x, list) for x in joints_data):
                full_joints = self.flatten_joint_group(joints_data)
            else:
                # 写法2：直接给一维列表 [v1, v2, ...]
                full_joints = [float(x) for x in joints_data]
        else:
            rospy.logerr("joints 格式错误，必须是 list")
            return False

        full_joints = self.apply_joint_sources(action, full_joints)
        if full_joints is None:
            return False

        rospy.logdebug("使用静态关节值，关节总数: %d", len(full_joints))
        move_ok = self.call_movej_whole_body(service_name, full_joints, motion)
        if not move_ok:
            return False

        if action.get("pose_source") or action.get("base_key"):
            left_pose, right_pose = self.resolve_arm_poses(action)
            if left_pose is None or right_pose is None:
                self.finish_relative_offset_selection(action, False)
                return False
            self.finish_relative_offset_selection(action, True)
            rospy.logdebug(
                "movej(use_ik=false) 已完成静态 joints 运动，随后解析目标位姿并更新缓存: "
                "left_pos=(%.3f, %.3f, %.3f), right_pos=(%.3f, %.3f, %.3f)",
                left_pose.position.x, left_pose.position.y, left_pose.position.z,
                right_pose.position.x, right_pose.position.y, right_pose.position.z
            )

        return True

    def execute_ik_move_action(self, action):
        service_name = action["service"]
        motion = action.get("motion", {})
        arm_poses = action.get("arm_poses", {})

        if "left_arm" not in arm_poses or "right_arm" not in arm_poses:
            rospy.logerr("IK 动作缺少 arm_poses.left_arm 或 arm_poses.right_arm")
            return False

        left_pose = self.pose_from_dict(arm_poses["left_arm"]["pose"])
        right_pose = self.pose_from_dict(arm_poses["right_arm"]["pose"])

        try:
            left_ref, right_ref, other_ref = self.get_action_ref_joints(action)
        except ValueError as exc:
            rospy.logerr(str(exc))
            return False

        left_q7_ref = left_ref[6]
        right_q7_ref = right_ref[6]

        rospy.logdebug("开始左臂 IK，q7_ref={:.4f}".format(left_q7_ref))
        left_ik_joints = self.services.call_left_ik(left_pose, left_q7_ref)

        rospy.logdebug("开始右臂 IK，q7_ref={:.4f}".format(right_q7_ref))
        right_ik_joints = self.services.call_right_ik(right_pose, right_q7_ref)

        if left_ik_joints is None or right_ik_joints is None:
            rospy.logwarn("动作 {} IK 失败，跳过".format(action.get("name", "")))
            return False

        left_joint_list = list(left_ik_joints.joint)
        right_joint_list = list(right_ik_joints.joint)

        if len(left_joint_list) != 7:
            rospy.logwarn("左臂 IK 结果不是7维，当前长度: {}".format(len(left_joint_list)))
            return False

        if len(right_joint_list) != 7:
            rospy.logwarn("右臂 IK 结果不是7维，当前长度: {}".format(len(right_joint_list)))
            return False

        full_joints = []
        full_joints.extend(left_joint_list)
        full_joints.extend(right_joint_list)
        full_joints.extend(other_ref)

        rospy.logdebug("IK 动作生成完整关节向量长度: {}".format(len(full_joints)))
        return self.call_movej_whole_body(service_name, full_joints, motion)

    def execute_path_action(self, action):
        service_name = action["service"]
        motion = action.get("motion", {})
        path = action.get("path", [])

        if not path:
            rospy.logwarn("轨迹动作 {} 缺少 path".format(action.get("name", "")))
            return False

        rospy.logdebug("开始执行轨迹动作: {}, 轨迹点数量: {}".format(
            action.get("name", ""),
            len(path),
        ))
        return self.call_movej_by_path_whole_body(service_name, path, motion)

    def execute_dual_arm_moveL_action(self, action):
        service_name = action["service"]
        motion = action.get("motion", {})
        action.pop("_skip_motion", None)

        left_pose, right_pose = self.resolve_arm_poses(action)
        if left_pose is None or right_pose is None:
            self.finish_relative_offset_selection(action, False)
            return False

        skip_reason = action.pop("_skip_motion", None)
        if skip_reason:
            rospy.logdebug("跳过 MoveL 动作 action_id=%s: %s", action.get("id", ""), skip_reason)
            self.finish_relative_offset_selection(action, True)
            return True

        ok = self.services.call_dual_arm_movel(service_name, left_pose, right_pose, motion)
        if ok:
            ok = self.verify_dual_arm_movel_result(action, left_pose, right_pose, motion)
        self.finish_relative_offset_selection(action, ok)
        return ok

    def execute_hand_action(self, action):
        service_name = action["service"]
        offset_id = action.get("offset_id", None)
        q_values = action.get("q", [])

        if not q_values or len(q_values) != 12:
            rospy.logerr("手掌动作缺少有效的 q 数组（长度 12）")
            return False

        # ---- 如果指定了 offset_id，先设置补偿 ----
        if offset_id is not None:
            if not self.set_offset_by_id(offset_id):
                rospy.logwarn("设置偏移参数失败，但继续执行手掌动作")

        return self.services.call_hand_joint(service_name, q_values)

    def trigger_and_cache_detection(self, detect_config, default_key=""):
        key = detect_config.get("key") or detect_config.get("cache_key") or default_key
        if not key:
            rospy.logerr("视觉动作缺少 key")
            return False

        offset_id = detect_config.get("offset_id", None)
        if offset_id:
            if not self.set_offset_by_id(offset_id):
                rospy.logwarn("设置 offset_id=%s 失败，继续尝试触发视觉", offset_id)

        trigger_value = detect_config.get("trigger_value", 2)
        labels = self.normalize_labels(detect_config.get("labels", detect_config.get("label", [])))
        point_names = self.normalize_point_names(
            detect_config.get("points", detect_config.get("point_names", []))
        )
        has_motion_mode = "motion_mode" in detect_config or "mode" in detect_config
        motion_mode = self.normalize_motion_mode(detect_config.get("motion_mode", detect_config.get("mode", 1)))
        if not has_motion_mode and len(point_names) == 1:
            motion_mode = 4
            rospy.logdebug(
                "视觉动作单点 points=%s 未配置 motion_mode，自动按单点模式缓存",
                point_names,
            )
        timeout = float(detect_config.get("timeout", 1.0))

        trigger_resp = self.services.trigger_detect(
            key,
            trigger_value=trigger_value,
            labels=labels,
            point_names=point_names,
            motion_mode=motion_mode,
            timeout=timeout,
        )
        if trigger_resp is None:
            rospy.logwarn("触发动态识别失败: 服务无响应")
            return False

        if not trigger_resp.success:
            rospy.logwarn("触发动态识别失败: %s", trigger_resp.message)
            return False

        self.clear_pose_cache_for_key(key)

        cache_key = self.target_cache_key(key, point_names, motion_mode)
        self._pose_cache[cache_key] = (trigger_resp.left_pose, trigger_resp.right_pose)
        if cache_key != key:
            rospy.logdebug("视觉动作选点结果已缓存 cache_key=%s", cache_key)
        if int(trigger_value) == 1:
            self._pose_cache[CENTER_CACHE_PREFIX + key] = (
                trigger_resp.left_pose,
                trigger_resp.right_pose,
            )

        rospy.logdebug(
            "视觉识别结果已缓存 key=%s, trigger_value=%s, labels=%s, motion_mode=%s, points=%s",
            key,
            trigger_value,
            labels,
            motion_mode,
            point_names,
        )
        return True

    def execute_vision_action(self, action):
        detect_config = action.get("vision", None)
        if detect_config is None:
            detect_config = action
        ok = self.trigger_and_cache_detection(detect_config, default_key=action.get("id", ""))
        if ok:
            return True
        if action.get("optional", False) or action.get("continue_on_fail", False):
            rospy.logwarn("视觉动作失败，但 optional=True，继续执行后续动作")
            return True
        return False

    def execute_open_loop_motion_action(self, action):
        return self.services.call_open_loop_motion(action)

    def execute_action(self, action):
        action_id = action.get("id", "")
        action_name = action.get("name", "")
        action_label = action.get("label", "")

        rospy.logdebug("========== 执行动作 {} {} {} ==========".format(
            action_id, action_name, action_label))

        action_type = str(action.get("type", "")).lower()
        if action_type in ("vision", "detect", "vision_detect", "target_detect"):
            return self.execute_vision_action(action)
        if action_type in ("open_loop", "open_loop_motion", "base_open_loop", "base_motion"):
            return self.execute_open_loop_motion_action(action)

        service_name = action.get("service", "")
        if not service_name:
            msg = "动作 {} 没有 service 字段".format(action_name)
            if self.config.get("skip_unknown_action", True):
                rospy.logwarn(msg + "，已跳过")
                return True
            rospy.logerr(msg)
            return False

        # 手掌关节动作 (hand joint_switch)
        if "joint_switch" in service_name:
            return self.execute_hand_action(action)

        # 关节轨迹运动 (movej_by_path)
        if "movej_by_path" in service_name:
            return self.execute_path_action(action)

        if "movej" in service_name:
            use_ik = action.get("use_ik", True)   # 默认 True，兼容旧配置
            if use_ik:
                return self.execute_ik_move_action(action)
            else:
                return self.execute_movej_static_action(action)

        # 双臂直线运动 (movel)
        if "movel" in service_name.lower():
            return self.execute_dual_arm_moveL_action(action)

        # 未识别的动作
        msg = "动作 {} 类型无法识别 (service: {})".format(action_name, service_name)
        if self.config.get("skip_unknown_action", True):
            rospy.logwarn(msg + "，已跳过")
            return True
        rospy.logerr(msg)
        return False

    def get_cached_arm_poses(self, key, log_missing_as_error=True, point_names=None, motion_mode=1):
        point_names = self.normalize_point_names(point_names)
        motion_mode = self.normalize_motion_mode(motion_mode)
        cache_key = self.target_cache_key(key, point_names, motion_mode)

        if cache_key in self._pose_cache:
            left_pose, right_pose = self._pose_cache[cache_key]
            rospy.logdebug("使用本地缓存位姿 cache_key=%s", cache_key)
            return left_pose, right_pose

        resp = self.services.get_target(
            key,
            timeout=1.0,
            motion_mode=motion_mode,
            point_names=point_names,
        )
        if resp is None:
            return None, None

        if not resp.success:
            if log_missing_as_error:
                rospy.logerr("获取缓存位姿失败 cache_key=%s: %s", cache_key, resp.message)
            else:
                rospy.logdebug("缓存位姿不存在 cache_key=%s: %s", cache_key, resp.message)
            return None, None
        left_pose = resp.left_pose
        right_pose = resp.right_pose
        self._pose_cache[cache_key] = (left_pose, right_pose)
        rospy.logdebug("从服务首次获取并缓存 cache_key=%s", cache_key)
        return left_pose, right_pose

    def get_center_or_cached_arm_poses(self, key, point_names=None, motion_mode=1):
        point_names = self.normalize_point_names(point_names)
        motion_mode = self.normalize_motion_mode(motion_mode)

        if point_names:
            left_pose, right_pose = self.get_cached_arm_poses(
                key,
                point_names=point_names,
                motion_mode=motion_mode,
            )
            return left_pose, right_pose, "selected_points:{}".format(",".join(point_names))

        center_key = CENTER_CACHE_PREFIX + key
        center_left, center_right = self.get_cached_arm_poses(
            center_key,
            log_missing_as_error=False,
        )
        if center_left is not None and center_right is not None:
            rospy.logdebug("base_key=%s 优先使用视觉中心点缓存 center_key=%s", key, center_key)
            return center_left, center_right, "center"

        rospy.logwarn("未找到视觉中心点缓存 center_key=%s，回退使用 base_key=%s 的左右点中点", center_key, key)
        left_pose, right_pose = self.get_cached_arm_poses(key)
        return left_pose, right_pose, "left_right_midpoint"

    def apply_relative_offset_from_current_midpoint(
            self,
            action,
            base_key,
            relative_offset,
            cache_key=None,
            point_names=None,
            motion_mode=1):
        base_left, base_right, base_source = self.get_center_or_cached_arm_poses(
            base_key,
            point_names=point_names,
            motion_mode=motion_mode,
        )
        if base_left is None or base_right is None:
            return None, None

        current_left, current_right = self.get_current_hand_poses(action)
        if current_left is None or current_right is None:
            rospy.logerr("获取当前末端位姿失败，无法基于当前双手中点执行 base_key=%s 的相对偏移", base_key)
            return None, None

        base_mid = (
            (base_left.position.x + base_right.position.x) * 0.5,
            (base_left.position.y + base_right.position.y) * 0.5,
            (base_left.position.z + base_right.position.z) * 0.5,
        )
        current_mid = (
            (current_left.position.x + current_right.position.x) * 0.5,
            (current_left.position.y + current_right.position.y) * 0.5,
            (current_left.position.z + current_right.position.z) * 0.5,
        )

        current_waist = self.get_current_waist_angle()
        midpoint_offset, offset_source = self.parse_midpoint_offset(relative_offset)
        left_orientation, right_orientation = self.parse_relative_orientations(relative_offset)
        rotate_with_waist = bool(relative_offset.get("rotate_with_waist", True))

        if rotate_with_waist:
            theta = current_waist
            midpoint_rot = self.rotate_xy_offset(midpoint_offset, theta)
            rospy.logdebug(
                "基于当前双手中点应用腰部旋转偏移 key=%s, offset_source=%s, "
                "offset_raw=(%.3f, %.3f, %.3f), offset_rotated=(%.3f, %.3f, %.3f), waist=%.3f",
                base_key,
                offset_source,
                midpoint_offset[0], midpoint_offset[1], midpoint_offset[2],
                midpoint_rot[0], midpoint_rot[1], midpoint_rot[2],
                theta
            )
        else:
            midpoint_rot = midpoint_offset
            rospy.logdebug(
                "基于当前双手中点应用base坐标偏移 key=%s, offset_source=%s, offset=(%.3f, %.3f, %.3f)",
                base_key,
                offset_source,
                midpoint_rot[0], midpoint_rot[1], midpoint_rot[2]
            )

        target_mid = (
            base_mid[0] + midpoint_rot[0],
            base_mid[1] + midpoint_rot[1],
            base_mid[2] + midpoint_rot[2],
        )
        move_delta = (
            target_mid[0] - current_mid[0],
            target_mid[1] - current_mid[1],
            target_mid[2] - current_mid[2],
        )

        new_left = copy.deepcopy(current_left)
        new_right = copy.deepcopy(current_right)

        new_left.position.x += move_delta[0]
        new_left.position.y += move_delta[1]
        new_left.position.z += move_delta[2]
        new_right.position.x += move_delta[0]
        new_right.position.y += move_delta[1]
        new_right.position.z += move_delta[2]

        orientation_waist = current_waist if rotate_with_waist else None
        left_has_orientation = self.apply_pose_orientation(
            new_left,
            left_orientation,
            waist_angle=orientation_waist,
        )
        right_has_orientation = self.apply_pose_orientation(
            new_right,
            right_orientation,
            waist_angle=orientation_waist,
        )
        if left_has_orientation or right_has_orientation:
            rospy.logdebug(
                "双手中点相对偏移应用目标姿态 left_has_orientation=%s, right_has_orientation=%s, rotate_with_waist=%s",
                left_has_orientation,
                right_has_orientation,
                rotate_with_waist
            )

        if cache_key:
            self._pose_cache[cache_key] = (new_left, new_right)
            rospy.logdebug("相对偏移结果已缓存 key=%s, base_key=%s", cache_key, base_key)

        rospy.logdebug(
            "双手中点移动 base_source=%s, base_mid=(%.3f, %.3f, %.3f), current_mid=(%.3f, %.3f, %.3f), "
            "target_mid=(%.3f, %.3f, %.3f), delta=(%.3f, %.3f, %.3f)",
            base_source,
            base_mid[0], base_mid[1], base_mid[2],
            current_mid[0], current_mid[1], current_mid[2],
            target_mid[0], target_mid[1], target_mid[2],
            move_delta[0], move_delta[1], move_delta[2]
        )
        rospy.logdebug(
            "保持双手相对位姿后 left_pos=(%.3f, %.3f, %.3f), left_quat=(%.4f, %.4f, %.4f, %.4f), "
            "right_pos=(%.3f, %.3f, %.3f), right_quat=(%.4f, %.4f, %.4f, %.4f)",
            new_left.position.x, new_left.position.y, new_left.position.z,
            new_left.orientation.x, new_left.orientation.y,
            new_left.orientation.z, new_left.orientation.w,
            new_right.position.x, new_right.position.y, new_right.position.z,
            new_right.orientation.x, new_right.orientation.y,
            new_right.orientation.z, new_right.orientation.w
        )
        return new_left, new_right

    def apply_relative_offset_from_single_point(
            self,
            action,
            base_key,
            relative_offset,
            cache_key=None,
            point_names=None):
        base_left, base_right = self.get_cached_arm_poses(
            base_key,
            point_names=point_names,
            motion_mode=3,
        )
        if base_left is None or base_right is None:
            return None, None

        current_left, current_right = self.get_current_hand_poses(action)
        if current_left is None or current_right is None:
            rospy.logerr("获取当前末端位姿失败，无法执行 motion_mode=3")
            return None, None

        base_point = base_left
        target_offsets = self.parse_mode3_hand_offsets(relative_offset)
        if not target_offsets:
            rospy.logerr("motion_mode=3 需要 relative_offset 至少指定一个手的偏移")
            return None, None

        current_waist = self.get_current_waist_angle()
        rotate_with_waist = bool(relative_offset.get("rotate_with_waist", True))
        left_orientation, right_orientation = self.parse_relative_orientations(relative_offset)

        new_left = copy.deepcopy(current_left)
        new_right = copy.deepcopy(current_right)

        def apply_target(hand_name, pose, offset, orientation):
            if rotate_with_waist:
                target_offset = self.rotate_xy_offset(offset, current_waist)
            else:
                target_offset = offset

            pose.position.x = base_point.position.x + target_offset[0]
            pose.position.y = base_point.position.y + target_offset[1]
            pose.position.z = base_point.position.z + target_offset[2]
            self.apply_pose_orientation(
                pose,
                orientation,
                waist_angle=current_waist if rotate_with_waist else None,
            )
            rospy.logdebug(
                "motion_mode=3 %s手目标: base=(%.3f, %.3f, %.3f), offset_raw=(%.3f, %.3f, %.3f), "
                "target=(%.3f, %.3f, %.3f), rotate_with_waist=%s",
                hand_name,
                base_point.position.x,
                base_point.position.y,
                base_point.position.z,
                offset[0],
                offset[1],
                offset[2],
                pose.position.x,
                pose.position.y,
                pose.position.z,
                rotate_with_waist,
            )

        if "left" in target_offsets:
            apply_target("左", new_left, target_offsets["left"], left_orientation)
        else:
            rospy.logdebug("motion_mode=3 未指定左手偏移，左手保持当前位姿")

        if "right" in target_offsets:
            apply_target("右", new_right, target_offsets["right"], right_orientation)
        else:
            rospy.logdebug("motion_mode=3 未指定右手偏移，右手保持当前位姿")

        if cache_key:
            self._pose_cache[cache_key] = (new_left, new_right)
            rospy.logdebug("motion_mode=3 结果已缓存 key=%s, base_key=%s", cache_key, base_key)

        return new_left, new_right

    def get_mode4_point_pose(self, source, role_name, motion_mode=4):
        if not isinstance(source, dict):
            rospy.logerr("motion_mode=%d %s点源必须是 dict，当前=%s", motion_mode, role_name, type(source).__name__)
            return None

        key = source.get("base_key", source.get("key", source.get("cache_key")))
        if not key:
            rospy.logerr("motion_mode=%d %s点源缺少 base_key/key: %s", motion_mode, role_name, source)
            return None

        point_names = self.normalize_point_names(source.get("points", source.get("point_names", [])))
        if not point_names:
            point_names = ["center"]
        left_pose, right_pose = self.get_cached_arm_poses(
            key,
            point_names=point_names,
            motion_mode=motion_mode,
        )
        if left_pose is None or right_pose is None:
            rospy.logerr("motion_mode=%d 获取%s点失败 key=%s, points=%s", motion_mode, role_name, key, point_names)
            return None

        pose = copy.deepcopy(left_pose)
        pose.position.x = (left_pose.position.x + right_pose.position.x) * 0.5
        pose.position.y = (left_pose.position.y + right_pose.position.y) * 0.5
        pose.position.z = (left_pose.position.z + right_pose.position.z) * 0.5
        rospy.logdebug(
            "motion_mode=%d %s点 key=%s, points=%s, pos=(%.3f, %.3f, %.3f)",
            motion_mode,
            role_name,
            key,
            point_names,
            pose.position.x,
            pose.position.y,
            pose.position.z,
        )
        return pose

    def apply_relative_offset_from_point_delta(
            self,
            action,
            relative_offset,
            cache_key=None,
            pose_source=None):
        target_source, current_source = self.read_mode4_point_sources(action, pose_source)
        if target_source is None or current_source is None:
            rospy.logerr(
                "motion_mode=4 需要配置 point_delta.target 和 point_delta.current，"
                "或 point_pairs 前两项分别为目标点/当前点"
            )
            return None, None

        target_pose = self.get_mode4_point_pose(target_source, "目标")
        current_point_pose = self.get_mode4_point_pose(current_source, "当前")
        if target_pose is None or current_point_pose is None:
            return None, None

        current_left, current_right = self.get_current_hand_poses(action)
        if current_left is None or current_right is None:
            rospy.logerr("获取当前末端位姿失败，无法执行 motion_mode=4")
            return None, None

        computed_delta = (
            target_pose.position.x - current_point_pose.position.x,
            target_pose.position.y - current_point_pose.position.y,
            target_pose.position.z - current_point_pose.position.z,
        )
        skip_threshold, skip_axes = self.read_mode4_skip_threshold(
            action,
            pose_source=pose_source,
            relative_offset=relative_offset,
        )
        if skip_threshold is not None:
            distance = self.mode4_distance_for_axes(computed_delta, skip_axes)
            rospy.logdebug(
                "motion_mode=4 跳过阈值检查 axes=%s, distance=%.4f, threshold=%.4f",
                skip_axes,
                distance,
                skip_threshold,
            )
            if distance < skip_threshold:
                action["_skip_motion"] = (
                    "motion_mode=4 {}轴距离 {:.4f}m 小于阈值 {:.4f}m".format(
                        "/".join(skip_axes),
                        distance,
                        skip_threshold,
                    )
                )
                return copy.deepcopy(target_pose), copy.deepcopy(current_point_pose)

        left_delta, right_delta, left_overrides, right_overrides = self.parse_mode4_delta_overrides(
            relative_offset,
            computed_delta,
        )

        new_left = copy.deepcopy(current_left)
        new_right = copy.deepcopy(current_right)

        new_left.position.x += left_delta[0]
        new_left.position.y += left_delta[1]
        new_left.position.z += left_delta[2]
        new_right.position.x += right_delta[0]
        new_right.position.y += right_delta[1]
        new_right.position.z += right_delta[2]

        left_orientation, right_orientation = self.parse_relative_orientations(relative_offset)
        rotate_orientation = bool(relative_offset.get("rotate_orientation_with_waist", False))
        orientation_waist = self.get_current_waist_angle() if rotate_orientation else None
        left_has_orientation = self.apply_pose_orientation(
            new_left,
            left_orientation,
            waist_angle=orientation_waist,
        )
        right_has_orientation = self.apply_pose_orientation(
            new_right,
            right_orientation,
            waist_angle=orientation_waist,
        )

        if cache_key:
            self._pose_cache[cache_key] = (new_left, new_right)
            rospy.logdebug("motion_mode=4 结果已缓存 key=%s", cache_key)

        rospy.logdebug(
            "motion_mode=4 视觉差值 computed=(%.3f, %.3f, %.3f), "
            "left_delta=(%.3f, %.3f, %.3f), right_delta=(%.3f, %.3f, %.3f), "
            "left_override_axes=%s, right_override_axes=%s",
            computed_delta[0],
            computed_delta[1],
            computed_delta[2],
            left_delta[0],
            left_delta[1],
            left_delta[2],
            right_delta[0],
            right_delta[1],
            right_delta[2],
            sorted(left_overrides.keys()),
            sorted(right_overrides.keys()),
        )
        if left_has_orientation or right_has_orientation:
            rospy.logdebug(
                "motion_mode=4 应用目标姿态 left_has_orientation=%s, right_has_orientation=%s, rotate_orientation=%s",
                left_has_orientation,
                right_has_orientation,
                rotate_orientation,
            )

        return new_left, new_right

    def apply_relative_offset_from_hand_targets(
            self,
            action,
            relative_offset,
            cache_key=None,
            pose_source=None):
        left_current_source, right_current_source, left_target_source, right_target_source = (
            self.read_mode5_point_sources(action, pose_source)
        )
        if None in (left_current_source, right_current_source, left_target_source, right_target_source):
            rospy.logerr(
                "motion_mode=5 需要配置 point_delta.left_current/right_current 和 "
                "point_delta.left_target/right_target"
            )
            return None, None

        left_current_point = self.get_mode4_point_pose(left_current_source, "左手当前", motion_mode=5)
        right_current_point = self.get_mode4_point_pose(right_current_source, "右手当前", motion_mode=5)
        left_target = self.get_mode4_point_pose(left_target_source, "左手目标", motion_mode=5)
        right_target = self.get_mode4_point_pose(right_target_source, "右手目标", motion_mode=5)
        if None in (left_current_point, right_current_point, left_target, right_target):
            return None, None

        left_computed_delta = (
            left_target.position.x - left_current_point.position.x,
            left_target.position.y - left_current_point.position.y,
            left_target.position.z - left_current_point.position.z,
        )
        right_computed_delta = (
            right_target.position.x - right_current_point.position.x,
            right_target.position.y - right_current_point.position.y,
            right_target.position.z - right_current_point.position.z,
        )

        skip_threshold, skip_axes = self.read_mode4_skip_threshold(
            action,
            pose_source=pose_source,
            relative_offset=relative_offset,
        )
        if skip_threshold is not None:
            left_distance = self.mode4_distance_for_axes(left_computed_delta, skip_axes)
            right_distance = self.mode4_distance_for_axes(right_computed_delta, skip_axes)
            rospy.logdebug(
                "motion_mode=5 跳过阈值检查 axes=%s, left_distance=%.4f, right_distance=%.4f, threshold=%.4f",
                skip_axes,
                left_distance,
                right_distance,
                skip_threshold,
            )
            if left_distance < skip_threshold and right_distance < skip_threshold:
                action["_skip_motion"] = (
                    "motion_mode=5 左/右手{}轴距离 {:.4f}/{:.4f}m 均小于阈值 {:.4f}m".format(
                        "/".join(skip_axes),
                        left_distance,
                        right_distance,
                        skip_threshold,
                    )
                )
                return copy.deepcopy(left_target), copy.deepcopy(right_target)

        current_left, current_right = self.get_current_hand_poses(action)
        if current_left is None or current_right is None:
            rospy.logerr("获取当前末端位姿失败，无法执行 motion_mode=5")
            return None, None

        left_delta, right_delta, left_overrides, right_overrides = self.parse_mode5_delta_overrides(
            relative_offset,
            left_computed_delta,
            right_computed_delta,
        )

        new_left = copy.deepcopy(current_left)
        new_right = copy.deepcopy(current_right)
        new_left.position.x += left_delta[0]
        new_left.position.y += left_delta[1]
        new_left.position.z += left_delta[2]
        new_right.position.x += right_delta[0]
        new_right.position.y += right_delta[1]
        new_right.position.z += right_delta[2]

        left_orientation, right_orientation = self.parse_relative_orientations(relative_offset)
        rotate_orientation = bool(relative_offset.get("rotate_orientation_with_waist", False))
        orientation_waist = self.get_current_waist_angle() if rotate_orientation else None
        left_has_orientation = self.apply_pose_orientation(
            new_left,
            left_orientation,
            waist_angle=orientation_waist,
        )
        right_has_orientation = self.apply_pose_orientation(
            new_right,
            right_orientation,
            waist_angle=orientation_waist,
        )

        if cache_key:
            self._pose_cache[cache_key] = (new_left, new_right)
            rospy.logdebug("motion_mode=5 结果已缓存 key=%s", cache_key)

        rospy.logdebug(
            "motion_mode=5 视觉点差 left_computed=(%.3f, %.3f, %.3f), right_computed=(%.3f, %.3f, %.3f), "
            "left_delta=(%.3f, %.3f, %.3f), right_delta=(%.3f, %.3f, %.3f), "
            "left_override_axes=%s, right_override_axes=%s",
            left_computed_delta[0],
            left_computed_delta[1],
            left_computed_delta[2],
            right_computed_delta[0],
            right_computed_delta[1],
            right_computed_delta[2],
            left_delta[0],
            left_delta[1],
            left_delta[2],
            right_delta[0],
            right_delta[1],
            right_delta[2],
            sorted(left_overrides.keys()),
            sorted(right_overrides.keys()),
        )
        if left_has_orientation or right_has_orientation:
            rospy.logdebug(
                "motion_mode=5 应用目标姿态 left_has_orientation=%s, right_has_orientation=%s, rotate_orientation=%s",
                left_has_orientation,
                right_has_orientation,
                rotate_orientation,
            )

        return new_left, new_right

    def apply_relative_offset_from_current_hands(self, action, relative_offset):
        current_left, current_right = self.get_current_hand_poses(action)
        if current_left is None or current_right is None:
            rospy.logerr("获取当前末端位姿失败，无法执行普通相对偏移")
            return None, None

        current_waist = self.get_current_waist_angle()
        left_offset, right_offset = self.parse_relative_offsets(relative_offset)
        left_orientation, right_orientation = self.parse_relative_orientations(relative_offset)
        rotate_with_waist = bool(relative_offset.get("rotate_with_waist", True))

        if rotate_with_waist:
            theta = current_waist
            left_rot = self.rotate_xy_offset(left_offset, theta)
            right_rot = self.rotate_xy_offset(right_offset, theta)
            rospy.logdebug(
                "普通相对偏移应用腰部旋转 left_raw=(%.3f, %.3f, %.3f), left_rotated=(%.3f, %.3f, %.3f), "
                "right_raw=(%.3f, %.3f, %.3f), right_rotated=(%.3f, %.3f, %.3f), waist=%.3f",
                left_offset[0], left_offset[1], left_offset[2],
                left_rot[0], left_rot[1], left_rot[2],
                right_offset[0], right_offset[1], right_offset[2],
                right_rot[0], right_rot[1], right_rot[2],
                theta
            )
        else:
            left_rot = left_offset
            right_rot = right_offset
            rospy.logdebug(
                "普通相对偏移使用base坐标 left=(%.3f, %.3f, %.3f), right=(%.3f, %.3f, %.3f)",
                left_rot[0], left_rot[1], left_rot[2],
                right_rot[0], right_rot[1], right_rot[2]
            )

        new_left = copy.deepcopy(current_left)
        new_right = copy.deepcopy(current_right)

        new_left.position.x += left_rot[0]
        new_left.position.y += left_rot[1]
        new_left.position.z += left_rot[2]
        new_right.position.x += right_rot[0]
        new_right.position.y += right_rot[1]
        new_right.position.z += right_rot[2]

        orientation_waist = current_waist if rotate_with_waist else None
        left_has_orientation = self.apply_pose_orientation(
            new_left,
            left_orientation,
            waist_angle=orientation_waist,
        )
        right_has_orientation = self.apply_pose_orientation(
            new_right,
            right_orientation,
            waist_angle=orientation_waist,
        )
        if left_has_orientation or right_has_orientation:
            rospy.logdebug(
                "普通相对偏移应用目标姿态 left_has_orientation=%s, right_has_orientation=%s, rotate_with_waist=%s",
                left_has_orientation,
                right_has_orientation,
                rotate_with_waist
            )

        rospy.logdebug(
            "普通相对偏移后 left_pos=(%.3f, %.3f, %.3f), left_quat=(%.4f, %.4f, %.4f, %.4f), "
            "right_pos=(%.3f, %.3f, %.3f), right_quat=(%.4f, %.4f, %.4f, %.4f)",
            new_left.position.x, new_left.position.y, new_left.position.z,
            new_left.orientation.x, new_left.orientation.y,
            new_left.orientation.z, new_left.orientation.w,
            new_right.position.x, new_right.position.y, new_right.position.z,
            new_right.orientation.x, new_right.orientation.y,
            new_right.orientation.z, new_right.orientation.w
        )
        return new_left, new_right

    def resolve_arm_poses(self, action):
        """
        根据 base_key 或 pose_source 返回 (left_pose, right_pose)
        优先级：base_key/pose_source.base_key > pose_source.use_runtime > 静态 arm_poses
        """
        pose_source = action.get("pose_source", {}) or {}
        point_names = self.read_action_point_names(action, pose_source)
        motion_mode = self.read_action_motion_mode(action, pose_source)

        if motion_mode in (4, 5):
            raw_relative_offset = action.get("relative_offset", pose_source.get("relative_offset", {}))
            if raw_relative_offset is None:
                relative_offset = {}
            else:
                relative_offset = self.select_relative_offset(action, raw_relative_offset)
                if relative_offset is None:
                    return None, None
            cache_key = pose_source.get("key", action.get("cache_key"))
            if motion_mode == 5:
                return self.apply_relative_offset_from_hand_targets(
                    action,
                    relative_offset,
                    cache_key=cache_key,
                    pose_source=pose_source,
                )
            return self.apply_relative_offset_from_point_delta(
                action,
                relative_offset,
                cache_key=cache_key,
                pose_source=pose_source,
            )

        # ---------- 1. 处理 base_key 逻辑 ----------
        base_key = action.get("base_key", pose_source.get("base_key"))
        if base_key:
            raw_relative_offset = action.get("relative_offset", pose_source.get("relative_offset"))
            if raw_relative_offset is not None:
                relative_offset = self.select_relative_offset(action, raw_relative_offset)
                if relative_offset is None:
                    return None, None
                cache_key = pose_source.get("key")
                if motion_mode == 3:
                    return self.apply_relative_offset_from_single_point(
                        action,
                        base_key,
                        relative_offset,
                        cache_key,
                        point_names=point_names,
                    )
                return self.apply_relative_offset_from_current_midpoint(
                    action,
                    base_key,
                    relative_offset,
                    cache_key,
                    point_names=point_names if motion_mode == 2 else None,
                    motion_mode=motion_mode,
                )

            # 无 relative_offset 时，base_key 仍按缓存位姿执行
            left_pose, right_pose = self.get_cached_arm_poses(
                base_key,
                point_names=point_names,
                motion_mode=motion_mode,
            )
            if left_pose is not None and right_pose is not None:
                return left_pose, right_pose
            if not (pose_source.get("fallback_to_yaml", False) or action.get("fallback_to_yaml", False)):
                return None, None
            rospy.logdebug("缓存位姿缺失 key=%s，回退到 YAML 静态位姿", base_key)

        # ---------- 2. 无 base_key 的普通相对运动 ----------
        raw_relative_offset = action.get("relative_offset", pose_source.get("relative_offset"))
        if raw_relative_offset is not None:
            relative_offset = self.select_relative_offset(action, raw_relative_offset)
            if relative_offset is None:
                return None, None
            return self.apply_relative_offset_from_current_hands(action, relative_offset)

        # ---------- 3. pose_source 逻辑（base_key 和 relative_offset 都不存在时才走这里） ----------
        use_runtime = pose_source.get("use_runtime", False)

        if use_runtime:
            key = pose_source.get("key", action.get("id", ""))
            if self.trigger_and_cache_detection(pose_source, default_key=key):
                resp = self.services.get_target(
                    key,
                    timeout=1.0,
                    motion_mode=motion_mode,
                    point_names=point_names,
                )
                if resp is not None and resp.success:
                    rospy.logdebug("动态位姿获取成功 key=%s", key)
                    self._pose_cache[self.target_cache_key(key, point_names, motion_mode)] = (
                        resp.left_pose,
                        resp.right_pose,
                    )
                    if int(pose_source.get("trigger_value", 2)) == 1:
                        self._pose_cache[CENTER_CACHE_PREFIX + key] = (
                            resp.left_pose,
                            resp.right_pose,
                        )
                    return resp.left_pose, resp.right_pose
                if resp is not None:
                    rospy.logwarn("动态位姿服务返回失败: %s", resp.message)

            if not pose_source.get("fallback_to_yaml", True):
                rospy.logerr("动态获取失败且 fallback_to_yaml=False，动作中止")
                return None, None
            rospy.logdebug("回退到 YAML 静态位姿")

        # ---------- 4. 静态位姿 ----------
        arm_poses = action.get("arm_poses", {})
        left = arm_poses.get("left_arm", {}).get("pose")
        right = arm_poses.get("right_arm", {}).get("pose")
        if not left or not right:
            rospy.logerr("缺少静态 arm_poses 定义")
            return None, None
        return self.pose_from_dict(left), self.pose_from_dict(right)

    def execute_behavior(self, behavior, action_name=None, action_id=None):
        rospy.loginfo("====== 执行行为 {} {} ======".format(
            behavior.get("id", ""),
            behavior.get("label", behavior.get("name", "")),
        ))

        matched = False
        for action in behavior.get("actions", []):
            if action_id is not None and action.get("id") != action_id:
                continue
            if action_name is not None and action.get("name") != action_name:
                continue

            matched = True
            ok = self.execute_action(action)

            if not ok:
                rospy.logwarn("动作执行失败: {}".format(action.get("name", "")))
                if self.config.get("continue_on_action_fail", False):
                    rospy.logwarn("continue_on_action_fail=True，继续执行后续动作")
                    continue
                return False

        if (action_id is not None or action_name is not None) and not matched:
            rospy.logwarn("当前行为中没有找到指定动作 action_id={}, action_name={}".format(
                action_id,
                action_name,
            ))
            return False

        return True

    def execute_location(self,
                         location,
                         behavior_name=None,
                         behavior_id=None,
                         action_name=None,
                         action_id=None):
        rospy.loginfo("==== 执行地点 {} {} ====".format(
            location.get("id", ""),
            location.get("label", location.get("name", "")),
        ))

        nav_pose = location.get("nav_pose")
        twice_move = location.get("twice_move")
        if nav_pose:
            if self.config.get("skip_nav", True):
                rospy.loginfo("skip_nav=True，跳过导航 nav_pose")
            else:
                rospy.loginfo("准备执行导航: x={}, y={}, yaw={}".format(
                    nav_pose.get("x"),
                    nav_pose.get("y"),
                    nav_pose.get("yaw"),
                ))
                if not self.services.call_navigation(nav_pose):
                    rospy.logwarn("导航执行失败: {}".format(location.get("name", "")))
                    return False
        elif twice_move:
            rospy.loginfo("准备执行二次矫正: {}".format(location.get("name", "")))
            if not self.services.call_twice_move(twice_move):
                rospy.logwarn("二次矫正执行失败: {}".format(location.get("name", "")))
                return False
        elif location.get("behaviors"):
            rospy.loginfo("当前地点无导航/二次矫正，直接执行行为: {}".format(location.get("name", "")))
        else:
            rospy.logwarn("当前地点没有配置 nav_pose: {}".format(location.get("name", "")))

        matched = False
        for behavior in location.get("behaviors", []):
            if behavior_id is not None and behavior.get("id") != behavior_id:
                continue
            if behavior_name is not None and behavior.get("name") != behavior_name:
                continue

            matched = True
            ok = self.execute_behavior(
                behavior,
                action_name=action_name,
                action_id=action_id,
            )

            if not ok:
                rospy.logwarn("行为执行失败: {}".format(behavior.get("name", "")))
                if self.config.get("continue_on_behavior_fail", False):
                    rospy.logwarn("continue_on_behavior_fail=True，继续执行后续行为")
                    continue
                return False

        if (behavior_id is not None or behavior_name is not None) and not matched:
            rospy.logwarn("当前地点中没有找到指定行为 behavior_id={}, behavior_name={}".format(
                behavior_id,
                behavior_name,
            ))
            return False

        return True

    def execute_task(self,
                     task,
                     location_name=None,
                     location_id=None,
                     behavior_name=None,
                     behavior_id=None,
                     action_name=None,
                     action_id=None):
        rospy.loginfo("== 执行任务 {} {} ==".format(
            task.get("id", ""),
            task.get("label", task.get("name", "")),
        ))

        matched = False
        for location in task.get("locations", []):
            if location_id is not None and location.get("id") != location_id:
                continue
            if location_name is not None and location.get("name") != location_name:
                continue

            matched = True
            ok = self.execute_location(
                location,
                behavior_name=behavior_name,
                behavior_id=behavior_id,
                action_name=action_name,
                action_id=action_id,
            )

            if not ok:
                rospy.logwarn("地点执行失败: {}".format(location.get("name", "")))
                if self.config.get("continue_on_location_fail", False):
                    rospy.logwarn("continue_on_location_fail=True，继续执行后续地点")
                    continue
                return False

        if (location_id is not None or location_name is not None) and not matched:
            rospy.logwarn("当前任务中没有找到指定地点 location_id={}, location_name={}".format(
                location_id,
                location_name,
            ))
            return False

        return True

    @staticmethod
    def find_task(tree, task_name=None, task_id=None):
        for task in tree.get("tasks", []):
            if task_id is not None and task.get("id") == task_id:
                return task
            if task_name is not None and task.get("name") == task_name:
                return task
        return None

    def execute_tree(self, tree):
        rospy.loginfo("开始执行行为树: {}".format(tree.get("name", "")))

        for task in tree.get("tasks", []):
            ok = self.execute_task(task)
            if not ok:
                rospy.logwarn("任务执行失败: {}".format(task.get("name", "")))
                return False

        rospy.loginfo("行为树执行完成")
        return True

    def execute_by_select(self,
                          tree,
                          task_name=None,
                          task_id=None,
                          location_name=None,
                          location_id=None,
                          behavior_name=None,
                          behavior_id=None,
                          action_name=None,
                          action_id=None):
        if task_name is None and task_id is None:
            return self.execute_tree(tree)

        task = self.find_task(tree, task_name=task_name, task_id=task_id)
        if task is None:
            rospy.logerr("没有找到指定任务 task_name={}, task_id={}".format(
                task_name,
                task_id,
            ))
            return False

        return self.execute_task(
            task,
            location_name=location_name,
            location_id=location_id,
            behavior_name=behavior_name,
            behavior_id=behavior_id,
            action_name=action_name,
            action_id=action_id,
        )
