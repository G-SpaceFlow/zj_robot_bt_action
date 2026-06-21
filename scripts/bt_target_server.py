#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import rospy
import copy
import tf.transformations as tfm
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from robot_bt_action.srv import GetTarget, GetTargetResponse
from robot_bt_action.srv import SetOffset, SetOffsetResponse


CENTER_CACHE_PREFIX = "__center__:"
VISION_POINTS_TOPIC = "/yolo_vision/front_points_base_json"
WALL_ANGLE_TOPIC = "/yolo_vision/wall_angle"
VISION_CONTROL_TOPIC = "/yolo_vision/control"
TARGET_LABELS_TOPIC = "/yolo_vision/target_labels"
TRIGGER_DETECT_SERVICE = "/bt_target_server/trigger_detect"
GET_TARGET_SERVICE = "/bt_target_server/get_target"
SET_OFFSET_SERVICE = "/bt_target_server/set_offset"


class TargetServer:
    def __init__(self):
        self._init_state()
        self._init_ros_interfaces()
        rospy.loginfo("服务就绪: get_target, trigger_detect, set_offset")

    def _init_state(self):
        self.cache = {}                  # key → (left_pose, right_pose)
        self.detection_cache = {}        # key → 原始视觉 JSON
        self.point_cache = {}            # key → {point_name: raw_pose}
        self.latest_detection = None
        self.latest_front_detection = None
        self.latest_wall_angle = None
        self.offset_map = {
            "default": self._identity_offset(),
        }                                # 新增：存储offset配置

    def _init_ros_interfaces(self):
        rospy.Subscriber(VISION_POINTS_TOPIC, String, self.vision_cb)
        rospy.Subscriber(WALL_ANGLE_TOPIC, String, self.wall_angle_cb)
        self.control_pub = rospy.Publisher(VISION_CONTROL_TOPIC, Int32, queue_size=1)
        self.label_pub = rospy.Publisher(TARGET_LABELS_TOPIC, String, queue_size=1)

        rospy.Service(TRIGGER_DETECT_SERVICE, GetTarget, self.handle_trigger_detect)
        rospy.Service(GET_TARGET_SERVICE, GetTarget, self.handle_get_target)
        rospy.Service(SET_OFFSET_SERVICE, SetOffset, self.handle_set_offset)

    def get_current_waist_angle(self, timeout=0.5):
        """
        从 /zj_humanoid/upperlimb/joint_states 获取当前 Waist_Z 角度
        （只读取一次话题最新消息）
        """
        try:
            msg = rospy.wait_for_message(
                "/zj_humanoid/upperlimb/joint_states",
                JointState,
                timeout=timeout
            )
            idx = msg.name.index("Waist_Z")
            return msg.position[idx]
        except (rospy.ROSException, ValueError) as e:
            rospy.logwarn("获取 Waist_Z 角度失败: %s，使用 0.0 代替", str(e))
            return 0.0

    def apply_offset_transform(self, left_pose, right_pose, offset_id):
        """
        根据 offset_id 对左右臂位姿进行动态补偿。
        返回 (new_left_pose, new_right_pose)，失败返回 (None, None)
        """
        entry = self.offset_map.get(offset_id)
        if entry is None:
            rospy.logerr("apply_offset_transform: offset_id '%s' 未找到", offset_id)
            return None, None

        # 1. 获取当前腰部角度（只取一次）
        theta = self.get_current_waist_angle()
        q_waist = tfm.quaternion_about_axis(theta, (0, 0, 1))

        # 2. 取出 offset 姿态四元数
        q_left_off = [
            float(entry["left_ox"]), float(entry["left_oy"]),
            float(entry["left_oz"]), float(entry["left_ow"])
        ]
        q_right_off = [
            float(entry["right_ox"]), float(entry["right_oy"]),
            float(entry["right_oz"]), float(entry["right_ow"])
        ]

        # 3. 姿态补偿：offset姿态在腰部坐标系下标定，转换到base坐标系
        q_left_rot = tfm.unit_vector(tfm.quaternion_multiply(q_waist, q_left_off))
        q_right_rot = tfm.unit_vector(tfm.quaternion_multiply(q_waist, q_right_off))

        # 4. 创建新位姿
        new_left = copy.deepcopy(left_pose)
        new_right = copy.deepcopy(right_pose)

        # 4.1 原始补偿向量：补偿是在腰部局部坐标系下标定的
        left_offset = [
            float(entry.get("left_dx", 0.0)),
            float(entry.get("left_dy", 0.0)),
            float(entry.get("left_dz", 0.0)),
            0.0
        ]

        right_offset = [
            float(entry.get("right_dx", 0.0)),
            float(entry.get("right_dy", 0.0)),
            float(entry.get("right_dz", 0.0)),
            0.0
        ]

        # 4.2 只旋转补偿量，不旋转视觉点
        waist_mat = tfm.quaternion_matrix(q_waist)

        left_offset_rot = waist_mat.dot(left_offset)
        right_offset_rot = waist_mat.dot(right_offset)



        rospy.logdebug(
            "左补偿旋转前=(%.4f, %.4f, %.4f), 旋转后=(%.4f, %.4f, %.4f)",
            left_offset[0], left_offset[1], left_offset[2],
            left_offset_rot[0], left_offset_rot[1], left_offset_rot[2]
        )

        rospy.logdebug(
            "右补偿旋转前=(%.4f, %.4f, %.4f), 旋转后=(%.4f, %.4f, %.4f)",
            right_offset[0], right_offset[1], right_offset[2],
            right_offset_rot[0], right_offset_rot[1], right_offset_rot[2]
        )

        rospy.logdebug(
            "视觉原始点 left=(%.4f, %.4f, %.4f), right=(%.4f, %.4f, %.4f)",
            left_pose.position.x, left_pose.position.y, left_pose.position.z,
            right_pose.position.x, right_pose.position.y, right_pose.position.z
        )

        # 4.3 最终位置 = 视觉点 + 旋转后的补偿
        new_left.position.x = left_pose.position.x + left_offset_rot[0]
        new_left.position.y = left_pose.position.y + left_offset_rot[1]
        new_left.position.z = left_pose.position.z + left_offset_rot[2]

        new_right.position.x = right_pose.position.x + right_offset_rot[0]
        new_right.position.y = right_pose.position.y + right_offset_rot[1]
        new_right.position.z = right_pose.position.z + right_offset_rot[2]

        # 5. 姿态补偿：补偿姿态跟随腰部方向旋转
        new_left.orientation.x = q_left_rot[0]
        new_left.orientation.y = q_left_rot[1]
        new_left.orientation.z = q_left_rot[2]
        new_left.orientation.w = q_left_rot[3]

        new_right.orientation.x = q_right_rot[0]
        new_right.orientation.y = q_right_rot[1]
        new_right.orientation.z = q_right_rot[2]
        new_right.orientation.w = q_right_rot[3]

        rospy.logdebug(
            "最终补偿后 left=(%.4f, %.4f, %.4f), right=(%.4f, %.4f, %.4f), waist=%.3f",
            new_left.position.x, new_left.position.y, new_left.position.z,
            new_right.position.x, new_right.position.y, new_right.position.z,
            theta
        )
        rospy.logdebug(
            "最终姿态 left=(%.4f, %.4f, %.4f, %.4f), right=(%.4f, %.4f, %.4f, %.4f)",
            new_left.orientation.x, new_left.orientation.y,
            new_left.orientation.z, new_left.orientation.w,
            new_right.orientation.x, new_right.orientation.y,
            new_right.orientation.z, new_right.orientation.w
        )
        return new_left, new_right

    def vision_cb(self, msg):
        detection = self._parse_json_msg(msg, "视觉 JSON")
        if detection is not None:
            self.latest_front_detection = detection
            self.latest_detection = detection

    def wall_angle_cb(self, msg):
        detection = self._parse_json_msg(msg, "墙壁角度 JSON")
        if detection is not None:
            self.latest_wall_angle = detection

    @staticmethod
    def _parse_json_msg(msg, name):
        try:
            data = json.loads(msg.data)
            if isinstance(data, str):
                data = json.loads(data)
            if not isinstance(data, dict):
                rospy.logwarn("%s 不是对象: %s", name, type(data).__name__)
                return None
            return data
        except Exception as e:
            rospy.logwarn("解析%s出错: %s", name, e)
            return None

    def _reset_latest_for_trigger(self, trigger_val):
        if int(trigger_val) == 3:
            self.latest_wall_angle = None
        else:
            self.latest_front_detection = None
            self.latest_detection = None

    def _latest_for_trigger(self, trigger_val):
        if int(trigger_val) == 3:
            return self.latest_wall_angle
        return self.latest_front_detection or self.latest_detection

    def handle_get_target(self, req):
        resp = GetTargetResponse()
        point_names = self._normalize_point_names(req.point_names if hasattr(req, "point_names") else [])
        motion_mode = self._normalize_motion_mode(req.motion_mode if hasattr(req, "motion_mode") else 1)

        if point_names:
            return self._response_from_cached_points(req.key, point_names, motion_mode)

        if req.key in self.cache:
            resp.success = True
            resp.message = "ok"
            resp.left_pose, resp.right_pose = self.cache[req.key]
        else:
            resp.success = False
            resp.message = "No pose for key '{}'".format(req.key)
        return resp

    def handle_trigger_detect(self, req):
        resp = GetTargetResponse()
        key = req.key if req.key else "1-1-1-2"

        # 从请求中读取 trigger_value，若未设置则默认 2
        trigger_val = req.trigger_value if hasattr(req, 'trigger_value') else 2
        labels = self._normalize_labels(req.labels if hasattr(req, "labels") else [])
        rospy.loginfo("触发识别 key=%s, trigger_value=%d, labels=%s", key, trigger_val, labels)

        self._reset_latest_for_trigger(trigger_val)
        self.label_pub.publish(String(data=json.dumps(labels, ensure_ascii=False)))
        self.control_pub.publish(Int32(data=trigger_val))

        detection = None
        start_time = rospy.Time.now()
        timeout = rospy.Duration(5.0)
        rate = rospy.Rate(20)

        try:
            while not rospy.is_shutdown() and rospy.Time.now() - start_time < timeout:
                detection = self._latest_for_trigger(trigger_val)
                if detection is not None:
                    rospy.logdebug("已收到视觉数据 key=%s，1秒后关闭视觉触发", key)
                    rospy.sleep(1.0)
                    break
                rate.sleep()
        finally:
            self.control_pub.publish(Int32(data=0))

        if detection is None:
            resp.success = False
            resp.message = "5秒内未收到视觉数据"
            return resp

        if labels and not self._detection_matches_labels(detection, labels):
            resp.success = False
            resp.message = "视觉数据未匹配指定标签: {}".format(",".join(labels))
            return resp

        self.latest_detection = detection
        self._cache_detection(key, detection)

        point_names = self._normalize_point_names(req.point_names if hasattr(req, "point_names") else [])
        motion_mode = self._normalize_motion_mode(req.motion_mode if hasattr(req, "motion_mode") else 1)
        if point_names:
            if motion_mode == 1 and len(point_names) == 1:
                motion_mode = 4
                rospy.logdebug(
                    "触发识别请求为单点 points=%s 且 motion_mode=1，自动按单点模式缓存",
                    point_names,
                )
            return self._response_from_cached_points(key, point_names, motion_mode)

        points = self.latest_detection.get("points", {})
        orig_orient = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        center_raw = points.get("center")
        if center_raw is not None:
            center_pt = {
                "x": center_raw["x"],
                "y": center_raw["y"],
                "z": center_raw["z"]
            }
            center_pose = self._make_pose(center_pt, orig_orient)
            self.cache[CENTER_CACHE_PREFIX + key] = (center_pose, center_pose)
            rospy.logdebug(
                "缓存视觉中心点 center_key=%s raw=(%.4f, %.4f, %.4f)",
                CENTER_CACHE_PREFIX + key,
                center_pose.position.x,
                center_pose.position.y,
                center_pose.position.z
            )

        if trigger_val == 1:
            if center_raw is None:
                resp.success = False
                resp.message = "视觉数据缺少 center 点"
                return resp

            self.cache[key] = (center_pose, center_pose)
            resp.success = True
            resp.message = "ok"
            resp.left_pose = center_pose
            resp.right_pose = center_pose
            rospy.logdebug(
                "trigger_value=1 返回视觉中心点 key=%s，不应用补偿或旋转: center=(%.4f, %.4f, %.4f)",
                key,
                center_pose.position.x,
                center_pose.position.y,
                center_pose.position.z
            )
            return resp

        left_raw = points.get("left")
        right_raw = points.get("right")
        if left_raw is None or right_raw is None:
            resp.success = False
            resp.message = "视觉数据缺少 left/right 点"
            return resp

        rospy.logdebug("视觉目标源点: left/right")

        # 原始坐标（不应用简单offset，因为apply_offset_transform会处理）
        left_pt = {
            "x": left_raw["x"],
            "y": left_raw["y"],
            "z": left_raw["z"]
        }
        right_pt = {
            "x": right_raw["x"],
            "y": right_raw["y"],
            "z": right_raw["z"]
        }

        left_pose_raw = self._make_pose(left_pt, orig_orient)
        right_pose_raw = self._make_pose(right_pt, orig_orient)

        # 调用apply_offset_transform进行位姿转换（默认使用"default"作为offset_id）
        offset_id = "default"
        new_left_pose, new_right_pose = self.apply_offset_transform(left_pose_raw, right_pose_raw, offset_id)

        if new_left_pose is None or new_right_pose is None:
            resp.success = False
            resp.message = "位姿转换失败"
            return resp

        self.cache[key] = (new_left_pose, new_right_pose)

        resp.success = True
        resp.message = "ok"
        resp.left_pose = new_left_pose
        resp.right_pose = new_right_pose
        return resp

    def _cache_detection(self, key, detection):
        self.detection_cache[key] = copy.deepcopy(detection)

        raw_points = detection.get("points", {}) if isinstance(detection, dict) else {}
        if not isinstance(raw_points, dict):
            rospy.logwarn("视觉 points 字段不是对象 key=%s, type=%s", key, type(raw_points).__name__)
            raw_points = {}
        point_map = {}
        orig_orient = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        if isinstance(detection, dict):
            scalar_names = ["yaw_rad", "yaw_deg"]
            for name in scalar_names:
                if name not in detection:
                    continue
                try:
                    point_map[name] = self._make_pose(
                        {"x": float(detection[name]), "y": 0.0, "z": 0.0},
                        orig_orient,
                    )
                except (TypeError, ValueError):
                    rospy.logwarn("视觉标量无法转换为浮点 key=%s, field=%s, value=%s", key, name, detection[name])

            if all(axis in detection for axis in ("base_x", "base_y", "base_z")):
                try:
                    point_map["base"] = self._make_pose(
                        {
                            "x": float(detection["base_x"]),
                            "y": float(detection["base_y"]),
                            "z": float(detection["base_z"]),
                        },
                        orig_orient,
                    )
                except (TypeError, ValueError):
                    rospy.logwarn("视觉 base 坐标无法转换为浮点 key=%s, data=%s", key, detection)

        for name, point in raw_points.items():
            if not isinstance(point, dict):
                rospy.logwarn("忽略无效视觉点 key=%s, point=%s", key, name)
                continue
            if not all(axis in point for axis in ("x", "y", "z")):
                rospy.logwarn("视觉点缺少坐标 key=%s, point=%s, data=%s", key, name, point)
                continue
            point_map[str(name)] = self._make_pose(
                {
                    "x": float(point["x"]),
                    "y": float(point["y"]),
                    "z": float(point["z"]),
                },
                orig_orient,
            )

        self.point_cache[key] = point_map
        rospy.logdebug(
            "已缓存视觉原始数据 key=%s, points=%s",
            key,
            sorted(point_map.keys()),
        )

        for name, pose in point_map.items():
            rospy.logdebug(
                "视觉点缓存 key=%s, point=%s, raw=(%.4f, %.4f, %.4f)",
                key,
                name,
                pose.position.x,
                pose.position.y,
                pose.position.z,
            )

    def _response_from_cached_points(self, key, point_names, motion_mode):
        resp = GetTargetResponse()
        selected = self._select_cached_points(key, point_names, motion_mode)
        if selected is None:
            resp.success = False
            resp.message = "No selected pose for key '{}', points={}, motion_mode={}".format(
                key,
                point_names,
                motion_mode,
            )
            return resp

        left_pose, right_pose, cache_key = selected
        self.cache[cache_key] = (left_pose, right_pose)
        resp.success = True
        resp.message = "ok"
        resp.left_pose = left_pose
        resp.right_pose = right_pose
        return resp

    def _select_cached_points(self, key, point_names, motion_mode):
        point_map = self.point_cache.get(key)
        if not point_map:
            rospy.logerr("没有视觉点缓存 key=%s", key)
            return None

        offset_id = "default"
        if motion_mode in (2, 3, 4, 5):
            names = point_names or ["center"]
            if len(names) != 1:
                rospy.logerr("运动模式%d需要一个点名，当前=%s", motion_mode, names)
                return None
            raw_pose = point_map.get(names[0])
            if raw_pose is None:
                rospy.logerr("视觉缓存 key=%s 缺少点 %s，可用点=%s", key, names[0], sorted(point_map.keys()))
                return None

            left_pose = copy.deepcopy(raw_pose)
            right_pose = copy.deepcopy(raw_pose)
            cache_key = self._target_cache_key(key, names, motion_mode)
            rospy.logdebug(
                "模式%d选点 key=%s, point=%s, cache_key=%s, 使用视觉原始单点，不应用 left/right offset",
                motion_mode,
                key,
                names[0],
                cache_key,
            )
            return left_pose, right_pose, cache_key

        names = point_names or ["left", "right"]
        if len(names) < 2:
            rospy.logerr("运动模式1至少需要两个点名，当前=%s", names)
            return None
        if len(names) > 2:
            rospy.logdebug("运动模式1使用前两个点作为左右手目标，额外点只保留在视觉缓存: %s", names[2:])
            names = names[:2]

        left_raw = point_map.get(names[0])
        right_raw = point_map.get(names[1])
        if left_raw is None or right_raw is None:
            rospy.logerr("视觉缓存 key=%s 缺少点 %s，可用点=%s", key, names, sorted(point_map.keys()))
            return None

        left_pose, right_pose = self.apply_offset_transform(left_raw, right_raw, offset_id)
        if left_pose is None or right_pose is None:
            return None
        cache_key = self._target_cache_key(key, names, motion_mode)
        rospy.logdebug("模式1选点 key=%s, left_point=%s, right_point=%s, cache_key=%s",
                       key, names[0], names[1], cache_key)
        return left_pose, right_pose, cache_key

    @staticmethod
    def _target_cache_key(key, point_names, motion_mode):
        return "{}|mode={}|points={}".format(
            key,
            int(motion_mode),
            ",".join(point_names),
        )

    def handle_set_offset(self, req):
        resp = SetOffsetResponse()
        # 默认使用"default"作为offset_id，也可根据需要扩展为从req获取offset_id
        offset_id = "default"
        self.offset_map[offset_id] = {
            "left_dx": req.left_dx,
            "left_dy": req.left_dy,
            "left_dz": req.left_dz,
            "right_dx": req.right_dx,
            "right_dy": req.right_dy,
            "right_dz": req.right_dz,
            "left_ox": req.left_ox,
            "left_oy": req.left_oy,
            "left_oz": req.left_oz,
            "left_ow": req.left_ow,
            "right_ox": req.right_ox,
            "right_oy": req.right_oy,
            "right_oz": req.right_oz,
            "right_ow": req.right_ow,
        }
        self.cache.clear()
        rospy.loginfo("补偿更新: offset_id=%s", offset_id)
        rospy.logdebug("补偿更新数据: %s", self.offset_map[offset_id])
        resp.success = True
        resp.message = "offset updated"
        return resp

    @staticmethod
    def _make_pose(point, orient):
        pose = Pose()
        pose.position.x = point["x"]
        pose.position.y = point["y"]
        pose.position.z = point["z"]
        pose.orientation.x = orient["x"]
        pose.orientation.y = orient["y"]
        pose.orientation.z = orient["z"]
        pose.orientation.w = orient["w"]
        return pose

    @staticmethod
    def _normalize_labels(labels):
        if labels is None:
            return []
        if isinstance(labels, str):
            labels = [labels]
        return [str(label).strip() for label in labels if str(label).strip()]

    @staticmethod
    def _normalize_point_names(point_names):
        if point_names is None:
            return []
        if isinstance(point_names, str):
            point_names = [point_names]
        return [str(name).strip() for name in point_names if str(name).strip()]

    @staticmethod
    def _normalize_motion_mode(motion_mode):
        try:
            mode = int(motion_mode)
        except (TypeError, ValueError):
            mode = 1
        if mode == 0:
            return 1
        return mode

    @classmethod
    def _detection_matches_labels(cls, detection, expected_labels):
        found_labels = cls._collect_detection_labels(detection)
        if not found_labels:
            rospy.logwarn("视觉数据未包含标签字段，已跳过标签结果校验")
            return True

        found_set = set(label.lower() for label in found_labels)
        return all(label.lower() in found_set for label in expected_labels)

    @classmethod
    def _collect_detection_labels(cls, value):
        label_keys = set(["label", "labels", "class", "class_name", "name", "tag", "tags", "category"])
        labels = []

        if isinstance(value, dict):
            for key, item in value.items():
                if key in label_keys:
                    labels.extend(cls._label_values(item))
                labels.extend(cls._collect_detection_labels(item))
        elif isinstance(value, list):
            for item in value:
                labels.extend(cls._collect_detection_labels(item))

        return labels

    @classmethod
    def _label_values(cls, value):
        if isinstance(value, (str, int, float)):
            return [str(value)]
        if isinstance(value, list):
            labels = []
            for item in value:
                labels.extend(cls._label_values(item))
            return labels
        if isinstance(value, dict):
            labels = []
            for item in value.values():
                labels.extend(cls._label_values(item))
            return labels
        return []

    @staticmethod
    def _identity_offset():
        return {
            "left_dx": 0.0,
            "left_dy": 0.0,
            "left_dz": 0.0,
            "right_dx": 0.0,
            "right_dy": 0.0,
            "right_dz": 0.0,
            "left_ox": 0.0,
            "left_oy": 0.0,
            "left_oz": 0.0,
            "left_ow": 1.0,
            "right_ox": 0.0,
            "right_oy": 0.0,
            "right_oz": 0.0,
            "right_ow": 1.0,
        }


if __name__ == "__main__":
    rospy.init_node("bt_target_server")
    TargetServer()
    rospy.spin()
