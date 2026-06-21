#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os
import sys
import threading

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import actionlib
import rospy

from actionlib_msgs.msg import GoalID, GoalStatus
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32, String

from navigation.msg import NavigationAction, NavigationGoal, Waypoint
from robot_bt_action.srv import NavigateToPose, NavigateToPoseResponse


ACTION_NAME = "/zj_humanoid/navigation/navigation"
CANCEL_TOPIC = ACTION_NAME + "/cancel"
SERVICE_NAME = "/bt_navigation_server/navigate_to_pose"
DEFAULT_DISTANCE_TOLERANCE = 0.04
DEFAULT_HEADING_TOLERANCE = 0.04
DEFAULT_ABORT_ACCEPT_DISTANCE = 0.10
DEFAULT_ABORT_ACCEPT_HEADING = 0.10
DEFAULT_TIMEOUT = 180.0
DEFAULT_VISION_CONTROL_TOPIC = "/yolo_vision/control"
STATE_TOPIC = "/bt_navigation_server/state"
BT_CANCEL_TOPIC = "/bt_navigation_server/cancel"
STATE_IDLE = "idle"
STATE_NAVIGATION = "navigation"
STATE_TWICE_MOVE = "twice_move"
GOAL_STATUS_NAMES = {
    GoalStatus.PENDING: "PENDING",
    GoalStatus.ACTIVE: "ACTIVE",
    GoalStatus.PREEMPTED: "PREEMPTED",
    GoalStatus.SUCCEEDED: "SUCCEEDED",
    GoalStatus.ABORTED: "ABORTED",
    GoalStatus.REJECTED: "REJECTED",
    GoalStatus.PREEMPTING: "PREEMPTING",
    GoalStatus.RECALLING: "RECALLING",
    GoalStatus.RECALLED: "RECALLED",
    GoalStatus.LOST: "LOST",
}

try:
    from scripts.twice_move_corrector import TwiceMoveCorrector
except ImportError:
    try:
        from twice_move_corrector import TwiceMoveCorrector
    except ImportError:
        TwiceMoveCorrector = None


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def make_pose(x, y, yaw):
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = 0.0
    qx, qy, qz, qw = yaw_to_quat(float(yaw))
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def field_exists(msg, field_name):
    return hasattr(msg, "__slots__") and field_name in msg.__slots__


def safe_set_simple_field(msg, field_name, value):
    if field_exists(msg, field_name):
        setattr(msg, field_name, value)
        return True
    return False


def safe_set_value_field(msg, field_name, value):
    if not field_exists(msg, field_name):
        return False

    sub_msg = getattr(msg, field_name)
    if hasattr(sub_msg, "value"):
        sub_msg.value = value
        return True

    try:
        setattr(msg, field_name, value)
        return True
    except Exception:
        return False


def finite_or_default(value, default):
    value = float(value)
    if math.isnan(value):
        return default
    return value


def result_float(result, field_name, default=float("inf")):
    if result is None or not hasattr(result, field_name):
        return default
    try:
        return abs(float(getattr(result, field_name)))
    except (TypeError, ValueError):
        return default


class NavigationService(object):
    def __init__(self):
        self.action_name = rospy.get_param("~action_name", ACTION_NAME)
        self.cancel_topic = rospy.get_param(
            "~cancel_topic",
            self.action_name + "/cancel",
        )
        self.client = actionlib.SimpleActionClient(
            self.action_name,
            NavigationAction,
        )
        self.cancel_pub = rospy.Publisher(self.cancel_topic, GoalID, queue_size=1)
        self.state_pub = rospy.Publisher(STATE_TOPIC, String, queue_size=1, latch=True)
        self.vision_pubs = {}
        self.lock = threading.Lock()
        self.navigation_goal_active = False
        self.cancel_requested = False
        self.current_controller = None
        self.current_vision_topic = DEFAULT_VISION_CONTROL_TOPIC
        self.current_vision_stop_value = 0
        self.publish_state(STATE_IDLE)
        rospy.on_shutdown(self.cancel_active_goal)
        rospy.Subscriber(BT_CANCEL_TOPIC, String, self.cancel_callback, queue_size=1)
        rospy.Service(SERVICE_NAME, NavigateToPose, self.handle_navigate)
        rospy.loginfo("Navigation service ready: %s", SERVICE_NAME)

    def publish_state(self, state):
        self.state_pub.publish(String(data=state))
        rospy.logdebug("Navigation service state: %s", state)

    def cancel_callback(self, msg):
        reason = msg.data if msg is not None else ""
        rospy.logwarn("Received BT navigation cancel request: %s", reason)
        self.cancel_requested = True
        self.cancel_active_goal()
        controller = self.current_controller
        if controller is not None:
            controller.request_stop()
        self.publish_vision_control(
            self.current_vision_topic,
            int(self.current_vision_stop_value),
        )

    def cancel_active_goal(self):
        if self.navigation_goal_active:
            self.navigation_goal_active = False
            cancel_msg = GoalID()
            for _ in range(10):
                self.cancel_pub.publish(cancel_msg)
                rospy.sleep(0.03)
            rospy.loginfo("Canceled active navigation goal on %s", self.cancel_topic)

    def get_vision_pub(self, topic):
        if topic not in self.vision_pubs:
            self.vision_pubs[topic] = rospy.Publisher(topic, Int32, queue_size=1)
            rospy.sleep(0.1)
        return self.vision_pubs[topic]

    def publish_vision_control(self, topic, value):
        pub = self.get_vision_pub(topic)
        msg = Int32(data=int(value))
        for _ in range(3):
            pub.publish(msg)
            rospy.sleep(0.05)
        rospy.loginfo("Published vision control: topic=%s value=%d", topic, value)

    def make_waypoint(self, req):
        waypoint = Waypoint()
        if not field_exists(waypoint, "pose"):
            raise RuntimeError("navigation/Waypoint has no pose field")

        waypoint.pose = make_pose(req.x, req.y, req.yaw)
        safe_set_simple_field(waypoint, "id", int(req.waypoint_id))
        safe_set_simple_field(waypoint, "action", int(req.action))
        safe_set_simple_field(waypoint, "audio", int(req.audio))
        safe_set_value_field(
            waypoint,
            "distance_tolerance",
            finite_or_default(req.distance_tolerance, DEFAULT_DISTANCE_TOLERANCE),
        )
        safe_set_value_field(
            waypoint,
            "heading_tolerance",
            finite_or_default(req.heading_tolerance, DEFAULT_HEADING_TOLERANCE),
        )
        safe_set_value_field(waypoint, "task_type", 0)
        return waypoint

    def make_goal(self, req):
        goal = NavigationGoal()
        if field_exists(goal, "header"):
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = req.frame_id or "map"

        safe_set_value_field(goal, "task_type", 0)
        safe_set_value_field(
            goal,
            "distance_tolerance",
            finite_or_default(req.distance_tolerance, DEFAULT_DISTANCE_TOLERANCE),
        )

        if field_exists(goal, "translation"):
            if hasattr(goal.translation, "enable"):
                goal.translation.enable = False
            if hasattr(goal.translation, "heading"):
                goal.translation.heading = 0.0

        if not field_exists(goal, "waypoints"):
            raise RuntimeError("NavigationGoal has no waypoints field")

        goal.waypoints.append(self.make_waypoint(req))
        return goal

    def execute_navigation(self, req):
        rospy.loginfo(
            "Waiting navigation action: %s, waypoint_id=%d, target=(%.5f, %.5f, %.5f)",
            self.action_name,
            req.waypoint_id,
            req.x,
            req.y,
            req.yaw,
        )
        if not self.client.wait_for_server(rospy.Duration(10.0)):
            return False, "navigation action server timeout"

        goal = self.make_goal(req)
        self.navigation_goal_active = True
        self.publish_state(STATE_NAVIGATION)
        self.client.send_goal(goal)

        try:
            timeout = finite_or_default(req.timeout, DEFAULT_TIMEOUT)
            if timeout > 0.0:
                finished = self.client.wait_for_result(rospy.Duration(timeout))
            else:
                finished = self.client.wait_for_result()
        finally:
            self.navigation_goal_active = False

        if not finished:
            self.cancel_pub.publish(GoalID())
            return False, "navigation timed out after {:.1f}s".format(timeout)

        state = self.client.get_state()
        state_name = GOAL_STATUS_NAMES.get(state, "UNKNOWN")
        result = self.client.get_result()
        if state != GoalStatus.SUCCEEDED:
            abort_accept_distance = float(
                rospy.get_param(
                    "~navigation_abort_accept_distance",
                    DEFAULT_ABORT_ACCEPT_DISTANCE,
                )
            )
            abort_accept_heading = float(
                rospy.get_param(
                    "~navigation_abort_accept_heading",
                    DEFAULT_ABORT_ACCEPT_HEADING,
                )
            )
            distance_deviation = result_float(result, "distance_deviation")
            heading_deviation = result_float(result, "heading_deviation")
            if (
                state == GoalStatus.ABORTED
                and distance_deviation <= abort_accept_distance
                and heading_deviation <= abort_accept_heading
            ):
                rospy.logwarn(
                    (
                        "Navigation aborted but close enough, continue. "
                        "distance_deviation=%.4f/%.4f, heading_deviation=%.4f/%.4f"
                    ),
                    distance_deviation,
                    abort_accept_distance,
                    heading_deviation,
                    abort_accept_heading,
                )
                return True, "navigation close enough"

            return False, "navigation failed, action state={}({}), result={}".format(
                state,
                state_name,
                result,
            )

        rospy.loginfo("Navigation completed")
        return True, "navigation completed"

    def build_twice_move_params(self, req):
        params = {}
        if req.twice_move_params_json:
            params.update(json.loads(req.twice_move_params_json))

        values = {
            "target_x": req.target_x,
            "target_y": req.target_y,
            "target_yaw": req.target_yaw,
            "vision_target_x": req.vision_target_x,
            "vision_target_y": req.vision_target_y,
            "vision_target_yaw": req.vision_target_yaw,
        }
        for name, value in values.items():
            if not math.isnan(float(value)):
                params[name] = float(value)
        return params

    def run_twice_move(self, req):
        if TwiceMoveCorrector is None:
            return False, "failed to import twice_move_corrector.TwiceMoveCorrector"

        self.cancel_requested = False
        self.publish_state(STATE_TWICE_MOVE)
        params = self.build_twice_move_params(req)
        for name, value in params.items():
            rospy.set_param("~" + name, value)

        topic = req.vision_control_topic or DEFAULT_VISION_CONTROL_TOPIC
        stop_value = int(req.vision_stop_value)
        controller = None
        self.current_vision_topic = topic
        self.current_vision_stop_value = stop_value
        try:
            self.publish_vision_control(topic, int(req.vision_trigger_value))
            if self.cancel_requested:
                return False, "twice_move canceled"
            controller = TwiceMoveCorrector()
            self.current_controller = controller
            controller.run()
            if self.cancel_requested:
                return False, "twice_move canceled"
            return True, "twice move completed"
        except rospy.ROSInterruptException as exc:
            if self.cancel_requested:
                rospy.logwarn("twice_move canceled: %s", exc)
                return False, "twice_move canceled"
            raise
        except Exception as exc:
            rospy.logerr("twice_move failed: %s", exc)
            return False, "twice_move failed: {}".format(exc)
        finally:
            try:
                if controller is not None:
                    controller.release_control()
            finally:
                self.current_controller = None
                self.publish_vision_control(topic, stop_value)

    def handle_navigate(self, req):
        with self.lock:
            resp = NavigateToPoseResponse()
            try:
                if not req.skip_navigation:
                    ok, message = self.execute_navigation(req)
                    if not ok:
                        resp.success = False
                        resp.message = message
                        return resp
                else:
                    rospy.loginfo("Skip map navigation, run twice_move directly")

                if req.enable_twice_move:
                    ok, message = self.run_twice_move(req)
                    if not ok:
                        resp.success = False
                        resp.message = message
                        return resp
                elif req.skip_navigation:
                    resp.success = False
                    resp.message = "skip_navigation=True requires enable_twice_move=True"
                    return resp

                resp.success = True
                resp.message = "ok"
                return resp
            finally:
                self.publish_state(STATE_IDLE)


def main():
    rospy.init_node("bt_navigation_server")
    NavigationService()
    rospy.spin()


if __name__ == "__main__":
    main()
