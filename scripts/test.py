#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math


SERVICE_NAME = "/zj_humanoid/upperlimb/movej/whole_body"

# 左臂7维关节角。这里会扫描第6个关节，也就是下标5。
BASE_LEFT_JOINTS = [
    0.0000, 0.4363, 0.5236, -0.0000, -0.0000, -1.0000, 0.0000
]
SCAN_JOINT_INDEX = 5


def build_scan_values(start, end, step_deg):
    step = math.radians(abs(step_deg))
    if step <= 0.0:
        raise ValueError("step_deg must be greater than 0")

    direction = 1.0 if end >= start else -1.0
    values = []
    current = float(start)
    epsilon = step * 0.001

    while (direction > 0 and current <= end + epsilon) or (
            direction < 0 and current >= end - epsilon):
        values.append(current)
        current += direction * step

    if not values or abs(values[-1] - end) > 1e-9:
        values.append(float(end))

    return values


def call_movej(client, request_cls, joints, args):
    req = request_cls()
    req.joints = joints
    req.v = args.v
    req.acc = args.acc
    req.t = args.t
    req.is_async = False
    req.arm_type = args.arm_type
    return client(req)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Scan left arm joint 6 from -1.0 rad to 1.0 rad by 1 degree."
    )
    parser.add_argument("--start", type=float, default=0.0,
                        help="start angle in rad, default: 1.0")
    parser.add_argument("--end", type=float, default=1.2,
                        help="end angle in rad, default: 1.2")
    parser.add_argument("--step-deg", type=float, default=1.0,
                        help="scan step in degree, default: 1.0")
    parser.add_argument("--v", type=float, default=0.2,
                        help="MoveJ velocity, default: 0.2")
    parser.add_argument("--acc", type=float, default=0.2,
                        help="MoveJ acceleration, default: 0.2")
    parser.add_argument("--t", type=float, default=0.0,
                        help="MoveJ total time, default: 0.0")
    parser.add_argument("--arm-type", type=int, default=1,
                        help="MoveJ arm_type, default: 1 (left arm)")
    parser.add_argument("--pause", type=float, default=0.2,
                        help="pause seconds after each successful point, default: 0.2")
    parser.add_argument("--yes", action="store_true",
                        help="start without interactive confirmation")
    parser.add_argument("--continue-on-fail", action="store_true",
                        help="continue scanning if one point fails")
    return parser.parse_args()


def main():
    args = parse_args()
    scan_values = build_scan_values(args.start, args.end, args.step_deg)

    print("准备测试左臂第6关节:")
    print("  服务: {}".format(SERVICE_NAME))
    print("  范围: {:.4f} rad -> {:.4f} rad".format(args.start, args.end))
    print("  步长: {:.4f} deg ({:.6f} rad)".format(
        args.step_deg, math.radians(abs(args.step_deg))))
    print("  点数: {}".format(len(scan_values)))
    print("  基础关节: {}".format(["{:.4f}".format(v) for v in BASE_LEFT_JOINTS]))

    if not args.yes:
        answer = input("确认开始执行真实机器人运动？输入 y 继续: ").strip().lower()
        if answer != "y":
            print("已取消")
            return

    import rospy
    from upperlimb.srv import MoveJ, MoveJRequest

    rospy.init_node("left_arm_joint6_scan_test")
    rospy.loginfo("等待 MoveJ 服务: %s", SERVICE_NAME)
    rospy.wait_for_service(SERVICE_NAME)
    client = rospy.ServiceProxy(SERVICE_NAME, MoveJ)

    for index, angle in enumerate(scan_values, 1):
        joints = list(BASE_LEFT_JOINTS)
        joints[SCAN_JOINT_INDEX] = angle

        rospy.loginfo(
            "测试 %d/%d: 左臂第6关节=%.4f rad (%.2f deg), joints=%s",
            index,
            len(scan_values),
            angle,
            math.degrees(angle),
            ["{:.4f}".format(v) for v in joints],
        )

        try:
            resp = call_movej(client, MoveJRequest, joints, args)
        except rospy.ServiceException as exc:
            rospy.logerr("MoveJ 服务调用异常: %s", str(exc))
            if args.continue_on_fail:
                continue
            break

        if not resp.success:
            rospy.logerr("MoveJ 执行失败: %s", resp.message)
            if args.continue_on_fail:
                continue
            break

        rospy.loginfo("MoveJ 执行成功: %s", resp.message)
        if args.pause > 0.0:
            rospy.sleep(args.pause)

    rospy.loginfo("左臂第6关节扫描测试结束")


if __name__ == "__main__":
    main()
