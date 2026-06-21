#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import signal
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rospy
from bt_executor import BehaviorTreeExecutor
from bt_runner_config import add_common_runner_args
from bt_runner_config import build_config_from_args


DEFAULT_YAML_PATH = os.path.abspath(os.path.join(SCRIPT_DIR, os.pardir, "行为树_底涂上下料.yaml"))


def parse_args():
    parser = argparse.ArgumentParser(description="自定义行为树流程控制程序")
    add_common_runner_args(parser)
    return parser.parse_args()


def execute_step(executor, tree, step):
    rospy.loginfo("执行 step={}".format(step))
    ok = executor.execute_by_select(
        tree,
        task_id=step.get("task_id"),
        location_id=step.get("location_id"),
        behavior_id=step.get("behavior_id"),
        action_id=step.get("action_id"),
    )

    if not ok:
        rospy.logwarn("执行失败: {}".format(step))
    return ok


def main():
    rospy.init_node("custom_bt_runner")

    args = parse_args()
    config = build_config_from_args(args)
    rospy.loginfo("使用行为树文件: %s", DEFAULT_YAML_PATH)
    rospy.loginfo("执行配置: skip_nav=%s", config.get("skip_nav"))
    executor = BehaviorTreeExecutor(config=config)
    rospy.on_shutdown(executor.services.cancel_navigation)

    def handle_sigint(signum, frame):
        rospy.logwarn("收到 Ctrl+C，正在取消导航/二次矫正并退出")
        executor.services.cancel_navigation()
        raise SystemExit(130)

    signal.signal(signal.SIGINT, handle_sigint)
    tree = executor.load_tree(DEFAULT_YAML_PATH)

    workflow = [

        {
            "name": "拿料盘",
            "repeat": 1,
            "steps": [
                {
                    "task_id": "task_001",
                    # "behavior_id": "behavior_001",
                    # "action_id": "move_001"
                },
            ],
        },
        # {
        #     "name": "放料盘",
        #     "repeat": 1,
        #     "steps": [
        #         {
        #             "task_id": "task_002",
        #         },
        #     ],
        # },


    ]

    for group in workflow:
        repeat = int(group.get("repeat", 1))
        steps = group.get("steps", [])
        group_name = group.get("name", "")

        for cycle_index in range(repeat):
            rospy.loginfo("开始执行组=%s，第 %d/%d 次", group_name, cycle_index + 1, repeat)

            for step in steps:
                rospy.sleep(1.0)
                if not execute_step(executor, tree, step):
                    rospy.logwarn("执行失败，停止后续动作")
                    return

    rospy.loginfo("自定义动作组合执行完成")


if __name__ == "__main__":
    main()
