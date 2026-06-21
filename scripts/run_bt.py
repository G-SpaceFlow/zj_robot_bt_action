#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rospy
try:
    from scripts.bt_executor import BehaviorTreeExecutor
    from scripts.bt_runner_config import add_common_runner_args
    from scripts.bt_runner_config import build_config_from_args
except ImportError:
    from bt_executor import BehaviorTreeExecutor
    from bt_runner_config import add_common_runner_args
    from bt_runner_config import build_config_from_args


DEFAULT_YAML_PATH = os.path.abspath(os.path.join(SCRIPT_DIR, os.pardir, "行为树_底涂上下料.yaml"))


def parse_args():
    parser = argparse.ArgumentParser(description="行为树 YAML 控制程序")

    parser.add_argument(
        "--yaml",
        default=DEFAULT_YAML_PATH,
        help="行为树 YAML 文件路径",
    )

    parser.add_argument("--task-id", default=None)
    parser.add_argument("--task-name", default=None)

    parser.add_argument("--location-id", default=None)
    parser.add_argument("--location-name", default=None)

    parser.add_argument("--behavior-id", default=None)
    parser.add_argument("--behavior-name", default=None)

    parser.add_argument("--action-id", default=None)
    parser.add_argument("--action-name", default=None)

    add_common_runner_args(parser)

    return parser.parse_args()


def main():
    rospy.init_node("behavior_tree_control_executor")

    args = parse_args()
    config = build_config_from_args(args)

    executor = BehaviorTreeExecutor(config=config)
    tree = executor.load_tree(args.yaml)

    ok = executor.execute_by_select(
        tree,
        task_name=args.task_name,
        task_id=args.task_id,
        location_name=args.location_name,
        location_id=args.location_id,
        behavior_name=args.behavior_name,
        behavior_id=args.behavior_id,
        action_name=args.action_name,
        action_id=args.action_id,
    )

    if ok:
        rospy.loginfo("控制程序执行完成")
    else:
        rospy.logwarn("控制程序执行失败")


if __name__ == "__main__":
    main()
