#!/usr/bin/env python3
# -*- coding: utf-8 -*-

DEFAULT_EXEC_CONFIG = {
    "skip_nav": False,
    "skip_unknown_action": True,
    "continue_on_action_fail": False,
    "continue_on_behavior_fail": False,
    "continue_on_location_fail": False,
}


def build_exec_config(overrides=None):
    config = DEFAULT_EXEC_CONFIG.copy()
    if overrides:
        config.update(overrides)
    return config


def add_common_runner_args(parser):
    parser.add_argument(
        "--skip-nav",
        action="store_true",
        help="默认执行 nav_pose 导航；加上该参数后跳过导航",
    )
    parser.add_argument(
        "--continue-on-action-fail",
        action="store_true",
        help="动作失败后继续执行后续动作",
    )


def build_config_from_args(args):
    overrides = {}

    if getattr(args, "skip_nav", False):
        overrides["skip_nav"] = True

    if getattr(args, "continue_on_action_fail", False):
        overrides["continue_on_action_fail"] = True

    return build_exec_config(overrides)
