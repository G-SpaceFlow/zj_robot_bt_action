宿主机视觉话题打开->
打开视觉处理服务
->naviproject/src/robot_bt_action/scrips
python3 bt_target_service.py -> +终端 
开始动作
python3 control.py

# robot_bt_action 使用说明

这个包用 YAML 行为树描述机器人动作流程，用 Python 节点按任务、地点、行为、动作逐层执行。主要适合把“视觉识别 -> 取目标点 -> 手臂/手掌动作 -> 放置/调整”等流程写成可维护的配置。

## 主要文件

- `行为树.yaml`：行为树主配置。任务、地点、行为、动作都写在这里。
- `scripts/run_bt.py`：通用执行入口，可按 `task_id`、`behavior_id`、`action_id` 选择执行范围。
- `scripts/control.py`：自定义流程入口，在代码里的 `workflow` 中组合多个行为并设置循环次数。
- `scripts/bt_executor.py`：行为树解释器，负责读取 YAML、解析动作类型、调用 ROS 服务。
- `scripts/executor_services.py`：ROS 控制服务封装，包括 MoveJ、MoveL、手掌、视觉目标点等服务调用。
- `scripts/bt_target_server.py`：视觉目标点缓存服务，负责触发视觉、缓存点位、按 key/point/motion_mode 返回左右手目标位姿。
- `offset_params.yaml`：夹取或放置时使用的补偿参数，可通过 `offset_id` 引用。
- `srv/GetTarget.srv`、`srv/SetOffset.srv`：目标点和补偿服务定义。

## 编译和环境

在 ROS 工作空间中编译：

```bash
cd /navi_ws
catkin_make
source devel/setup.bash
```

如果脚本没有执行权限，可以加一次：

```bash
chmod +x src/py_demos/scripts/I2/cxl/scripts/*.py
```

运行前需要确保机器人控制、IK、手掌、视觉等相关服务已经启动。这个包会用到的常见接口包括：

- `/zj_humanoid/upperlimb/movej/whole_body`
- `/zj_humanoid/upperlimb/movej_by_path/whole_body`
- `/zj_humanoid/upperlimb/movel/dual_arm`
- `/zj_humanoid/upperlimb/IK/left_arm`
- `/zj_humanoid/upperlimb/IK/right_arm`
- `/bt_target_server/trigger_detect`
- `/bt_target_server/get_target`
- `/bt_target_server/set_offset`

## 启动目标点服务

视觉识别点位需要先启动目标点服务：

```bash
rosrun robot_bt_action bt_target_server.py
```

这个服务会：

- 向 `/yolo_vision/control` 发布视觉触发值。
- 向 `/yolo_vision/target_labels` 发布期望识别标签。
- 监听 `/yolo_vision/front_points_base_json` 和 `/yolo_vision/wall_angle`。
- 提供 `/bt_target_server/trigger_detect`、`/bt_target_server/get_target`、`/bt_target_server/set_offset`。

## 运行行为树

推荐用 `run_bt.py` 做单次调试。为了避免默认路径受中文文件名影响，建议显式传入 YAML 路径：

```bash
rosrun robot_bt_action run_bt.py --yaml /navi_ws/src/py_demos/scripts/I2/cxl/行为树.yaml
```

只执行某个任务：

```bash
rosrun robot_bt_action run_bt.py \
  --yaml /navi_ws/src/py_demos/scripts/I2/cxl/行为树.yaml \
  --task-id task_001
```

只执行某个行为：

```bash
rosrun robot_bt_action run_bt.py \
  --yaml /navi_ws/src/py_demos/scripts/I2/cxl/行为树.yaml \
  --task-id task_001 \
  --behavior-id behavior_001
```

只执行某个动作：

```bash
rosrun robot_bt_action run_bt.py \
  --yaml /navi_ws/src/py_demos/scripts/I2/cxl/行为树.yaml \
  --task-id task_001 \
  --behavior-id behavior_001 \
  --action-id detect_002
```

常用参数：

- `--task-id` / `--task-name`：选择任务。
- `--location-id` / `--location-name`：选择地点。
- `--behavior-id` / `--behavior-name`：选择行为。
- `--action-id` / `--action-name`：选择动作。
- `--no-skip-nav`：默认跳过导航；加上后不跳过 `nav_pose`。
- `--continue-on-action-fail`：动作失败后继续执行后续动作。

## 使用 control.py 编排固定流程

`scripts/control.py` 适合写固定生产流程。核心是修改 `workflow`：

```python
workflow = [
    {
        "name": "初始化行为",
        "repeat": 1,
        "steps": [
            {
                "task_id": "task_001",
                "behavior_id": "behavior_000",
            },
        ],
    },
    {
        "name": "循环行为",
        "repeat": 2,
        "steps": [
            {
                "task_id": "task_001",
                "behavior_id": "behavior_001",
            },
            {
                "task_id": "task_001",
                "behavior_id": "behavior_002",
            },
        ],
    },
]
```

字段说明：

- `name`：这一组动作的说明，只用于日志。
- `repeat`：这一组 `steps` 重复执行几次。
- `steps`：按顺序执行的行为或动作。
- `task_id`：对应 `行为树.yaml` 里的任务 id。
- `location_id`：可选，不写时执行任务下所有地点。
- `behavior_id`：可选，不写时执行地点下所有行为。
- `action_id`：可选，写了就只执行指定动作。

运行：

```bash
rosrun robot_bt_action control.py
```

如果要临时验证某一个动作，优先用 `run_bt.py`；如果要让多个行为按固定顺序循环，改 `control.py`。

## 导航服务与二次矫正

`control.py` 和 `run_bt.py` 在执行带 `nav_pose` 或 `twice_move` 的地点前，会等待导航封装服务：

```bash
rosrun robot_bt_action bt_navigation_server.py
```

服务启动后应能看到：

```bash
rosservice list | grep /bt_navigation_server/navigate_to_pose
```

常用启动顺序：

```bash
rosrun robot_bt_action bt_navigation_server.py
rosrun robot_bt_action control.py
```

`twice_move` 用于地图导航后的底盘二次矫正。典型配置：

```yaml
twice_move:
  enabled: true
  trigger_value: 4
  target:
    x: -2.33335
    y: 0.78240
    yaw: -3.00037
  vision_target:
    x: 0.9868
    y: 0.0271
    yaw: -0.0425
  params: {}
```

字段含义：

- `target.x/y/yaw`：地图/odom 坐标下的最终目标位姿。
- `vision_target.x/y`：视觉话题 `/yolo_vision/wall_angle` 中 `base_x/base_y` 的目标值。
- `vision_target.yaw`：默认不作为最终角度目标；当前角度默认使用 `/zj_humanoid/navigation/odom_info` 的 yaw，并以 `target.yaw` 为目标。
- `trigger_value`：发送给 `/yolo_vision/control` 的视觉触发值。
- `params`：覆盖二次矫正节点参数。

二次矫正默认视觉字段：

```yaml
params:
  vision_x_field: base_x
  vision_y_field: base_y
  vision_yaw_source: odom
  vision_yaw_target_source: target
```

当前坐标约定：

- 小车左移时 `base_y` 减小。
- 小车右移时 `base_y` 增大。
- odom yaw 逆时针增大，超过 `pi` 后跳到 `-pi`。
- 普通横向修正默认 `vision_lateral_sign: 1.0`。
- 特殊横向 nudge 中，`base_y` 偏大时会顺时针转出、后退、回正、前进，用于减小 `base_y`。

视觉丢失处理：

- 进入最终视觉矫正后，视觉短暂丢失会先停住等待。
- 超过 `vision_lost_hold_timeout`，默认 2 秒后，不再直接失败，而是临时切回地图/odom 运动。
- 视觉话题恢复后会自动回到最终视觉矫正。

最终对角处理：

- `align_final_yaw` 不只看 yaw。
- 如果旋转过程中距离又漂出 `position_tolerance`，会回到位置修正阶段，不会只因 yaw 到位就结束。

特殊横向修正常用参数：

```yaml
params:
  vision_lateral_nudge_enable: true
  vision_lateral_nudge_x_gate: 0.02
  vision_lateral_nudge_y_gate: 0.01
  vision_lateral_nudge_distance: 0.08
  vision_lateral_nudge_speed: 0.012
  vision_lateral_nudge_step_pause: 0.2
  vision_lateral_nudge_settle_delay: 1.0
```

当前特殊横向修正顺序：转出角度 -> 斜向后退 -> 反向转出角度 -> 斜向前进 -> 最终回正 -> 等待视觉稳定。

特殊横向修正按固定周期执行：周期开始后不会再用视觉 x/y/yaw 改动作，只用里程计 yaw 控制旋转，等整套动作结束后再重新读取视觉误差。

参数说明：

- `vision_lateral_nudge_x_gate`：只有前后误差足够小时，才允许进入特殊横向修正；数值越大越容易触发。
- `vision_lateral_nudge_y_gate`：横向误差超过该值才触发；数值越小越容易触发。
- `vision_lateral_nudge_distance`：斜向后退距离，反向出角后的斜向前进也使用同一距离。
- `vision_lateral_nudge_speed`：特殊横向修正前进/后退速度。
- `vision_lateral_nudge_turn_pause`：旋转到位后、开始前进/后退前的等待时间，用来等底盘角速度真正停稳。
- `vision_lateral_nudge_step_pause`：每一步减速停止后额外等待时间，用于消除底盘惯性。
- `vision_lateral_nudge_settle_delay`：整套特殊横向修正结束后等待视觉稳定的时间。

调试建议：

- 如果日志中没有 `Start vision lateral nudge`，说明还没进入特殊横向修正，优先检查 `error_x/error_y` 是否满足 gate。
- 如果已经进入但方向反了，优先检查 `vision_lateral_nudge_turn_sign` 和 `vision_lateral_nudge_yaw_sign` 是否被 ROS 参数覆盖。
- 修改 `twice_move_corrector.py` 后需要重启 `bt_navigation_server.py`，否则仍会运行旧代码。
- 可用 `rosparam get /bt_navigation_server/参数名` 检查当前实际参数。

## 行为树 YAML 基本结构

`行为树.yaml` 的层级是：

```yaml
tasks:
  - id: task_001
    name: plate_pick_task
    label: 任务说明
    locations:
      - id: loc_001
        name: pickup_station
        nav_pose:
          x: 1.2
          y: 0.35
          yaw: 1.57
        behaviors:
          - id: behavior_001
            name: approach_plate
            actions:
              - id: action_001
                name: some_action
```

选择执行时，`run_bt.py` 会按 `task -> location -> behavior -> action` 过滤。比如只给 `--task-id task_001`，会执行这个任务下所有地点、行为和动作；如果同时给 `--behavior-id behavior_001`，就只执行该行为。

## 常见动作写法

### 1. 视觉识别动作

```yaml
- id: detect_002
  name: detect_target
  label: 视觉识别：夹取左右点
  type: vision
  key: "1-1-1-2"
  trigger_value: 2
  labels: ["tray"]
  motion_mode: 1
  points: ["left", "right", "center", "top"]
  optional: true
```

字段说明：

- `type: vision`：表示这是视觉动作。
- `key`：视觉结果缓存 key，后续动作用 `base_key` 引用它。
- `trigger_value`：发送给视觉节点的控制值。
- `labels`：希望识别的目标标签。
- `motion_mode`：目标点解释模式。
- `points`：本次需要缓存或读取的点名。
- `optional: true`：视觉失败也继续执行后续动作；不希望继续时设为 `false`。

### 2. 关节运动 MoveJ

```yaml
- id: rotate_001
  name: move waist
  service: /zj_humanoid/upperlimb/movej/whole_body
  use_ik: false
  joints:
    - [0.7, 0.0, 0.1]
  motion:
    v: 0.5
    acc: 0.05
    t: 4.0
    is_async: false
    arm_type: 24
```

`use_ik: false` 表示直接使用 `joints`。如果 `use_ik` 不写，默认会按 IK 动作处理，需要提供 `arm_poses` 和参考关节。

### 3. 关节轨迹 MoveJByPath

```yaml
- id: action_001
  name: left_arm_three_point_joint_traj
  service: /zj_humanoid/upperlimb/movej_by_path/whole_body
  motion:
    time: 12.0
    timestamp: []
    is_async: false
    arm_type: 2
  path:
    - - [-0.7, -1.175, 1.0835, -1.0769, -0.2203, 0.2589, 0.6207]
    - - [-1.4492, -1.575, 1.7835, -1.0769, -0.2203, 0.489, 0.8207]
```

每个 `path` 点可以按分组写，执行时会自动展平成一维关节数组。

### 4. 双臂 MoveL 使用视觉点

```yaml
- id: action_002
  name: dual_arm_moveL_to_target
  service: /zj_humanoid/upperlimb/movel/dual_arm
  motion:
    v: 0.5
    acc: 0.2
    is_async: false
  base_key: "1-1-1-2"
  motion_mode: 1
  points: ["left", "right"]
```

这里会从目标点服务中读取 `key=1-1-1-2` 的 `left` 和 `right` 点，作为左右手目标。

### 5. 双臂 MoveL 使用相对偏移

```yaml
- id: action_003
  name: dual_arm_relative_move
  service: /zj_humanoid/upperlimb/movel/dual_arm
  motion:
    v: 0.5
    acc: 0.2
    is_async: false
  relative_offset:
    left:  {dx: 0.0, dy: 0.0, dz: 0.05}
    right: {dx: 0.0, dy: 0.0, dz: 0.05}
```

没有 `base_key` 时，`relative_offset` 基于当前左右手末端位姿做偏移。

### 6. 双臂 MoveL 使用视觉标量偏移

适用于视觉只返回一个标量偏移量的场景，例如 `/yolo_vision/mode5_points_json`：

```json
{"frame_id":"BASE","y_offset":-0.0079,"y_method":"2d"}
```

先触发视觉并缓存 `y_offset`：

```yaml
- id: detect_001
  name: detect_target
  type: vision
  key: "3-1-1-1"
  trigger_value: 5
  labels: ["tray"]
  points: ["y_offset"]
  motion_mode: 6
```

再用 `motion_mode: 6` 将标量写入手臂相对位移。`index` 表示修改哪个轴：`0=dx`，`1=dy`，`2=dz`。最终修正量为 `y_offset - initial_point`，小于阈值时跳过 MoveL。

```yaml
- id: action_001
  name: dual_arm_moveL_to_place
  service: /zj_humanoid/upperlimb/movel/dual_arm
  motion:
    v: 0.5
    acc: 0.1
    is_async: false
  motion_mode: 6
  relative_offset:
    left: {dx: 0.00, dy: 0.00, dz: 0.00}
    right: {dx: 0.00, dy: 0.00, dz: 0.00}
    rotate_with_waist: false
  mode_source:
    index: 1
    base_key: "3-1-1-1"
    points: ["y_offset"]
    initial_point: 0.004
    hands: ["left", "right"]
    skip_if_distance_less_than:
      point: "y_offset"
      threshold: 0.005
```

`mode_source.base_key` 必须和前面的视觉动作 `key` 一致。因为该视觉话题声明 `frame_id: BASE`，示例中使用 `rotate_with_waist: false`，避免把 BASE 坐标下的偏移再次按腰部角度旋转。

### 7. 手掌动作

```yaml
- id: hand_001
  name: close_hand
  type: hand
  service: /zj_humanoid/hand/joint_switch
  offset_id: "tray_1"
  q: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
```

`q` 必须是 12 个数。配置 `offset_id` 时，会先从 `offset_params.yaml` 读取补偿并调用 `/bt_target_server/set_offset`。

## motion_mode 说明

`GetTarget.srv` 中定义了 `motion_mode`：

- `1`：左右手分别使用指定左右点，常见写法是 `points: ["left", "right"]`。
- `2`：使用单点作为双手中点目标，常见写法是 `points: ["center"]`。
- `3`：使用单点作为某只手或双手目标，并结合 `relative_offset` 决定左右手偏移。
- `4`：使用目标点和当前点计算差值，适合视觉纠偏。
- `5`：左右手分别使用单点目标，适合左/右手各有当前点和目标点的纠偏。
- `6`：使用 `y_offset` 这类视觉标量作为手臂相对偏移修正，标量通过 `Pose.position.x` 返回。

常用规则：

- `points` 在 YAML 中是数组，例如 `["left", "right"]`。
- 终端调用服务时也要写成数组元素，不能写成 `['left, right']`。
- 单点视觉动作如果没有显式写 `motion_mode`，程序会自动按 `motion_mode=4` 缓存；`trigger_value: 5` 会自动按 `motion_mode=6` 缓存。

## 视觉服务手动调试

触发视觉并缓存点位：

```bash
rosservice call /bt_target_server/trigger_detect "key: '1-1-1-2'
trigger_value: 2
labels: ['tray']
motion_mode: 1
point_names: ['left', 'right']"
```

读取缓存点位：

```bash
rosservice call /bt_target_server/get_target "key: '1-1-1-2'
trigger_value: 2
labels: ['tray']
motion_mode: 1
point_names: ['left', 'right']"
```

注意：

- `point_names: ['left', 'right']` 是两个点。
- `point_names: ['left, right']` 是一个点，名字叫 `left, right`，通常是错的。
- `labels` 要和视觉输出标签一致，例如料盘通常写 `['tray']`。

## offset_params.yaml

补偿参数按 id 管理：

```yaml
offsets:
  - id: "tray_1"
    left_dx: -0.077409
    left_dy: 0.0207
    left_dz: 0.14857
    right_dx: -0.08933
    right_dy: -0.01412
    right_dz: 0.14926
    left_ox: 0.0470
    left_oy: 0.168
    left_oz: 0.0309
    left_ow: -0.9846
    right_ox: 0.0470
    right_oy: -0.168
    right_oz: -0.0309
    right_ow: 0.9846
```

在 YAML 动作中通过 `offset_id: "tray_1"` 使用。新增补偿时，复制一组并改 `id` 和对应数值即可。

## 修改建议

1. 新增动作时，先给唯一的 `id`，再确认 `service` 类型是否能被 `bt_executor.py` 识别。
2. 调视觉点时，先单独执行 `detect_xxx`，确认服务返回成功，再让 MoveL 动作引用它的 `base_key`。
3. 调试危险动作时，优先只跑单个 `action_id`，不要直接跑完整任务。
4. 修改 `control.py` 前，先用 `run_bt.py` 把每个行为单独跑通。
5. `optional: true` 只适合非关键视觉动作；夹取、放置依赖的关键点建议设为 `false`。
6. 中文路径或中文文件名在不同终端环境下可能有编码问题，运行时建议显式传 `--yaml` 绝对路径。

