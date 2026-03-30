# Isaac 资产修复说明

## 关节链结构

当前六足机器人每条腿的结构为：

- `base_link -> leg_center_*`：固定关节
- `leg_center_* -> coxa_*`：coxa 转动关节
- `coxa_* -> femur_*`：femur 转动关节
- `femur_* -> tarsus_*`：tarsus 转动关节
- `tarsus_* -> foot_*`：固定脚端关节

总共有 18 个主动转动关节。

## 为什么需要这套修复脚本

URDF 本身虽然可以正常导入，但 Isaac 导出的 USD 资产曾经存在两个典型问题：

- 18 个关节的驱动参数没有可靠保留
- `foot_*` 脚端可能只剩下空占位 prim，没有真正可用的碰撞几何

这会导致：

- 机器人在仿真里发软、站不稳
- 脚端高摩擦材质虽然“看起来绑定了”，但实际上没有落在真实碰撞体上

## 当前修复思路

现在把 URDF 与 USD 后处理视为一条完整链路：

- URDF 中必须保留显式 `foot_*` 固定链路
- 每个脚端碰撞球统一使用稳定名字 `generated_collision_sphere`
- 若 Isaac reimport 后仍只得到空 `foot_*` 占位 prim，则再由后处理脚本补出真实球形碰撞体
- 再把驱动参数与脚端高摩擦材质写回 USD

## 2026-03-27 最新风险提示

到 2026-03-27 这轮夜间调试为止，又确认了一个比 drive 参数更靠前的风险：

- `hexbot_isaac_rooted_reimport.usd` 可能把坏的物理状态直接写回磁盘
- 典型症状是：
  - `/hexbot/hexbot` 出现异常的本地 `xformOp:translate/orient`
  - 多个关节的 `state:angular:physics:position` 出现十几到二十多的异常值
  - 即使没有任何 ROS 控制发布者，机器人也会直接趴地

这一轮已经确认：

- 旧文件  
  `hexbot_description_ros2/urdf/hexbot_isaac_rooted/hexbot_isaac_rooted.usd`
  没有这些坏状态
- 目标文件  
  `hexbot_description_ros2/urdf/hexbot_isaac_rooted_reimport/hexbot_isaac_rooted_reimport.usd`
  出现了坏 `xform/state`
- 因此 `reload-stage` 重新打开同一路径时，不等于“干净复位”

## 主要文件

- `foot_collision_defs.py`：统一读取 URDF 中脚端碰撞球的名字、半径和位姿
- `reimport_hexbot_urdf_asset.py`：重新导入 URDF，检查脚端 prim 是否完整
- `synthesize_hexbot_foot_colliders.py`：在脚端下补写显式球形碰撞体
- `apply_hexbot_isaac_drives.py`：写入 18 个关节驱动参数，并给脚端绑定高摩擦材质
- `audit_hexbot_isaac_setup.py`：检查驱动参数、材质绑定、命名脚端碰撞体是否完整
- `inspect_hexbot_foot_prims.py`：查看 live stage 或 USD 中脚端 prim 树结构
- `run_hexbot_isaac_repair_pipeline.py`：安全入口脚本，默认只预览、不写盘

## 安全执行方式

建议使用 Isaac Sim 自带 Python 运行，而不是系统 Python。

默认安全预览模式：

```bash
./python.sh /path/to/hexbot_description_ros2/isaac/run_hexbot_isaac_repair_pipeline.py
```

这个模式会：

- 检查参数
- 读取并检查当前 USD
- 预览将要做的修复动作
- 进行审计
- 不写回你的 USD 文件

只有在你显式加上 `--write` 时，才允许真正写盘：

```bash
./python.sh /path/to/hexbot_description_ros2/isaac/run_hexbot_isaac_repair_pipeline.py --write
```

但结合 2026-03-27 这轮实测，`--write` 需要再加一条经验约束：

- 最好不要在“当前已经加载了目标 reimport USD 的 live Isaac 实例”里直接写盘
- 更稳妥的方式是：
  1. 关闭当前占用该 USD 的 Isaac 实例
  2. 用独立、干净的 Isaac 进程执行 `--write`
  3. 写盘完成后重新打开新的 Isaac 会话验证

## 单独运行某个脚本

例如只做严格审计：

```bash
./python.sh /path/to/hexbot_description_ros2/isaac/audit_hexbot_isaac_setup.py --strict
```

## 运行中 Isaac Sim 远程执行

如果已经启用了：

- `isaacsim.code_editor.vscode`
- `omni.kit.debug.vscode`
- VS Code 端 `Isaac Sim VS Code Edition`

那么运行中的 Isaac Sim 默认会打开两个常用入口：

- `127.0.0.1:8226`
  用于把 Python 代码直接发进当前 Isaac Sim 实例执行
- `127.0.0.1:3000`
  用于 VS Code Python Attach 调试

这个仓库里现在提供了一个终端侧的小工具：

```bash
python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac/isaac_vscode_exec.py \
  --code "print('hello from isaac')"
```

也可以直接执行文件：

```bash
python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac/isaac_vscode_exec.py \
  --file /绝对路径/example.py
```

例如查询当前 stage 和 timeline：

```bash
python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac/isaac_vscode_exec.py --code "
import json
import omni.timeline
import omni.usd
timeline = omni.timeline.get_timeline_interface()
print(json.dumps({
    'stage_url': omni.usd.get_context().get_stage_url(),
    'is_playing': bool(timeline.is_playing()),
    'is_stopped': bool(timeline.is_stopped()),
}))
"
```

说明：

- `8226` 这条链路本质上是 VS Code 集成扩展使用的明文 TCP 执行口
- 返回值是 JSON，常见字段为 `status` 与 `output`
- 这条能力适合做轻量查询、临时执行和自动化排查
- 如果要做断点、单步和变量观察，仍应优先使用 `3000` attach 调试

如果只想做常见运行控制，不想每次手写 `--code`，现在也可以直接用：

```bash
python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac/isaac_runtime_control.py status
python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac/isaac_runtime_control.py stop
python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac/isaac_runtime_control.py play
python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac/isaac_runtime_control.py reload-stage
python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac/isaac_runtime_control.py reload-and-play
python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac/isaac_runtime_control.py articulation-roots
```

补充说明：

- 在当前 Isaac Sim 4.5.0 实例里，`play/stop` 返回的布尔值不总是和即时物理状态严格同步
- 更可靠的判断方式仍然是宿主机话题探针，例如 `/joint_states`、`/odom`、`/clock` 是否重新开始发布
- 当前 stage 已确认的 articulation root 是 `/hexbot/hexbot`

如果要进一步诊断“为什么 `/joint_command` 已经回到 stand，但 `/joint_states` 仍明显偏离”，现在可以直接运行：

```bash
source /opt/ros/humble/setup.bash
source /home/u-zhuang/ros2_ws/install/setup.bash
source /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/install_humble/setup.bash
python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac/diagnose_hexbot_live_state.py
```

这个脚本会同时输出：

- 配置文件理论 stand 关节角
- 当前 `/joint_command`
- 当前 `/joint_states`
- Isaac articulation `get_applied_actions()`
- articulation solver iterations 与 world pose

2026-03-27 这轮实测里，它已经确认：

- `expected stand == /joint_command == Isaac applied action`
- 但 `Isaac joint state` 仍可残留约 `0.19 ~ 0.20 rad` 误差
- 当前 live articulation solver 配置仍是 `position=32 / velocity=1`

同一轮后续又确认了更靠前的资产层问题：

- 直接读取磁盘上的  
  `hexbot_isaac_rooted_reimport.usd`
  也能看到坏的 `xform/state`
- 说明问题不只是 live stage 漂移，而是 USD 本体可能已经被污染

另外，这一轮还做过一次 live 写盘 repair pipeline，结果是：

- Isaac Sim 卡住
- `hexbot_isaac_rooted_reimport.usd` 的文件大小从 `163142` 字节变成了 `687` 字节

因此当前建议非常明确：

- 若怀疑 reimport USD 已被污染，优先重新生成资产，而不是继续在其上做 gait / OCS2 调试
- 在重新生成前，不要把当前这份 reimport USD 当作可信基线

## 当前 drive profile 状态

到 2026-03-27 这一轮为止：

- `hexbot_drive_profile.json` 已默认指向  
  `../urdf/hexbot_isaac_rooted_reimport/hexbot_isaac_rooted_reimport.usd`
- `joint_targets_deg` 已经正式对齐到  
  `ocs2_hexbot_legged_robot/config/command/reference.info`
  的 18 关节中性站姿

这意味着：

- 后续重新生成并写回 reimport USD 时，drive 默认站姿已经不再是全 0 角
- 但如果当前磁盘上的 reimport USD 已经被坏状态污染，仍需要先重新生成资产本体

## 调参说明

可通过 `hexbot_drive_profile.json` 调整：

- `stiffness`：位置控制比例项
- `damping`：位置控制阻尼项
- `max_force`：力矩上限
- `max_joint_velocity`：关节最大速度
- `target_position_deg`：默认站立目标角
- `joint_targets_deg`：逐关节覆写目标角
- `foot_material`：脚端高摩擦材质参数
