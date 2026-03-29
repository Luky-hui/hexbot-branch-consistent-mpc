# 六足机器人重构准备说明

## 当前优先级说明

这份文档现在属于“次级探索分支”，不是当前第一主线。

当前真正的第一主线已经切换为：

- 把 `hexbot_locomotion_mpc + ocs2_hexbot_legged_robot_ros` 做成跨环境闭环成品
- 形成旧 baseline vs 新 OCS2 主线的 A/B 证据
- 为发明专利技术交底书准备系统方案、发明点和实施例材料

因此，本文涉及的 `hexapod_ros` / `OpenSHC` 参考重构，现阶段只作为后续优化输入，不应抢占闭环成品化优先级。

## 目标

在不破坏当前可运行 V2 链路的前提下，为后续 gait / 参数组织重构做准备。当前准备的主要外部参考对象有两个：

- `hexapod_ros`
- `OpenSHC`

这里的目标不是一次性替换现有工程，也不是当前专利主线，而是建立一个可回退、可 A/B 对比的后续优化路径。

## 参考方向

### 1. `hexapod_ros`

它的重要性在于：

- 与当前机器人几何尺度最接近
- 同时包含步态、里程计、建图、导航等完整链路
- 已在类似 PhantomX 六足平台上经过验证

优先借鉴内容：

- stance 几何
- 站立高度语义
- 抬腿高度
- 步态周期逻辑
- IMU 自动调平行为

### 2. `OpenSHC`

它的重要性在于：

- 多足控制架构成熟
- 参数组织方式明显优于当前 V2
- gait、body pose、IMU pose、terrain logic 分层更清晰

优先借鉴内容：

- `body_clearance`
- `step_frequency`
- `swing_height`
- 每条腿的 `stance_position`
- `imu_posing`
- `inclination_posing`

## 当前边界

当前真正参与运行并属于专利主线的包应视具体阶段区分：

- 历史 baseline：
  - `hexbot_description_ros2`
  - `hexbot_locomotion_v2`
  - `hexbot_isaac_runtime`
- 当前 OCS2 主线：
  - `hexbot_description_ros2`
  - `hexbot_locomotion_mpc`
  - `ocs2_hexbot_legged_robot`
  - `ocs2_hexbot_legged_robot_ros`

第三方参考仍然只作为只读输入，不直接参与正常构建与运行。

## 2026-03-27 当前必须先解决的非参数问题

在继续做 `hexapod_ros` / `OpenSHC` 风格参数借鉴之前，当前又确认了一个更靠前的阻塞：

- `hexbot_isaac_rooted_reimport.usd` 已被证明可能携带坏的 `xformOp` 和 `state:angular:physics:*` 状态
- `reload-stage` 重新打开的是同一个坏 USD，不等于干净复位
- 因此“躯干持续下沉、腿还在地上但整机越来越趴”的现象，并不一定首先来自 gait 参数

到 2026-03-27 这轮最后状态为止：

- `hexbot_drive_profile.json` 已经改到与 `reference.info` 中性站姿一致
- 但在 live Isaac 里直接执行写盘 repair pipeline 时，Isaac 出现卡住
- 同时 `hexbot_isaac_rooted_reimport.usd` 的文件大小从 `163142` 字节变成了 `687` 字节

因此，后续所有 stance / gait 参数重构都应建立在以下前提上：

- 先把 Isaac reimport 资产恢复成可信版本
- 再验证“无 OCS2、仅靠 Isaac drive 目标”能否稳定站立
- 最后再谈 `hexapod_ros` / `OpenSHC` 风格参数迁移

## 实施顺序建议

1. 先把当前 V2 参数与 `hexapod_ros` / `OpenSHC` 做逐项对比
2. 利用现有 `geometry_config_path` 启动参数做配置 A/B，而不是直接改默认基线
3. 先做 `hexapod_ros` 风格的 stance 与稳定性试验
4. 再做 `OpenSHC` 风格的参数组织重构
5. 最后再判断是否需要单独拆出 `v3` 包

## 当前产出

当前最直接的准备产物是：

- `hexapod_reference_parameter_comparison.md`

## 当前工程状态

和最初阶段相比，工程已经向前推进了一步，但主线重心也已经变化：

- 显式 stance 语义已经在 `hexbot_locomotion_v2` 中实现
- 历史 tripod baseline 仍可作为 gait 对比锚点
- 当前专利主线已经转向  
  `hexbot_locomotion_mpc + ocs2_hexbot_legged_robot_ros`
- 后续 `hexapod_ros` / `OpenSHC` 风格重构都应放在专利闭环成品化之后
- 并且在当前阶段，专利主线继续推进前还必须先解决 Isaac 资产重置与 reimport 一致性问题
