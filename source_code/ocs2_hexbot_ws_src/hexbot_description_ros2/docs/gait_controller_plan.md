# 六足步态控制器规划

## 目标

为当前 18 自由度六足机器人构建一个 ROS 2 步态控制器，实现：

- 订阅机体速度命令
- 计算每个关节的目标角度
- 向 Isaac Sim 发布 `/joint_command`
- 先支持平地上的前进、转向、停止

当前机器人已经能在 Isaac 中稳定站立，因此下一步应该优先做确定性的运动学控制器，而不是直接上强化学习。

## 为什么这样做

Isaac Sim 已经提供了完整的关节控制链：

- `ROS2 Subscribe Joint State` 接收 `/joint_command`
- `Articulation Controller` 执行关节目标
- 外部接口天然就是 `/joint_states`、`/joint_command`、`/cmd_vel`、IMU 等数据

所以当前阶段不需要先做定制 Isaac 扩展，普通 ROS 2 节点就足够。

## 当前机器人事实

### 关节排列

当前工程暴露 18 个主动关节，按腿为：

- `RR`
- `RM`
- `RF`
- `LR`
- `LM`
- `LF`

每条腿依次包含：

- `coxa_joint_*`
- `femur_joint_*`
- `tarsus_joint_*`

### 现有步态分组

历史实现中已经有交替 tripod 分组基础，可作为快速平地步态的起点。

## 推荐控制器结构

建议拆成以下模块：

1. `stance_manager`  
   负责站姿落脚点和机体高度

2. `gait_scheduler`  
   负责 tripod A / tripod B 切换与相位管理

3. `foot_trajectory_generator`  
   负责摆动相轨迹与支撑相轨迹

4. `body_twist_to_foot_velocity`  
   把机体速度命令转换成脚端相对运动

5. `leg_ik`  
   完成单腿逆解

6. `joint_command_publisher`  
   负责按 Isaac 所需关节顺序发送命令

## 步态策略

### 第一阶段：只做交替 tripod

原因：

- 这是平地快速步态的标准方案
- 实现简单
- 便于先打通稳定行走链路

### 前进

- 机体指令使用正 `vx`
- 支撑腿在机体坐标系中向后划
- 摆动腿抬起并前送到下一落点

### 转向

- 机体指令使用非零 `wz`
- 外侧腿走更长弧线，内侧腿走更短弧线
- 初期先不改变抬腿高度

### 停止

- 不要在摆动中途把关节全部清零
- 应让当前摆动腿先落地
- 再回到站姿落脚点

## 实施顺序建议

1. 先实现稳定站立控制器
2. 再做单腿 IK 调试模式
3. 然后做交替 tripod
4. 再加入转向
5. 最后加入保护机制与超时回站

## 初始参数建议

- 控制频率：`100 Hz`
- 抬腿高度：`0.02 ~ 0.03 m`
- 初始步长：`0.015 m`
- 初始转向速度从小值开始

## 当前阶段不要先做的事情

- 不要直接从 RL 开始
- 不要先做复杂地形自适应
- 不要在 gait 还没稳定前就同时引入大量姿态控制

## 最小可行实现

建议最先具备以下模块：

1. `hexbot_gait_controller_node`
2. `hexapod_geometry.py`
3. `leg_ik.py`
4. `tripod_scheduler.py`
5. `foot_trajectory.py`

第一个里程碑不是高速行走，而是：

- 能通过 `/joint_command` 在 Isaac 中稳定站住
- 能走出一步 tripod
- 再实现连续前进
- 最后实现转向与平滑停止
