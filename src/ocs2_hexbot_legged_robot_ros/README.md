# ocs2_hexbot_legged_robot_ros

这个包是从上游 `ocs2_legged_robot_ros` 恢复并派生出来的 Hexbot 六足 ROS 侧适配包。

当前主运行链已经收口到 Hexbot：

- 默认 URDF 入口是 `hexbot_description_ros2/urdf/hexbot_isaac_rooted.urdf`
- 默认 Python launch 入口是 `hexbot_sqp.launch.py`
- 运行时节点名前缀和 OCS2 话题前缀统一为 `hexbot`
- 可视化层不再硬编码 AnyMal 的 4 足 12 关节，而是按 Hexbot 的 6 足 18 关节发布

新增的两个关键节点：

- `hexbot_cmd_vel_target`
  - 订阅 `/cmd_vel`
  - 订阅 `hexbot_mpc_observation`
  - 发布 `hexbot_mpc_target`
  - 作用：把上层导航速度命令转成 OCS2 目标轨迹

- `hexbot_auto_gait_command`
  - 订阅 `/cmd_vel`
  - 发布 `hexbot_mpc_mode_schedule`
  - 作用：在 `stance` 和 `tripod_cycle` 之间自动切换，不再依赖键盘选步态

另外，`legged_robot_dummy` 现在还会额外发布：

- `/hexbot_mpc/output/joint_command`
  - 消息类型：`sensor_msgs/JointState`
  - 来源：OCS2 dummy/MRT 当前轨迹的下一步关节位置
  - 作用：对接 `hexbot_locomotion_mpc_bridge.py` 的后端命令入口

推荐启动方式：

```bash
ros2 launch ocs2_hexbot_legged_robot_ros hexbot_sqp.launch.py
```

当前状态：

- ROS 侧主入口已经脱离键盘示例
- `rviz` 默认关闭，避免旧 AnyMal 配置误导
- 真正编译运行仍需要在 Jazzy 容器里把 OCS2 依赖闭包编过
