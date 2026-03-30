# ocs2_hexbot_legged_robot

这个包是从上游 `ocs2_legged_robot` 恢复并派生出来的 Hexbot 六足版本骨架。

当前已经完成的恢复项：

- `feet_array_t` / `contact_flag_t` 从 4 足改成 6 足。
- `ModeNumber` 改成 Hexbot 六足位掩码，包含 `TRIPOD_A`、`TRIPOD_B`、`STANCE`。
- `jointNames`、`contactNames3DoF`、默认站姿全部替换成 Hexbot 当前基线。
- `task.info`、`reference.info`、`gait.info` 改成六足种子配置。

当前仍未完成的项：

- 上游 OCS2 `ros2` 分支对环境的预期是 Ubuntu 24.04 + ROS 2 Jazzy，和当前 Humble 工作区不完全一致。
- 还需要继续排查并泛化上游残留的四足专用实现。
- 还没有完成完整编译与闭环运行验证。

这个包现在的作用是保住恢复后的六足结构和配置，不让工程再次只剩“记忆里的改动”。
