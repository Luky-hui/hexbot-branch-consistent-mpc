# hexbot_locomotion_mpc

这个包是 Hexbot 恢复工程里的 MPC/WBC 桥接层，不是伪装成“已经完成”的控制器本体。

当前职责：

- 保持宿主机现有 ROS2 运行边界不变：
  - `/cmd_vel`
  - `/joint_states`
  - `/imu`
  - `/odom`
  - `/joint_command`
- 将这些运行时话题桥接到 OCS2 后端命名空间：
  - `/hexbot_mpc/input/cmd_vel`
  - `/hexbot_mpc/input/joint_states`
  - `/hexbot_mpc/input/imu`
  - `/hexbot_mpc/input/odom`
  - `/hexbot_mpc/output/joint_command`
- 提供 URDF 就绪性检查脚本，验证：
  - 18 个驱动关节是否齐全
  - 6 个脚端 `generated_collision_sphere` 是否存在
  - 惯性结构是否满足第一轮 OCS2 接入要求
- 提供宿主机运行态话题探针，验证：
  - `/clock`
  - `/joint_states`
  - `/odom`
  是否真的已经从 Isaac Sim + ROS bridge 发布出来
- 提供 OCS2 Hexbot seed 导出和第三方源码拉取脚本

当前运行事实：

- `ocs2_hexbot_legged_robot_ros` 里已经有：
  - `hexbot_cmd_vel_target`
  - `hexbot_auto_gait_command`
  - `hexbot_runtime_mrt`
- 现在主链已经不是只靠 `legged_robot_dummy` 做演示。
- 默认闭环是：
  - `bridge` 转发宿主机话题
  - `hexbot_runtime_mrt` 用真实 `/odom + /joint_states` 生成 observation
  - OCS2 策略评估后回写 `/hexbot_mpc/output/joint_command`
  - bridge 再转回宿主机 `/joint_command`

当前限制：

- 这个包本身仍不包含 OCS2 求解器实现。
- 真正的六足 OCS2 主体在：
  - `ocs2_hexbot_legged_robot`
  - `ocs2_hexbot_legged_robot_ros`
- 上游 OCS2 `ros2` 线偏向 Jazzy，真正编译仍需在 Jazzy 容器完成。
- Isaac Sim 进程“开着”不等于运行链 ready；如果 `/clock + /joint_states + /odom` 没出来，bridge 和 OCS2 都不该继续往下查。

常用命令：

```bash
python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_mpc/src/check_hexbot_mpc_readiness.py \
  --urdf /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/urdf/hexbot_isaac_rooted.urdf
```

```bash
python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_mpc/src/check_hexbot_runtime_topics.py
```

```bash
python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_mpc/src/probe_hexbot_closed_loop.py \
  --use-sim-time \
  --duration 5.0 \
  --linear-x 0.03 \
  --angular-z 0.0
```

```bash
bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_mpc/scripts/run_patent_closed_loop_case.sh
```

```bash
ros2 launch hexbot_locomotion_mpc hexbot_locomotion_mpc.launch.py \
  use_sim_time:=true \
  geometry_config_path:=/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_mpc/config/hexbot_mpc_default.yaml
```

如果要固定 Jazzy 容器运行边界，现在推荐：

```bash
bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/container_run_hexbot_jazzy.sh
bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/container_exec_hexbot_jazzy.sh
```
