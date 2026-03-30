# 运行时导航包说明

`hexbot_isaac_runtime` 是当前工程中负责建图、导航和 RViz 的运行时包。

## 主要启动文件

### RTAB-Map 建图

```bash
ros2 launch hexbot_isaac_runtime rtab-map-scan.launch.py use_sim_time:=true launch_viz:=false
```

### Navigation2

```bash
ros2 launch hexbot_isaac_runtime navigation2.launch.py use_sim_time:=true
```

### RViz

```bash
ros2 launch hexbot_isaac_runtime rviz.launch.py use_sim_time:=true
```

### AprilTag

```bash
ros2 launch hexbot_isaac_runtime apriltag.launch.py use_sim_time:=true
```

## 速度缩放

建议 locomotion 与 Nav2 使用相同的 `speed_scale`，保持指令范围一致：

```bash
ros2 launch hexbot_locomotion_v2 hexbot_locomotion_v2.launch.py use_sim_time:=true mode:=tripod speed_scale:=2.0
ros2 launch hexbot_isaac_runtime navigation2.launch.py use_sim_time:=true speed_scale:=2.0
```

也可以直接覆盖 Nav2 的具体速度与加速度限制：

```bash
ros2 launch hexbot_isaac_runtime navigation2.launch.py \
  use_sim_time:=true \
  nav_max_vel_x:=0.30 \
  nav_max_vel_theta:=0.90 \
  nav_acc_lim_x:=0.70 \
  nav_acc_lim_theta:=1.40
```

## 运行注意事项

- RViz 的 `Fixed Frame` 必须设置为 `map`
- 建议使用 `rviz/hexbot_navigation.rviz`
- 更完整的基线流程见根目录 `hexbot_navigation_runtime_baseline.md`
- 当前交付版不包含历史失败实验与第三方大仓库，只保留运行所需内容
