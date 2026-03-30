# Hexbot Seed

这个目录保存的是 Hexbot 六足机器人接入 OCS2 时需要的最小种子配置。

包含内容：

- `hexbot_mapping.yaml`：六足腿顺序、足端命名、18 个驱动关节顺序、默认站姿。
- `reference.info`：和当前恢复工程一致的参考站姿与初始模式模板。

用途：

- 作为把上游四足 `ocs2_legged_robot` 改造成 Hexbot 六足版本时的对照基线。
- 作为 `hexbot_locomotion_mpc` 桥接包导出/校验配置时的种子输入。

限制：

- 这不是完整可运行的六足 OCS2 后端。
- 上游代码里仍有不少四足假设，需要继续做结构泛化和编译验证。
