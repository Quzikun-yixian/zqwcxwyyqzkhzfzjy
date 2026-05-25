# 协作规则

1. 模型负责人只改 `assets/`、`models/`、`configs/robot_description.yaml`。
2. 夹爪负责人只改 `src/gripper/` 和 `configs/gripper_config.yaml`。
3. 控制负责人只改 `src/control/` 和 `configs/controller_config.yaml`。
4. 轨迹规划负责人只改 `src/planning/` 和 `configs/trajectory_config.yaml`。
5. IK 负责人只改 `src/ik/` 和 `configs/ik_config.yaml`。
6. 轴孔对接策略负责人只改 `src/task/peg_in_hole/` 和 `configs/peg_in_hole_task.yaml`。
7. GUI 只调用接口，不写核心算法。
8. 所有人必须遵守 `robot_description.yaml` 中的 joint/site 命名。
