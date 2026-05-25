# 算法组使用说明

本说明面向算法组。算法组不需要直接修改 MuJoCo XML、不需要直接操作 `data.qpos` / `data.ctrl`，优先通过平台已经封装好的 API 和 demo 脚本接入算法。

---

## 1. 算法组现在可以直接使用什么？

当前平台已经提供三类控制能力：

```text
1. 位置控制 / 运动学接口
   用于 FK、IK、轨迹规划、夹爪开合、基础抓取流程。

2. Joint4 导纳力控
   用于稳定的接触力反馈演示：
   F_error -> q4_cmd。

3. Stage 3 v2 力矩级力控
   用于更接近真实机器人底层控制的仿真：
   F_error -> F_task -> J^T F_task -> tau。
```

推荐算法组开发顺序：

```text
Step 1：先用 ArmPlatformAPI 做轨迹规划、IK、夹爪开合。
Step 2：接触阶段先调用 joint4 导纳力控。
Step 3：需要展示高级控制时，再运行 Stage 3 v2 力矩级力控。
Step 4：轴孔对接算法在此基础上加入 spiral search / wiggling / screwing。
```

---

## 2. 必须遵守的变量约定

机械臂本体关节顺序固定为：

```text
q_arm = [q1, q2, q3, d4]
```

含义：

```text
q1 -> joint1, revolute, rad
q2 -> joint2, revolute, rad
q3 -> joint3, revolute, rad
d4 -> joint4, prismatic, m
```

夹爪只通过一个变量控制：

```text
g = gripper_opening
```

不要直接控制 `joint6`。`joint6` 是 mimic joint，关系为：

```text
joint6 = -joint5
```

---

## 3. 算法组不要直接改哪些文件？

算法组一般不要修改：

```text
models/robot_with_gripper.xml
models/_*.xml
assets/meshes/
src/kinematics/
src/gripper/
src/api/arm_platform_api.py
src/control/joint_space_controller.py
```

这些属于平台层。算法组应优先新建或修改：

```text
experiments/
src/task/peg_in_hole/
scripts/run_algorithm_*.py
```

如果确实需要改模型、site、接触体或 actuator，需要先和仿真平台负责人确认。

---

## 4. 最小可用 API 示例：读取状态、IK、运动到目标点

新建一个脚本，例如：

```text
scripts/run_algorithm_example_move.py
```

内容可以参考：

```python
from pathlib import Path
import sys
import time
import numpy as np
import mujoco.viewer

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from api.arm_platform_api import ArmPlatformAPI

api = ArmPlatformAPI(PROJECT_ROOT / "models" / "robot_with_gripper.xml")

with mujoco.viewer.launch_passive(api.model, api.data) as viewer:
    state = api.get_state()
    print("Current q_arm:", state.q_arm)
    print("Current ee_pos:", state.ee_pos)

    target_pos = state.ee_pos + np.array([0.02, 0.00, 0.02])
    ik = api.ik_position(target_pos, q_init=state.q_arm)

    if not ik.success:
        print("IK failed. error =", ik.error_norm)
    else:
        print("IK success. q_goal =", ik.q)
        api.plan_joint_trajectory(ik.q, duration=3.0, method="quintic")

        while viewer.is_running():
            api.step()
            viewer.sync()
            time.sleep(api.model.opt.timestep)
```

运行：

```bash
python scripts/run_algorithm_example_move.py
```

---

## 5. 夹爪开合怎么用？

示例：

```python
api.open_gripper()
api.close_gripper()
api.set_gripper(0.02)
```

算法组不需要管 `joint5` / `joint6` 的底层 mimic 关系。

---

## 6. Joint4 导纳力控怎么用？

直接运行：

```bash
python scripts/run_joint4_admittance_force_control_demo.py
```

目标力 5 N：

```bash
python scripts/run_joint4_admittance_force_control_demo.py --target-force 5
```

目标力 8 N：

```bash
python scripts/run_joint4_admittance_force_control_demo.py --target-force 8
```

如果力达不到目标，可增大 joint4 预留空间：

```bash
python scripts/run_joint4_admittance_force_control_demo.py --force-margin 0.06
```

如果振荡，可降低导纳增益：

```bash
python scripts/run_joint4_admittance_force_control_demo.py --k-force 0.002
```

导纳力控的用途：

```text
夹住轴之后，沿 joint4 方向接触目标面，并把接触力调到目标值。
```

它的控制结构是：

```text
F_meas -> F_des - F_meas -> q4_cmd -> position actuator
```

输出日志：

```text
logs/admittance_force_control_log.csv
logs/admittance_force_control_report.md
```

---

## 7. Stage 3 v2 力矩级力控怎么用？

直接运行：

```bash
python scripts/run_torque_force_control_demo.py
```

目标力 5 N：

```bash
python scripts/run_torque_force_control_demo.py --target-force 5
```

如果希望目标力为 3 N 或 8 N：

```bash
python scripts/run_torque_force_control_demo.py --target-force 3
python scripts/run_torque_force_control_demo.py --target-force 8
```

如果力上不去：

```bash
python scripts/run_torque_force_control_demo.py --force-integral-gain 4 --max-task-force 80
```

如果振荡：

```bash
python scripts/run_torque_force_control_demo.py --force-gain 0.5 --force-integral-gain 1.5 --kd-scale 1.5
```

Stage 3 v2 的控制律是：

```text
F_task = Kf * e_F + Ki * integral(e_F dt)

tau = qfrc_bias + Kp(q_ref - q) - Kd*dq + J^T F_task
```

其中：

```text
qfrc_bias：MuJoCo 计算的重力、科氏、离心偏置项
J：ee_site 的平移雅可比
F_task：由接触力误差生成的笛卡尔任务力
tau：joint1~joint4 的 motor actuator 控制输入
```

输出日志：

```text
logs/torque_force_control_log.csv
logs/torque_force_control_report.md
```

---

## 8. 三种控制模式怎么选？

```text
只做路径规划 / IK / 抓取流程：
使用 ArmPlatformAPI。

要稳定展示接触力反馈：
使用 joint4 导纳力控。

要展示更接近真实电机力矩控制：
使用 Stage 3 v2 力矩级力控。

要做轴孔对接算法：
先用 ArmPlatformAPI 做接近；
接触后可以选择导纳力控或 Stage 3 v2；
再加入 spiral search / wiggling / screwing。
```

---

## 9. 当前力控版本的边界

当前版本足够算法组开发和接入验证，但需要注意：

```text
1. 夹爪夹住轴后仍使用 logical grasp lock；
2. 这不是夹爪纯摩擦夹持验证；
3. Stage 3 v2 是 MuJoCo motor actuator 仿真，不是实机电机驱动；
4. peg-in-hole 完整状态机还需要算法组继续开发；
5. 如果要研究夹爪摩擦夹持，需要后续取消 logical grasp lock 并重新调接触参数。
```

当前推荐表述：

```text
本平台已经提供位置控制、导纳力控和力矩级力控三类控制能力，可以支持算法组进行轨迹规划、抓取流程、接触力调节和轴孔对接策略开发。当前阶段重点验证机械臂本体控制与接触力控制，夹爪夹持采用任务层 logical grasp lock 表示稳定抓取。
```

---

## 10. 算法组交付内容建议

算法组后续可以交付：

```text
1. 轨迹规划脚本：
   scripts/run_algorithm_trajectory_demo.py

2. 轴孔对接状态机：
   src/task/peg_in_hole/state_machine.py

3. 螺旋搜索策略：
   src/planning/spiral_search.py

4. 插入策略：
   src/planning/screw_motion.py

5. 统一演示脚本：
   scripts/run_peg_in_hole_demo.py
```

每个算法脚本都应输出：

```text
logs/*.csv
logs/*.md
```

便于后续写报告和画图。

---

## 11. 交付前自检命令

算法组开始开发前，先运行：

```bash
python scripts/run_viewer.py
python scripts/check_gripper_mimic.py
python scripts/acceptance_kinematics.py
python scripts/acceptance_grasp_axis.py
python scripts/run_joint4_admittance_force_control_demo.py
python scripts/run_torque_force_control_demo.py
```

这些都能运行后，再开始写自己的算法。
