# Stage 3 力矩级力控版本说明

## 1. 这个版本是什么

本版本是第三阶段控制版本：

```text
motor actuator + torque-level hybrid force control
```

上一阶段导纳控制是：

```text
F_error -> q4_cmd
```

第三阶段力矩级控制是：

```text
F_error -> F_task
tau = qfrc_bias + Kp(q_ref-q) - Kd*dq + J^T F_task
```

也就是说，机械臂本体 `joint1` 到 `joint4` 不再用 position actuator，而是在脚本生成的临时场景中替换为 motor actuator。控制器直接输出广义力矩/力：

```text
joint1, joint2, joint3: torque, unit N*m
joint4: slide joint force, unit N
```

夹爪 `gripper_opening` 仍然保留 position actuator，因为当前任务不是研究夹爪电机力矩，而是研究机械臂本体接触力控制。

## 2. 运行命令

```bash
python scripts/run_torque_force_control_demo.py
```

常用调参：

```bash
python scripts/run_torque_force_control_demo.py --target-force 5
python scripts/run_torque_force_control_demo.py --force-gain 0.8
python scripts/run_torque_force_control_demo.py --kp-scale 0.7 --kd-scale 1.2
python scripts/run_torque_force_control_demo.py --max-task-force 20
```

如果出现明显振荡，优先使用：

```bash
python scripts/run_torque_force_control_demo.py --force-gain 0.5 --kp-scale 0.7 --kd-scale 1.5
```

如果力上不去，可以增大：

```bash
python scripts/run_torque_force_control_demo.py --force-gain 1.5 --max-task-force 50
```

## 3. 输出文件

```text
logs/torque_force_control_log.csv
logs/torque_force_control_report.md
models/_torque_force_control_scene.xml
```

CSV 会记录：

```text
q, dq
contact force
filtered force
force error
F_task
tau
tau_bias
tau_pd
tau_force
saturation
shaft position
```

## 4. 与导纳力控的区别

导纳力控版本：

```text
使用 position actuator
控制器输出 q4_cmd
本质是力反馈修正位置目标
```

力矩级版本：

```text
使用 motor actuator
控制器输出 tau
使用 MuJoCo qfrc_bias 做重力/偏置补偿
使用 MuJoCo Jacobian 做 J^T F 力映射
```

因此第三阶段更接近真实机器人底层电机力矩控制。

## 5. 当前限制

当前版本仍然是仿真控制器，不是实机控制器。它仍然使用：

```text
logical grasp lock
```

来表示轴已经被夹爪夹住。也就是说，轴与夹爪之间的附着关系仍然是任务层约束，而不是完全依赖夹爪摩擦。

这不影响第三阶段的核心目标：验证机械臂本体从 position actuator 升级到 motor actuator，并通过力矩控制实现接触力调节。


## v2 修正说明

上一版力矩级控制失败的主要原因是：力控制项只有比例项：

```text
F_task = Kf * (F_des - F_meas)
```

在 motor actuator + 关节姿态 PD 的结构中，比例力项会和关节 PD、接触刚度形成静态平衡，因此可能稳定在目标力以下。例如你上传的结果中，目标力为 5 N，但末段平均滤波力只有 1.799 N，误差为 3.201 N。

v2 加入积分力项：

```text
F_task = Kf * e_F + Ki * integral(e_F dt)
```

这样可以消除稳态力误差。

推荐先运行：

```bash
python scripts/run_torque_force_control_demo.py --target-force 5
```

如果仍然达不到目标力，逐步增加：

```bash
python scripts/run_torque_force_control_demo.py --force-integral-gain 4 --max-task-force 80
```

如果出现振荡，降低比例项并增加阻尼：

```bash
python scripts/run_torque_force_control_demo.py --force-gain 0.5 --force-integral-gain 1.5 --kd-scale 1.5
```
