# Joint4 导纳力控版本说明

## 1. 当前版本

当前版本为 `run_joint4_admittance_force_control_demo.py` v2。

它实现：

```text
导纳外环 + 位置执行器内环
```

控制器读取 MuJoCo 中竖直轴与力控平面之间的接触力，并根据力误差实时修正 `joint4` 的位置目标。

## 2. 与普通位置控制的区别

普通抓取/接触演示：

```text
预设 q4 轨迹 -> position actuator -> 运动
```

导纳力控：

```text
MuJoCo 接触力 F_meas
        ↓
F_des - F_meas
        ↓
导纳控制器
        ↓
q4_cmd 实时修正
        ↓
position actuator
        ↓
新的接触力
```

因此它已经形成接触力反馈闭环。

## 3. 上一版失败的原因

上一版日志中：

```text
q4_cmd = +0.20000
F = 0.949 N
err = 4.051 N
```

说明 joint4 已经在上限 `0.2`，控制器想继续下压，但命令被机械限位截断，因此接触力无法继续增加。

v2 修正方法：

```text
接触开始时不再把 q4 放到极限；
默认预留 force_margin = 0.045 m；
导纳控制阶段仍然可以继续向下压。
```

## 4. 运行命令

默认目标力 5 N：

```bash
python scripts/run_joint4_admittance_force_control_demo.py
```

目标力 8 N：

```bash
python scripts/run_joint4_admittance_force_control_demo.py --target-force 8
```

如果响应太快或振荡：

```bash
python scripts/run_joint4_admittance_force_control_demo.py --k-force 0.002
```

如果力仍然达不到目标，增大预留行程：

```bash
python scripts/run_joint4_admittance_force_control_demo.py --force-margin 0.06
```

如果初始接触太弱：

```bash
python scripts/run_joint4_admittance_force_control_demo.py --initial-penetration 0.003
```

如果纯物理测试，不使用 logical grasp lock：

```bash
python scripts/run_joint4_admittance_force_control_demo.py --no-grasp-lock
```

## 5. 输出

```text
logs/admittance_force_control_log.csv
logs/admittance_force_control_report.md
```

CSV 中包含：

```text
time
q4
q4_cmd
force_normal
force_filtered
force_error
velocity_command
saturated_low
saturated_high
contact_count
shaft position
```

## 6. 报告建议表述

考虑到当前 MuJoCo 仿真平台采用位置执行器作为底层伺服接口，本项目首先实现 joint4 方向的一维导纳力控。控制器实时读取被夹持竖直轴与力控平面之间的法向接触力，并将目标力与测量力之间的误差转换为 joint4 的位置修正量。因此，机械臂不再只是按照预设轨迹运动，而是能够根据物理接触反馈主动下压或回退，从而实现接触力调节。
