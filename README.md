# MuJoCo 机械臂-夹爪仿真平台

## 1. 项目定位

本项目是一个面向多人协作开发的 MuJoCo 机械臂仿真平台。当前版本包含：

- 机械臂真实 STL 外观模型
- 可开合双指夹爪
- mimic 夹爪关节处理
- 关节空间控制接口
- 正运动学接口
- 数值逆运动学接口
- 轨迹规划接口
- 算法组可直接调用的统一 API
- 简单抓取环境
- 未来动力学/力矩控制预留位置
- 轴孔对接任务状态机骨架

当前阶段的目标不是完成最终 peg-in-hole 对接任务，而是作为仿真平台负责人，提供一个稳定、可复用、可协作的底座，使算法组可以在统一接口上继续开发轨迹规划、抓取策略、螺旋搜索和插孔算法。

---

## 2. 当前工程状态

已经完成：

```text
1. 带夹爪机械臂模型导入 MuJoCo
2. 修正夹爪拓扑为双指同步开合
3. joint5 / joint6 mimic 关系建立
4. gripper_opening 单输入夹爪控制接口
5. 机械臂关节空间控制接口
6. 正运动学 FK
7. 位置逆运动学 IK
8. 关节空间轨迹规划
9. 算法调用统一 API
10. 简单抓取环境 simple_grasp_scene.xml
11. 未来 torque / dynamics control 模块占位
12. 详细协作目录结构
```

尚未完成或需要后续算法组继续做：

```text
1. 完整 peg-in-hole 自动对接任务
2. 孔、轴、工装夹具的最终模型
3. 精确 peg_tip_site / hole_entry_site / hole_axis_site
4. 接触状态检测和状态机自动切换
5. pushing / rubbing / wiggling / screwing 完整策略
6. 真实力矩控制与电机参数辨识
7. 完整 GUI 集成新版夹爪和抓取环境
```

---

## 3. 目录结构

```text
mujoco_arm_gripper_architecture_project/
├─ assets/
│  ├─ meshes/
│  │  ├─ base_link.STL
│  │  ├─ link1.STL
│  │  ├─ link2.STL
│  │  ├─ link3.STL
│  │  ├─ link4.STL
│  │  ├─ fin1.STL
│  │  └─ fin2.STL
│  └─ urdf/
│     ├─ robotic_arm_v2_original.urdf
│     └─ robotic_arm_v2_fixed_paths_and_limits.urdf
│
├─ models/
│  ├─ robot_with_gripper.xml
│  ├─ robot_with_gripper_parallel.xml
│  ├─ robot_with_gripper_serial_backup.xml
│  ├─ robot_with_gripper_simple_collision.xml
│  └─ simple_grasp_scene.xml
│
├─ configs/
│  ├─ robot_description.yaml
│  ├─ controller_config.yaml
│  ├─ gripper_config.yaml
│  ├─ ik_config.yaml
│  ├─ trajectory_config.yaml
│  ├─ peg_in_hole_task.yaml
│  └─ grasp_env.yaml
│
├─ src/
│  ├─ robot_model/
│  ├─ control/
│  ├─ gripper/
│  ├─ kinematics/
│  ├─ planning/
│  ├─ ik/
│  ├─ api/
│  ├─ environment/
│  ├─ dynamics_control/
│  ├─ task/peg_in_hole/
│  ├─ simulation/
│  └─ gui/
│
├─ scripts/
│  ├─ run_viewer.py
│  ├─ run_gripper_demo.py
│  ├─ check_gripper_mimic.py
│  ├─ run_fk_ik_demo.py
│  ├─ run_algorithm_api_demo.py
│  ├─ run_simple_grasp_demo.py
│  └─ run_torque_placeholder_demo.py
│
├─ docs/
├─ logs/
├─ experiments/
├─ requirements.txt
└─ README.md
```

---

## 4. 环境安装

推荐使用 Python 3.9 到 Python 3.11。

安装依赖：

```bash
pip install -r requirements.txt
```

如果没有使用 requirements 文件，也可以手动安装：

```bash
pip install mujoco numpy PyYAML
```

---

## 5. 快速运行

### 5.1 打开模型

```bash
python scripts/run_viewer.py
```

主模型为：

```text
models/robot_with_gripper.xml
```

---

### 5.2 检查夹爪 mimic

```bash
python scripts/check_gripper_mimic.py
```

预期结果：

```text
joint5 数值变化
joint6 数值反向变化
joint5 + joint6 接近 0
两个夹爪手指同步开合
```

---

### 5.3 运行夹爪开合 Demo

```bash
python scripts/run_gripper_demo.py
```

该脚本会周期性打开和闭合夹爪。

---

### 5.4 运行正逆运动学 Demo

```bash
python scripts/run_fk_ik_demo.py
```

该脚本会：

1. 读取当前关节角
2. 计算末端 `ee_site` 的正运动学
3. 设置一个附近目标点
4. 求解位置 IK
5. 用五次轨迹运动到目标位姿

---

### 5.5 运行算法接口 Demo

```bash
python scripts/run_algorithm_api_demo.py
```

该脚本演示算法组应如何调用统一 API，而不是直接操作 MuJoCo 内部变量。

---

### 5.6 运行简单抓取 Demo

```bash
python scripts/run_simple_grasp_demo.py
```

该脚本会打开：

```text
models/simple_grasp_scene.xml
```

该场景包含：

- 机械臂
- 可开合夹爪
- 一个简单圆柱物体
- 手指简化碰撞体

该 Demo 的目标是验证：

```text
1. 夹爪能开合
2. 手指碰撞体存在
3. 物体能与夹爪发生接触
4. 夹爪物理参数有基本可调位置
```

它不是最终的轴孔对接任务。

---

### 5.7 查看未来力矩控制占位

```bash
python scripts/run_torque_placeholder_demo.py
```

当前模型使用 position actuator。未来如果要实现真实电机扭矩控制，需要将 MJCF 中的 position actuator 替换或扩展为 motor actuator，然后把动力学控制模块输出的 torque 写入 `data.ctrl`。

---

## 6. 模型与关节说明

### 6.1 机械臂本体

机械臂控制变量：

```text
q_arm = [q1, q2, q3, d4]
```

对应：

```text
q1 -> joint1, revolute, rad
q2 -> joint2, revolute, rad
q3 -> joint3, revolute, rad
d4 -> joint4, prismatic, m
```

---

### 6.2 夹爪

夹爪控制变量：

```text
g = gripper_opening
```

对应：

```text
joint5 -> 夹爪主关节
joint6 -> mimic joint
```

mimic 关系：

```text
joint6 = -joint5
```

外部不要直接控制 joint6。所有夹爪控制都通过：

```text
gripper_opening
```

或代码接口：

```python
api.set_gripper(value)
api.open_gripper()
api.close_gripper()
```

---

## 7. 正运动学与逆运动学

### 7.1 正运动学

核心文件：

```text
src/kinematics/robot_kinematics.py
```

主要接口：

```python
fk = api.fk(q_arm)
pos = fk["position"]
rot = fk["rotation"]
```

默认末端：

```text
ee_site
```

如果后续需要以夹爪尖端或轴尖作为末端，可以将 `site_name` 改成：

```text
peg_tip_site
```

---

### 7.2 逆运动学

当前提供位置 IK：

```python
result = api.ik_position(target_pos)
q_goal = result.q
```

返回：

```text
q_goal
success
error_norm
iterations
```

注意：当前机械臂只有 4 个本体控制量，因此任意 6D 位姿目标通常是过约束。建议算法组优先使用位置 IK，即只给定 `x, y, z`。

---

### 7.3 给算法组的调用方式

算法组推荐只使用：

```python
from api.arm_platform_api import ArmPlatformAPI

api = ArmPlatformAPI("models/robot_with_gripper.xml")

state = api.get_state()
fk = api.fk(state.q_arm)
ik_result = api.ik_position([0.2, 0.1, 0.5])

api.plan_joint_trajectory(ik_result.q, duration=4.0, method="quintic")
api.step()
```

不要让算法组直接操作：

```text
data.qpos
data.ctrl
model.jnt_qposadr
model.actuator_ctrlrange
```

这些底层内容由平台层封装。

---

## 8. 算法接口说明

核心接口文件：

```text
src/api/arm_platform_api.py
```

主要类：

```text
ArmPlatformAPI
```

主要方法：

```python
api.get_state()
api.fk(q_arm=None, site_name="ee_site")
api.ik_position(target_pos, q_init=None, site_name="ee_site")
api.set_arm_target(q_arm)
api.set_gripper(opening)
api.open_gripper()
api.close_gripper()
api.plan_joint_trajectory(q_goal, duration=4.0, method="quintic")
api.step(n=1)
api.reset()
```

### 8.1 为什么需要 API 层

API 层的作用是隔离底层 MuJoCo 模型和上层算法。

这样未来即使：

```text
1. 换夹爪
2. 换模型
3. 改 joint 名字
4. 改 actuator
5. 改 IK 方法
6. 改控制器
```

算法组仍然尽量只调用同一套 API。

---

## 9. 简单抓取环境

### 9.1 场景文件

```text
models/simple_grasp_scene.xml
```

该文件在主模型基础上增加：

```text
1. grasp_object：一个轻量圆柱物体
2. fin1_finger_collision：左手指简化碰撞体
3. fin2_finger_collision：右手指简化碰撞体
```

### 9.2 设计目的

这个环境用于验证：

```text
1. 夹爪开合方向
2. 夹爪开口范围
3. 简化 collision 是否生效
4. 物体接触是否稳定
5. friction / mass / damping 参数是否需要调整
```

它不是最终 peg-in-hole 环境。

### 9.3 后续需要调整的物理参数

建议重点调：

```text
grasp_object mass
grasp_object friction
finger_collision size
finger_collision friction
joint damping
actuator kp / kv
actuator force range
timestep
solver 参数
```

---

## 10. 未来动力学与扭矩控制

当前模型主要使用：

```text
position actuator
```

即：

```text
data.ctrl = q_des
```

这适合快速搭建仿真平台、调试 IK、轨迹规划和 GUI。

未来如果要建立动力学参数、控制电机扭矩，应在以下位置开发：

```text
src/dynamics_control/torque_controller.py
```

当前已预留：

```text
ComputedTorqueController
gravity_compensation()
inverse_dynamics()
```

但注意：当前 MJCF 仍然是 position actuator，所以这些 torque 暂时不会直接驱动机械臂。未来需要：

```text
1. 新建 torque actuator 版本 MJCF
2. 将 position actuator 替换为 motor actuator
3. 将 tau 写入 data.ctrl
4. 增加电机力矩上限
5. 标定电机惯量、摩擦、阻尼、减速比
6. 验证重力补偿和轨迹跟踪
```

建议未来新增模型：

```text
models/robot_with_gripper_torque.xml
```

不要直接破坏当前稳定的 position actuator 模型。

---

## 11. 轴孔对接任务预留

当前状态机骨架位于：

```text
src/task/peg_in_hole/
```

主要文件：

```text
state_machine.py
```

预留状态：

```text
IDLE
APPROACH_HOLE
GRASP_PEG
MOVE_TO_PRE_INSERT
REACHING_PUSH
SEARCHING_SPIRAL
INSERTING_WIGGLE_SCREW
COMPLETE
FAILED
```

对应论文中的基本动作：

```text
pushing
rubbing / spiral searching
wiggling
screwing
```

后续算法组应在该模块内实现具体任务流程，不要把 peg-in-hole 逻辑写进 GUI 或底层控制器。

---

## 12. 协作开发边界

### 12.1 仿真环境负责人

负责：

```text
models/
assets/
configs/robot_description.yaml
configs/grasp_env.yaml
src/environment/
基础 demo
README
```

确保模型、物理参数、碰撞体、site、环境对象可靠。

### 12.2 运动学负责人

负责：

```text
src/kinematics/
src/ik/
configs/ik_config.yaml
```

确保 FK / IK 结果稳定，供算法组调用。

### 12.3 轨迹规划负责人

负责：

```text
src/planning/
configs/trajectory_config.yaml
```

实现关节空间轨迹、笛卡尔轨迹、螺旋搜索、插入轨迹等。

### 12.4 控制负责人

负责：

```text
src/control/
src/dynamics_control/
configs/controller_config.yaml
```

当前阶段维护 position control。未来拓展 torque control。

### 12.5 夹爪负责人

负责：

```text
src/gripper/
configs/gripper_config.yaml
```

维护夹爪开合接口和抓取状态判断。

### 12.6 GUI 负责人

负责：

```text
src/gui/
scripts/run_realtime_gui.py
```

GUI 只调用 API，不直接写核心算法。

---

## 13. 提交前验收命令

每次修改模型或接口后，建议至少运行：

```bash
python scripts/run_viewer.py
python scripts/check_gripper_mimic.py
python scripts/run_gripper_demo.py
python scripts/run_fk_ik_demo.py
python scripts/run_algorithm_api_demo.py
python scripts/run_simple_grasp_demo.py
```

如果涉及未来动力学控制，再运行：

```bash
python scripts/run_torque_placeholder_demo.py
```

---

## 14. 常见问题

### 14.1 为什么只控制 joint5，不控制 joint6？

因为 joint6 是 mimic joint：

```text
joint6 = -joint5
```

如果同时给 joint5 和 joint6 单独 actuator，两个控制器可能互相打架。正确方式是外部只控制 `gripper_opening`。

---

### 14.2 为什么 IK 不包含夹爪？

夹爪开合一般不决定机械臂末端位置。IK 只求机械臂本体：

```text
q_arm = [q1, q2, q3, d4]
```

夹爪开合由 gripper 模块单独控制。

---

### 14.3 为什么 simple_grasp_scene 不是最终环境？

因为它只是用于验证夹爪物理接触。最终轴孔对接还需要：

```text
hole model
peg model
peg_tip_site
hole_entry_site
hole_axis_site
contact detection
spiral search
insertion state machine
```

---

### 14.4 什么时候需要管动力学？

如果只是验证 FK、IK、轨迹规划，可以先使用 position actuator。

如果需要研究真实电机力矩、接触力、阻抗控制、插入过程稳定性，就需要进入动力学控制阶段。

---

## 15. 当前推荐主入口

普通看模型：

```bash
python scripts/run_viewer.py
```

检查夹爪：

```bash
python scripts/check_gripper_mimic.py
```

给算法组测试接口：

```bash
python scripts/run_algorithm_api_demo.py
```

测试正逆运动学：

```bash
python scripts/run_fk_ik_demo.py
```

测试简单抓取：

```bash
python scripts/run_simple_grasp_demo.py
```


---

## 导纳力控版本 v2

当前工程已经新增 joint4 一维导纳力控 demo：

```bash
python scripts/run_joint4_admittance_force_control_demo.py
```

该 demo 使用：

```text
导纳外环 + 位置执行器内环
```

控制逻辑为：

```text
F_meas = MuJoCo 接触力
F_error = F_des - F_meas
q4_cmd 根据 F_error 实时修正
```

这与普通位置轨迹的区别是：普通版本不读取接触力，而导纳力控版本会根据真实接触力主动下压或回退。

上一版导纳力控失败的主要原因是：force-control 开始时 `joint4` 已经到达机械上限 `q4 = 0.2`，控制器虽然想继续下压，但 `q4_cmd` 被限幅，导致接触力一直停在约 `0.949 N`。当前 v2 版本通过 `--force-margin` 预留 joint4 向下控制空间。

常用命令：

```bash
python scripts/run_joint4_admittance_force_control_demo.py --target-force 5
python scripts/run_joint4_admittance_force_control_demo.py --target-force 8 --k-force 0.002
python scripts/run_joint4_admittance_force_control_demo.py --force-margin 0.06
python scripts/run_joint4_admittance_force_control_demo.py --initial-penetration 0.003
```

输出：

```text
logs/admittance_force_control_log.csv
logs/admittance_force_control_report.md
```


---

## 第三阶段：力矩级力控版本

当前工程新增第三阶段力控 demo：

```bash
python scripts/run_torque_force_control_demo.py
```

该 demo 会自动生成临时模型：

```text
models/_torque_force_control_scene.xml
```

在该临时模型中：

```text
joint1, joint2, joint3, joint4: motor actuator
gripper_opening: position actuator
```

控制律为：

```text
tau = qfrc_bias + Kp(q_ref - q) - Kd*dq + J^T F_task
```

其中：

```text
qfrc_bias: MuJoCo 计算的重力、科氏、离心偏置项
J: ee_site 的平移雅可比
F_task: 由接触力误差生成的笛卡尔任务力
```

这和导纳力控版本的区别是：

```text
导纳力控：F_error -> q4_cmd
力矩级力控：F_error -> F_task -> tau
```

常用命令：

```bash
python scripts/run_torque_force_control_demo.py --target-force 5
python scripts/run_torque_force_control_demo.py --force-gain 0.8
python scripts/run_torque_force_control_demo.py --kp-scale 0.7 --kd-scale 1.2
```

如果振荡：

```bash
python scripts/run_torque_force_control_demo.py --force-gain 0.5 --kp-scale 0.7 --kd-scale 1.5
```

输出：

```text
logs/torque_force_control_log.csv
logs/torque_force_control_report.md
```

注意：该版本仍然使用 logical grasp lock 表示轴已经被夹住，因此重点是验证机械臂本体的 motor actuator + torque-level contact force control，而不是验证夹爪摩擦夹持。


---

## Stage 3 v2 力矩级力控修正

上一版 Stage 3 结果显示，目标力为 5 N，但末段平均滤波力只有约 1.799 N，说明纯比例式 `J^T F_task` 力控制项不能消除稳态误差。

v2 已将力控制项改成：

```text
F_task = Kf * e_F + Ki * integral(e_F dt)
```

完整控制律为：

```text
tau = qfrc_bias + Kp(q_ref-q) - Kd*dq + J^T F_task
```

运行：

```bash
python scripts/run_torque_force_control_demo.py --target-force 5
```

若力仍不足：

```bash
python scripts/run_torque_force_control_demo.py --force-integral-gain 4 --max-task-force 80
```

若振荡：

```bash
python scripts/run_torque_force_control_demo.py --force-gain 0.5 --force-integral-gain 1.5 --kd-scale 1.5
```
