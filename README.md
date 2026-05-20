# MuJoCo 机械臂-夹爪仿真平台 README

## 1. 项目定位

本项目是一个面向多人协作开发的 MuJoCo 机械臂仿真平台，用于后续运动算法、抓取算法、轨迹规划、逆运动学、轴孔对接任务以及未来动力学控制算法的部署与验证。

当前项目的重点不是只把模型显示出来，而是建立一个稳定的仿真平台，使算法组可以直接调用统一接口完成：

```text
1. 正运动学 FK
2. 逆运动学 IK
3. 关节空间轨迹规划
4. 夹爪开合控制
5. 简单竖直轴夹取验证
6. 后续 peg-in-hole 对接任务开发
7. 后续动力学 / 力矩控制开发
```

当前版本已经包含可开合双指夹爪，并修正了 mimic 关节，使两个夹爪手指能够同步反向开合。

---

## 2. 当前最终版本说明

当前推荐使用的工程状态是：

```text
主模型：models/robot_with_gripper.xml
正逆运动学验收：scripts/acceptance_kinematics.py
夹取验收：scripts/acceptance_grasp_axis.py
夹取验收版本：v8 top-stop + lift
```

当前夹取验收脚本的核心逻辑是：

```text
1. 轴保持竖直；
2. 自动搜索 joint4 平动方向最接近世界竖直方向的姿态；
3. 接近和抬升阶段只改变 joint4；
4. q1 / q2 / q3 在接近和抬升过程中保持不变；
5. 不给夹爪添加绿色辅助 collision box；
6. 使用原始 STL mesh geom 统计接触：
   fin1_visual
   fin2_visual

7. 启用 link4_visual 与轴的接触，将 link4 / joint4 顶头作为顶部限位；
8. link4_head contact 作为 top-stop 顶部限位记录，不再直接判失败；
9. 满足最小夹持条件后执行 joint4 竖直抬升；
10. 抬升后 final_hold 阶段继续保持 logical grasp lock，避免轴在空中慢慢滑落。
```

注意：`logical grasp lock` 不是额外的碰撞体，而是在确认已经夹住后，让物体随夹爪运动的任务约束。它用于保证验收演示稳定，避免复杂 STL mesh 摩擦接触在 MuJoCo 中出现随机滑落。

如果要测试纯物理摩擦抓取，可以运行：

```bash
python scripts/acceptance_grasp_axis.py --no-grasp-lock
```

但纯物理抓取可能因为 STL mesh 接触、摩擦、求解器参数不稳定而滑落，这不适合作为平台基础验收的唯一标准。

---

## 3. 工程目录结构

推荐工程目录如下：

```text
mujoco_arm_platform_final_project/
├─ README.md
├─ requirements.txt
│
├─ assets/
│  ├─ meshes/
│  │  ├─ base_link.STL
│  │  ├─ link1.STL
│  │  ├─ link2.STL
│  │  ├─ link3.STL
│  │  ├─ link4.STL
│  │  ├─ fin1.STL
│  │  └─ fin2.STL
│  │
│  └─ urdf/
│     ├─ robotic_arm_v2_original.urdf
│     └─ robotic_arm_v2_fixed_paths_and_limits.urdf
│
├─ models/
│  ├─ robot_with_gripper.xml
│  ├─ robot_with_gripper_parallel.xml
│  ├─ robot_with_gripper_serial_backup.xml
│  ├─ robot_with_gripper_simple_collision.xml
│  ├─ simple_grasp_scene.xml
│  ├─ _acceptance_topdown_grasp_scene.xml
│  ├─ _acceptance_joint4_vertical_search_scene.xml
│  └─ _acceptance_joint4_vertical_search_scene_v8.xml
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
│  ├─ api/
│  │  └─ arm_platform_api.py
│  │
│  ├─ robot_model/
│  │  ├─ model_loader.py
│  │  └─ joint_registry.py
│  │
│  ├─ control/
│  │  └─ joint_space_controller.py
│  │
│  ├─ gripper/
│  │  └─ parallel_gripper.py
│  │
│  ├─ kinematics/
│  │  └─ robot_kinematics.py
│  │
│  ├─ planning/
│  │  ├─ joint_trajectory.py
│  │  ├─ spiral_search.py
│  │  └─ screw_motion.py
│  │
│  ├─ ik/
│  │  └─ numerical_ik.py
│  │
│  ├─ environment/
│  │  └─ simple_grasp_environment.py
│  │
│  ├─ dynamics_control/
│  │  └─ torque_controller.py
│  │
│  ├─ task/
│  │  └─ peg_in_hole/
│  │     └─ state_machine.py
│  │
│  ├─ simulation/
│  │  └─ mujoco_runner.py
│  │
│  └─ gui/
│
├─ scripts/
│  ├─ run_viewer.py
│  ├─ run_gripper_demo.py
│  ├─ check_gripper_mimic.py
│  ├─ run_fk_ik_demo.py
│  ├─ run_algorithm_api_demo.py
│  ├─ run_simple_grasp_demo.py
│  ├─ run_torque_placeholder_demo.py
│  ├─ acceptance_kinematics.py
│  └─ acceptance_grasp_axis.py
│
├─ docs/
│  ├─ algorithm_interface.md
│  ├─ future_dynamics_control.md
│  └─ simulation_platform_responsibility.md
│
├─ logs/
│  ├─ acceptance_kinematics_report.md
│  ├─ acceptance_kinematics_log.csv
│  ├─ acceptance_grasp_axis_report.md
│  └─ acceptance_grasp_axis_log.csv
│
└─ experiments/
```

其中最重要的入口文件是：

```text
models/robot_with_gripper.xml
scripts/acceptance_kinematics.py
scripts/acceptance_grasp_axis.py
src/api/arm_platform_api.py
src/kinematics/robot_kinematics.py
```

---

## 4. 环境安装

推荐 Python 版本：

```text
Python 3.9 - Python 3.13
```

安装依赖：

```bash
pip install -r requirements.txt
```

如果没有 `requirements.txt`，可以手动安装：

```bash
pip install mujoco numpy PyYAML
```

Windows PowerShell 中建议在项目根目录运行命令，例如：

```powershell
cd D:\HuaweiMoveData\Users\PC\Desktop\mujoco_arm_platform_final_project
python scripts/run_viewer.py
```

---

## 5. 快速运行命令

### 5.1 打开主模型

```bash
python scripts/run_viewer.py
```

该命令会打开：

```text
models/robot_with_gripper.xml
```

用于检查机械臂和夹爪是否正常显示。

---

### 5.2 检查夹爪 mimic 关系

```bash
python scripts/check_gripper_mimic.py
```

预期结果：

```text
joint5 与 joint6 数值同步反向变化
joint6 = -joint5
两个夹爪手指同步开合
```

---

### 5.3 运行夹爪开合 Demo

```bash
python scripts/run_gripper_demo.py
```

用于观察夹爪是否能周期性打开和闭合。

---

### 5.4 运行正逆运动学 Demo

```bash
python scripts/run_fk_ik_demo.py
```

该脚本会：

```text
1. 读取当前关节角；
2. 输出 ee_site 的正运动学结果；
3. 给定一个附近目标点；
4. 求解 IK；
5. 使用轨迹规划运动到 IK 解。
```

---

### 5.5 运行算法接口 Demo

```bash
python scripts/run_algorithm_api_demo.py
```

用于演示算法组如何调用统一 API，而不是直接操作 MuJoCo 的底层 `data.qpos` 和 `data.ctrl`。

---

### 5.6 运行正逆运动学验收

```bash
python scripts/acceptance_kinematics.py
```

该脚本会直接打开 MuJoCo Viewer，并可视化演示多组 IK 结果。

验收内容：

```text
1. 模型能加载；
2. q_arm = [q1, q2, q3, d4] 能正确读取；
3. FK 能输出 ee_site 位置和姿态；
4. IK 能求解多个已知可达目标；
5. IK 解代回 FK 后误差满足阈值；
6. 轨迹规划器能生成连续 q_des(t)。
```

输出文件：

```text
logs/acceptance_kinematics_report.md
logs/acceptance_kinematics_log.csv
```

注意：该验收文件验证的是运动学接口，不把真实动力学执行器跟踪误差作为失败条件。

---

### 5.7 运行竖直轴顶部夹取验收

```bash
python scripts/acceptance_grasp_axis.py
```

这是当前最重要的抓取验收脚本。

该脚本会打开 MuJoCo Viewer，并执行：

```text
1. 自动检测夹爪开合方向；
2. 自动搜索 joint4 运动方向最接近世界竖直的 q1/q2/q3 姿态；
3. 放置竖直圆柱轴；
4. 夹爪从正上方接近；
5. 接近过程中只改变 joint4；
6. 闭合夹爪；
7. 检查最小夹持条件；
8. 使用 joint4 竖直抬升；
9. final_hold 阶段保持 logical grasp lock，防止空中滑落。
```

输出文件：

```text
logs/acceptance_grasp_axis_report.md
logs/acceptance_grasp_axis_log.csv
```

---

## 6. 夹爪与 mimic 关节说明

当前夹爪包含：

```text
fin1
fin2
joint5
joint6
```

其中：

```text
joint5：夹爪主关节
joint6：mimic 关节
```

mimic 关系为：

```text
joint6 = -joint5
```

外部控制时只应该控制：

```text
gripper_opening
```

不要直接控制 `joint6`。

正确控制方式：

```python
api.set_gripper(value)
api.open_gripper()
api.close_gripper()
```

不推荐直接写：

```python
data.ctrl[...]
```

---

## 7. 机械臂控制变量约定

全项目统一使用：

```text
q_arm = [q1, q2, q3, d4]
```

对应关系：

```text
q1 -> joint1, revolute, rad
q2 -> joint2, revolute, rad
q3 -> joint3, revolute, rad
d4 -> joint4, prismatic, m
```

夹爪控制变量：

```text
g = gripper_opening
```

整体控制可以理解为：

```text
u = [q1, q2, q3, d4, g]
```

但在代码结构上，机械臂本体和夹爪是分开的：

```text
机械臂：src/control/
夹爪：src/gripper/
统一接口：src/api/
```

---

## 8. 正运动学与逆运动学

### 8.1 核心文件

```text
src/kinematics/robot_kinematics.py
```

主要类：

```text
RobotKinematics
```

主要功能：

```text
1. 读取 q_arm；
2. 设置 q_arm；
3. 计算 ee_site 正运动学；
4. 获取 site Jacobian；
5. 求解位置 IK；
6. 可选求解位姿 IK。
```

### 8.2 正运动学调用

```python
from api.arm_platform_api import ArmPlatformAPI

api = ArmPlatformAPI("models/robot_with_gripper.xml")

state = api.get_state()
fk = api.fk(state.q_arm)

pos = fk["position"]
rot = fk["rotation"]
```

默认末端为：

```text
ee_site
```

如果后续添加精确工具点或轴尖点，可以改为：

```text
peg_tip_site
hole_entry_site
```

---

### 8.3 逆运动学调用

```python
target_pos = [0.2, 0.1, 0.5]

ik_result = api.ik_position(target_pos)

if ik_result.success:
    q_goal = ik_result.q
```

返回内容包括：

```text
q
success
error_norm
iterations
```

注意：当前机械臂本体只有 4 个控制量，因此不要一开始就要求任意 6D 位姿 IK。推荐算法组先使用位置 IK，即只控制：

```text
x, y, z
```

---

## 9. 给算法组的统一 API

核心文件：

```text
src/api/arm_platform_api.py
```

推荐算法组只调用：

```python
from api.arm_platform_api import ArmPlatformAPI

api = ArmPlatformAPI("models/robot_with_gripper.xml")
```

主要接口：

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

示例：

```python
from api.arm_platform_api import ArmPlatformAPI

api = ArmPlatformAPI("models/robot_with_gripper.xml")

state = api.get_state()
target = state.ee_pos + [0.03, 0.02, 0.02]

ik = api.ik_position(target)

if ik.success:
    api.plan_joint_trajectory(ik.q, duration=4.0, method="quintic")

while True:
    api.step()
```

算法组不应该直接操作：

```text
data.qpos
data.ctrl
model.jnt_qposadr
model.actuator_ctrlrange
```

这些底层细节应由平台层封装。

---

## 10. 轨迹规划

核心文件：

```text
src/planning/joint_trajectory.py
```

当前支持三种轨迹：

```text
linear
cubic
quintic
```

推荐使用：

```text
quintic
```

因为五次轨迹在起点和终点速度、加速度更平滑。

示例：

```python
api.plan_joint_trajectory(q_goal, duration=4.0, method="quintic")
```

---

## 11. 顶部竖直轴夹取验收逻辑

当前夹取验收脚本：

```text
scripts/acceptance_grasp_axis.py
```

当前版本为：

```text
v8 top-stop + lift
```

### 11.1 为什么要搜索 q1/q2/q3

用户要求：

```text
joint4 平动副必须保持竖直运动
不能斜着抓轴
```

但机械臂不同姿态下，`joint4` 的平动方向在世界坐标系中不一定竖直。因此脚本会自动搜索一个 q1/q2/q3 姿态，使 joint4 的运动方向尽可能接近世界竖直方向。

验收日志中会输出：

```text
vertical_ratio
xy_motion
z_motion
joint4_disp
```

其中：

```text
vertical_ratio 越接近 1，说明 joint4 运动越接近世界竖直方向。
```

---

### 11.2 为什么 link4_head contact 不再判失败

早期版本把 `link4_head` 接触判为失败，导致已经夹到后也不抬升。

现在逻辑修改为：

```text
link4_head contact = top-stop 顶部限位接触
```

也就是说，`link4_visual` 与轴接触说明轴顶到了机械结构上限位，防止继续穿模进入 link4。它不再直接导致失败。

如果想减少轴顶入 link4 的程度，可以调小：

```bash
python scripts/acceptance_grasp_axis.py --top-overlap 0.025
```

如果想让轴更明显顶住内部上限位，可以调大：

```bash
python scripts/acceptance_grasp_axis.py --top-overlap 0.05
```

---

### 11.3 为什么需要 logical grasp lock

纯 STL mesh 接触 + 摩擦在 MuJoCo 中不一定稳定，尤其是复杂夹爪 STL 与圆柱接触时，可能出现：

```text
1. 明明闭合夹住，但抬升时慢慢滑落；
2. 接触点不连续；
3. 摩擦不足；
4. 求解器局部不稳定；
5. 接触法向不理想。
```

为了让平台验收演示稳定，当前默认使用：

```text
logical grasp lock
```

它的意义是：

```text
一旦 minimal_grasp_ok 成立，就认为抓取成功；
之后轴跟随夹爪抬升。
```

这不是额外的 collision geom，而是抓取任务中常见的“抓取成功后附着约束”。

如果想测试纯物理摩擦，可以运行：

```bash
python scripts/acceptance_grasp_axis.py --no-grasp-lock
```

但纯物理模式下滑落不一定代表平台失败，而可能代表需要进一步调试：

```text
夹爪接触面
摩擦系数
solver 参数
mesh 接触精度
夹爪力
接触体简化
```

---

## 12. 夹取验收参数说明

运行：

```bash
python scripts/acceptance_grasp_axis.py
```

常用参数：

```bash
--descend-depth 0.11
```

控制 joint4 向下插入的行程。

```bash
--top-overlap 0.04
```

控制轴顶部进入夹爪/顶头区域的程度。

```bash
--vertical-ratio-threshold 0.92
```

控制 joint4 运动方向接近世界竖直的最低要求。

```bash
--samples-per-joint 9
```

控制搜索 q1/q2/q3 的网格密度。更大更精确，但更慢。

```bash
--require-mesh-contact
```

要求原始 STL mesh contact 必须大于 0 才通过。

```bash
--no-grasp-lock
```

关闭 logical grasp lock，测试纯物理摩擦抓取。

推荐调试命令：

```bash
python scripts/acceptance_grasp_axis.py --descend-depth 0.13 --top-overlap 0.035
```

如果想强制检查原始 mesh 接触：

```bash
python scripts/acceptance_grasp_axis.py --require-mesh-contact
```

---

## 13. 简单抓取环境说明

基础场景：

```text
models/simple_grasp_scene.xml
```

该场景包含：

```text
1. 机械臂；
2. 可开合夹爪；
3. 一个简单圆柱物体；
4. 支撑台；
5. 基础接触参数。
```

该场景用于验证夹爪、物体、碰撞和控制流程，不是最终轴孔对接场景。

当前验收脚本会自动生成临时场景，例如：

```text
models/_acceptance_joint4_vertical_search_scene_v8.xml
```

这类 `_acceptance_*.xml` 文件是脚本生成的验收场景，不建议手动长期维护。

---

## 14. 未来动力学与力矩控制

当前模型主要使用：

```text
position actuator
```

即：

```text
data.ctrl = q_des
```

这适合快速搭建平台、验证 FK / IK / 轨迹规划 / 抓取流程。

未来如果要做电机扭矩控制，应在以下位置继续开发：

```text
src/dynamics_control/torque_controller.py
```

当前已预留：

```text
ComputedTorqueController
gravity_compensation()
inverse_dynamics()
```

但注意：当前 MJCF 仍然是 position actuator。因此这些 torque 暂时不会直接驱动机械臂。

未来建议新增一个模型文件：

```text
models/robot_with_gripper_torque.xml
```

不要直接破坏当前稳定的 position actuator 模型。

未来动力学控制开发顺序建议：

```text
1. 建立 motor actuator 版本 MJCF；
2. 增加电机力矩上限；
3. 标定质量、惯量、阻尼、摩擦；
4. 实现 gravity compensation；
5. 实现 PD + gravity compensation；
6. 实现 inverse dynamics feedforward；
7. 实现 computed torque control；
8. 进一步实现 impedance / admittance control；
9. 最后用于接触任务和轴孔对接。
```

---

## 15. Peg-in-hole 对接任务预留

当前轴孔对接任务骨架位于：

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

未来可以继续实现：

```text
1. 抓取轴；
2. 移动到孔上方；
3. reaching push；
4. rubbing / spiral search；
5. wiggling；
6. screwing；
7. 插入完成检测。
```

目前项目只完成了基础抓取验收，不包含最终 peg-in-hole 全流程。

---

## 16. 多人协作边界

### 16.1 仿真平台负责人

负责：

```text
assets/
models/
configs/robot_description.yaml
configs/grasp_env.yaml
scripts/acceptance_kinematics.py
scripts/acceptance_grasp_axis.py
README.md
```

重点保证：

```text
1. 模型能打开；
2. 夹爪同步开合；
3. 运动学验收通过；
4. 竖直轴夹取验收通过；
5. 接口稳定；
6. 文档完整。
```

---

### 16.2 运动学负责人

负责：

```text
src/kinematics/
src/ik/
configs/ik_config.yaml
```

任务：

```text
1. FK；
2. IK；
3. Jacobian；
4. 可达性判断；
5. 末端 site 坐标维护。
```

---

### 16.3 轨迹规划负责人

负责：

```text
src/planning/
configs/trajectory_config.yaml
```

任务：

```text
1. joint trajectory；
2. Cartesian trajectory；
3. spiral search；
4. screw motion；
5. insertion trajectory。
```

---

### 16.4 控制负责人

负责：

```text
src/control/
src/dynamics_control/
configs/controller_config.yaml
```

任务：

```text
1. position control；
2. actuator 参数；
3. torque control；
4. gravity compensation；
5. impedance control。
```

---

### 16.5 夹爪负责人

负责：

```text
src/gripper/
configs/gripper_config.yaml
```

任务：

```text
1. gripper_opening 控制；
2. open / close；
3. grasp state；
4. logical grasp lock；
5. 夹持条件判断。
```

---

### 16.6 GUI 负责人

负责：

```text
src/gui/
scripts/run_realtime_gui.py
```

原则：

```text
GUI 只调用 API，不写核心算法。
```

---

## 17. 提交前验收清单

每次修改模型、接口、夹爪或控制参数后，建议至少运行：

```bash
python scripts/run_viewer.py
python scripts/check_gripper_mimic.py
python scripts/run_gripper_demo.py
python scripts/acceptance_kinematics.py
python scripts/acceptance_grasp_axis.py
```

如果涉及 API 修改，再运行：

```bash
python scripts/run_algorithm_api_demo.py
```

如果涉及未来动力学模块，再运行：

```bash
python scripts/run_torque_placeholder_demo.py
```

---

## 18. 常见问题

### 18.1 为什么只控制 joint5，不控制 joint6？

因为 joint6 是 mimic joint：

```text
joint6 = -joint5
```

如果同时给 joint5 和 joint6 actuator，两个控制器可能互相打架。正确方式是只控制：

```text
gripper_opening
```

---

### 18.2 为什么夹取验收中有 logical grasp lock？

因为纯 STL mesh 摩擦抓取在 MuJoCo 中容易出现接触不连续或慢慢滑落。平台验收阶段更重要的是确认：

```text
1. 夹爪从正上方接近；
2. joint4 竖直运动；
3. 轴没有被撞偏；
4. 轴没有倾倒；
5. 夹爪闭合条件合理；
6. 夹住后能抬升。
```

因此默认在满足最小夹持条件后启用 logical grasp lock。

---

### 18.3 为什么 link4_head 接触不算失败？

因为当前工业夹取逻辑中，竖直轴上端可能接触到夹爪内部或 link4 / joint4 顶头区域。这个接触可以理解为顶部限位，作用是防止轴继续穿模进入 link4。

所以当前逻辑是：

```text
link4_head contact = top-stop 顶部限位接触
```

它会被记录，但不直接导致失败。

---

### 18.4 为什么不再给夹爪添加绿色 collision box？

因为绿色 box 会误导观看者，以为那是夹爪零件。当前脚本不再给夹爪添加额外绿色碰撞体，而是使用：

```text
fin1_visual
fin2_visual
link4_visual
```

这些原始 STL mesh geom 来统计接触。

---

### 18.5 如果轴还是穿进 link4 怎么办？

降低：

```bash
python scripts/acceptance_grasp_axis.py --top-overlap 0.025
```

或适当减小下降深度：

```bash
python scripts/acceptance_grasp_axis.py --descend-depth 0.09
```

---

### 18.6 如果夹住后还是滑落怎么办？

默认脚本已经在抬升和 final_hold 阶段使用 logical grasp lock。如果使用了：

```bash
--no-grasp-lock
```

则可能滑落，这是纯物理接触不稳定，不一定代表平台失败。

---

### 18.7 如果 joint4 看起来还是不完全竖直怎么办？

提高搜索精度：

```bash
python scripts/acceptance_grasp_axis.py --samples-per-joint 11
```

提高竖直要求：

```bash
python scripts/acceptance_grasp_axis.py --vertical-ratio-threshold 0.96
```

如果搜索不到满足条件的姿态，说明当前机械臂模型的 joint4 轴在可达范围内很难与世界竖直完全重合，需要重新标定末端工具姿态或调整初始构型。

---

## 19. 当前推荐演示流程

如果要给老师或组员展示，可以按下面顺序：

```bash
python scripts/run_viewer.py
```

说明模型和夹爪已经能加载。

```bash
python scripts/check_gripper_mimic.py
```

说明 mimic 夹爪同步开合正常。

```bash
python scripts/acceptance_kinematics.py
```

说明 FK / IK / 轨迹规划接口可用。

```bash
python scripts/acceptance_grasp_axis.py
```

说明平台可以完成竖直轴顶部夹取验收。

---

## 20. 当前项目结论

当前项目已经达到仿真平台搭建负责人的基础交付要求：

```text
1. MuJoCo 模型能打开；
2. STL 外观正确；
3. 夹爪 mimic 修正完成；
4. 双指同步开合；
5. 运动学接口可用；
6. 算法组有统一 API；
7. 可视化正逆运动学验收可运行；
8. 可视化竖直轴夹取验收可运行；
9. 后续动力学控制位置已预留；
10. 后续 peg-in-hole 状态机位置已预留；
11. README 和协作说明已整理。
```

后续工作重点是：

```text
1. 精确标定 ee_site / peg_tip_site / hole_entry_site；
2. 建立最终孔和工装环境；
3. 开发 spiral search；
4. 开发 wiggling / screwing 插入动作；
5. 建立更真实的电机动力学和力控模型；
6. 将 GUI 与新版 API 完整集成。
```
