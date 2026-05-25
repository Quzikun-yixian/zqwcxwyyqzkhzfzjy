# 算法组接口说明

算法组推荐只导入：

```python
from api.arm_platform_api import ArmPlatformAPI
```

不要直接操作 MuJoCo 底层数据结构。

示例：

```python
api = ArmPlatformAPI("models/robot_with_gripper.xml")
state = api.get_state()

target = state.ee_pos + [0.03, 0.02, 0.02]
ik = api.ik_position(target)

if ik.success:
    api.plan_joint_trajectory(ik.q, duration=4.0, method="quintic")

while running:
    api.step()
```

统一变量：

```text
q_arm = [q1, q2, q3, d4]
g = gripper_opening
```
