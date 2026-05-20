# 可视化正逆运动学验收报告

- 模型文件：`D:\HuaweiMoveData\Users\PC\Desktop\mujoco_arm_platform_final_project\models\robot_with_gripper.xml`
- 初始关节：`[0. 0. 0. 0.]`
- 初始末端位置：`[ 0.489591 -0.714816  0.456058]`
- FK 误差阈值：`0.005` m
- 总体验收结果：**通过**

## IK 测试结果

| Case | Pass | IK success | IK error (m) | FK error (m) | Iterations |
|---|---:|---:|---:|---:|---:|
| reachable_fk_target_1 | True | True | 5.196792e-05 | 5.196792e-05 | 6 |
| reachable_fk_target_2 | True | True | 9.154540e-05 | 9.154540e-05 | 5 |
| reachable_fk_target_3 | True | True | 5.685397e-05 | 5.685397e-05 | 7 |
| reachable_fk_target_4 | True | True | 4.056740e-05 | 4.056740e-05 | 7 |

## 说明

本脚本会直接打开 MuJoCo Viewer，并用 qpos + mj_forward 进行运动学回放。因此它验收的是 FK、IK 和轨迹规划接口，不把动力学执行器跟踪误差算作失败。