# 仿真平台负责人交付清单

你作为仿真平台搭建负责人，需要保证以下内容稳定：

1. 主模型可以加载：`models/robot_with_gripper.xml`
2. 夹爪 mimic 正常：`joint6 = -joint5`
3. 关节命名稳定：`joint1` 到 `joint6`
4. 算法接口稳定：`ArmPlatformAPI`
5. FK / IK 可调用：`RobotKinematics`
6. 简单抓取环境可运行：`models/simple_grasp_scene.xml`
7. 物理参数有配置位置：`configs/grasp_env.yaml`
8. 未来动力学控制有模块位置：`src/dynamics_control/`
9. 详细 README 可以指导合作者运行项目
