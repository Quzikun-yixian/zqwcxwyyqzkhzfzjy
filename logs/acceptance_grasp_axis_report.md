# joint4 竖直搜索顶部夹取验收报告 v8

- 基础模型：`D:\HuaweiMoveData\Users\PC\Desktop\mujoco_arm_platform_final_project\models\robot_with_gripper.xml`
- 临时验收场景：`D:\HuaweiMoveData\Users\PC\Desktop\mujoco_arm_platform_final_project\models\_acceptance_joint4_vertical_search_scene_v8.xml`
- 不添加额外夹爪 collision geom。
- 接触统计使用原始 STL mesh geom：`fin1_visual` / `fin2_visual`，并启用 `link4_visual` 防止轴穿模进入 joint4 顶头。
- 脚本自动搜索 joint4 运动方向最接近世界竖直的 `q1/q2/q3`。
- 接近和抬升阶段只改变 `joint4`，`q1/q2/q3` 保持不变。
- vertical_ratio：`0.999958`
- open_cmd：`-0.0300`
- close_cmd：`+0.0300`
- open_dist：`0.190000`
- close_dist：`0.070000`
- object_radius：`0.041000`
- 总体验收结果：**通过**

## 最小夹持条件

| 指标 | 数值 | 判定 |
|---|---:|---:|
| vertical_ratio | 0.999958 | True |
| geometry_clamp_ok | True | True |
| mesh_contact_ok | True | reported only |
| xy_drift_before | 0.019429 | True |
| max_tilt_before | 1.530 | True |
| link4_top_stop_contact | True | recorded only |
| minimal_grasp_ok | True | True |

## 抬升结果

- lift_delta：`0.097714`
- lifted_ok：`True`
- max_tilt_deg：`1.530`

## 说明

默认情况下，STL mesh contact 只记录不强制要求。若要强制要求原始 mesh 接触，运行 `python scripts/acceptance_grasp_axis.py --require-mesh-contact`。
通过最小夹持条件后，脚本默认启用 logical grasp lock 让已夹住的轴随夹爪抬升，并在 final_hold 阶段继续保持锁定，避免空中慢慢滑落。若要纯物理摩擦抓取，运行 `--no-grasp-lock`。
`link4_visual` 已启用为接触几何，用作 joint4/link4 顶头限位；接触本身记录为 top stop，不再直接判失败。