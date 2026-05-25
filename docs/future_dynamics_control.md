# 未来动力学与扭矩控制位置

当前项目使用 position actuator：

```text
data.ctrl = q_des
```

未来如果要做电机扭矩控制，应新增：

```text
models/robot_with_gripper_torque.xml
```

并在其中使用 motor actuator。控制代码放在：

```text
src/dynamics_control/torque_controller.py
```

建议实现顺序：

1. 重力补偿
2. PD + gravity compensation
3. inverse dynamics feedforward
4. computed torque control
5. impedance / admittance control
6. 接触任务中的力控或柔顺控制
