# Stage 3 torque-level force-control report

- Controller: `tau = qfrc_bias + Kp(q_ref-q) - Kd*dq + J^T F_task`
- Force loop: `F_task = Kf*e_F + Ki*integral(e_F dt)`
- force_integral_gain: `2.000`
- Target force: `3.000` N
- Final filtered force: `2.902` N
- Tail mean filtered force: `2.902` N
- Tail mean error: `0.098` N
- vertical_ratio: `0.999958`
- q_pre: `[-0.80246  -0.794415  0.490331  0.055   ]`
- q_contact: `[-0.80246  -0.794415  0.490331  0.155   ]`
- max_abs_tau: `[ 0.098618  0.050195  4.594605 26.35007 ]`
- Result: **PASSED**

## Notes

This demo uses motor actuators for joint1--joint4. The gripper remains a position actuator.
The force term is generated in Cartesian space and mapped to joint torques through `J^T F`.
The script still uses logical grasp lock after the shaft is grasped, so the object attachment is a task-level constraint.