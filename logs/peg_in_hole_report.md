# Peg-in-Hole Archimedes Spiral Demo Report

- Final state: `COMPLETE`
- Result: **PASSED**
- Failure reason: `none`
- Target contact force: `5.000` N
- Final XY alignment error: `0.002000` m
- Alignment tolerance: `0.004000` m
- Final insertion depth: `0.025115` m
- Required insertion depth: `0.025000` m
- Max search radius used: `0.016177` m

## Notes

The search trajectory is an Archimedes spiral in the world XY plane.
The shaft is kept attached to the gripper with logical grasp lock after grasping.
The contact force used by the joint4 admittance loop is computed from tip penetration into the hole-entry plane for deterministic task-level validation.