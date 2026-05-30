# Peg-in-Hole Paper Algorithm Reproduction Report

- Reference paper: `Compliance-Based Robotic Peg-in-Hole Assembly Strategy Without Force Feedback`
- Reproduction level: `algorithm-level simulation`
- Final state: `COMPLETE`
- Result: **PASSED**
- Failure reason: `none`
- Simulated vision offset mode: `random`
- Simulated vision XY offset: `[-0.0194 -0.0014]` m
- Random offset radius limit: `0.020000` m
- Random seed: `4`
- Spiral search radius: `0.020000` m
- Spiral pitch: `0.003000` m/turn
- Spiral angular speed: `2.000000` rad/s
- Target contact force: `5.000` N
- Velocity/contact threshold: `0.020000` m/s for `0.300` s
- Wiggle amplitude: `0.002000` m
- Screw turns: `3.000`
- Final XY alignment error: `0.001996` m
- Alignment tolerance: `0.004000` m
- Final insertion depth: `0.025106` m
- Required insertion depth: `0.025000` m
- Hole XY radius from base: `0.514213` m
- Final shaft/tool axis: `[-0.0146 -0.0112 -0.9998]`
- Final tool tilt from world vertical: `1.056` deg
- Max search radius used: `0.017953` m
- CSV log: `peg_in_hole_sweep_search_radius_small_log.csv`
- Spiral trace image: `peg_in_hole_sweep_search_radius_small_spiral_trace.svg`

## Spiral Trace

![Archimedes spiral rubbing trace](peg_in_hole_sweep_search_radius_small_spiral_trace.svg)

## Paper Procedure Mapping

| Paper step | Unit motion represented in this demo |
| --- | --- |
| Setup | initial gripper grasp, then carry peg to estimated hole pose |
| Reaching | pushing |
| Searching | pushing + rubbing / Archimedes spiral |
| Inserting | pushing + wiggling + screwing |

## Notes

This is an algorithm-level reproduction of the paper workflow.
The simulated vision estimate is the hole pose plus a random bounded initial XY offset.
Pushing, rubbing, wiggling, and screwing are represented as task-level motion primitives.
The demo uses deterministic geometric contact cues instead of a full physical contact-state estimator.
It does not reproduce Kinect recognition, torque-level compliant control, or 0.01 mm clearance experiments.