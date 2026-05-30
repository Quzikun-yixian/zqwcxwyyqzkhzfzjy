# Peg-in-Hole Parameter Sweep

- Reproduction level: `algorithm-level simulation`
- Cases passed: `6/6`
- Purpose: compare random simulated vision offsets, Archimedes spiral parameters, and wiggling/screwing variants.

| Case | Seed | Offset XY (m) | Pass | Final state | XY error (m) | Insertion depth (m) | Search radius | Pitch | Wiggle | Screw turns |
| --- | ---: | ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| random_seed_1 | 1 | (+0.0136, -0.0044) | True | COMPLETE | 0.001991 | 0.025060 | 0.030 | 0.0030 | 0.0020 | 3.0 |
| random_seed_2 | 2 | (-0.0031, +0.0098) | True | COMPLETE | 0.001992 | 0.025062 | 0.030 | 0.0030 | 0.0020 | 3.0 |
| random_seed_3 | 3 | (+0.0005, +0.0058) | True | COMPLETE | 0.001991 | 0.025006 | 0.030 | 0.0030 | 0.0020 | 3.0 |
| search_radius_small | 4 | (-0.0194, -0.0014) | True | COMPLETE | 0.001996 | 0.025106 | 0.020 | 0.0030 | 0.0020 | 3.0 |
| spiral_pitch_coarse | 5 | (+0.0064, -0.0168) | True | COMPLETE | 0.001994 | 0.025083 | 0.030 | 0.0040 | 0.0020 | 3.0 |
| wiggle_screw_variant | 6 | (-0.0081, +0.0122) | True | COMPLETE | 0.002988 | 0.025065 | 0.030 | 0.0030 | 0.0030 | 4.0 |

## Notes

This sweep reuses the same task-level demo. It is not a torque-control or high-fidelity contact reproduction.