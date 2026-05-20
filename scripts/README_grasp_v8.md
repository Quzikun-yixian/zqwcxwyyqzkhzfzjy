# acceptance_grasp_axis.py v8

Fixes compared with v7:
1. `link4_visual` contact is treated as a top-stop contact, not as an automatic failure.
2. The shaft is allowed to touch the joint4/link4 head so it cannot visually pass through it.
3. The script will continue to lift if the minimum grasp conditions pass.
4. `final_hold` continues to apply logical grasp lock, so the shaft will not slowly slide down after being lifted.
5. No green helper collision boxes are added to the gripper.

Run:
```powershell
python scripts/acceptance_grasp_axis.py
```

If the top-stop looks too deep, reduce:
```powershell
python scripts/acceptance_grasp_axis.py --top-overlap 0.025
```

If you want stronger top-stop contact:
```powershell
python scripts/acceptance_grasp_axis.py --top-overlap 0.05
```
