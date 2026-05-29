# -*- coding: utf-8 -*-
"""
Run a peg-in-hole demo with Archimedes spiral search.

The demo is task-level and intentionally keeps grasping stable with a logical
grasp lock after the shaft has been grasped. The search plane is world XY and
the insertion direction is world Z downward.

Run from project root:
    python scripts/run_peg_in_hole_demo.py
    python scripts/run_peg_in_hole_demo.py --headless --no-sleep
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
sys.path.insert(0, str(SRC_DIR))

import mujoco
import mujoco.viewer

from api.arm_platform_api import ArmPlatformAPI
from control.admittance_controller import Joint4AdmittanceController
from planning.joint_trajectory import JointTrajectory
from planning.screw_motion import ScrewMotion
from planning.spiral_search import ArchimedesSpiralSearch
from task.peg_in_hole.state_machine import PegInHoleContext, PegInHoleState, PegInHoleStateMachine


@dataclass
class DemoResult:
    passed: bool
    final_state: PegInHoleState
    final_alignment_error: float
    final_insertion_depth: float
    max_search_radius_used: float
    failure_reason: str
    report_path: Path
    csv_path: Path


def array_str(x: np.ndarray, precision: int = 6) -> str:
    return np.array2string(np.asarray(x), precision=precision, suppress_small=False)


def get_id(model, objtype, name: str) -> int:
    oid = mujoco.mj_name2id(model, objtype, name)
    if oid < 0:
        raise ValueError(f"Object not found: {name}")
    return oid


def get_site_pos(model, data, name: str) -> np.ndarray:
    sid = get_id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    mujoco.mj_forward(model, data)
    return data.site_xpos[sid].copy()


def set_q_arm_kinematic(api: ArmPlatformAPI, q_arm: np.ndarray) -> None:
    api.data.qpos[api.kin.qpos_idx] = q_arm
    api.data.qvel[:] = 0.0
    api.arm_target = np.asarray(q_arm, dtype=float).copy()
    mujoco.mj_forward(api.model, api.data)


def set_slide_pair_direct(model, data, joint5_value: float) -> None:
    j5 = get_id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint5")
    j6 = get_id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint6")
    data.qpos[model.jnt_qposadr[j5]] = joint5_value
    data.qpos[model.jnt_qposadr[j6]] = -joint5_value
    mujoco.mj_forward(model, data)


def finger_distance(model, data) -> float:
    left = get_site_pos(model, data, "left_finger_tip_site")
    right = get_site_pos(model, data, "right_finger_tip_site")
    return float(np.linalg.norm(left - right))


def detect_open_close_commands(api: ArmPlatformAPI, a: float = 0.03):
    old_qpos = api.data.qpos.copy()
    old_qvel = api.data.qvel.copy()

    set_slide_pair_direct(api.model, api.data, -a)
    d_minus = finger_distance(api.model, api.data)

    set_slide_pair_direct(api.model, api.data, +a)
    d_plus = finger_distance(api.model, api.data)

    api.data.qpos[:] = old_qpos
    api.data.qvel[:] = old_qvel
    mujoco.mj_forward(api.model, api.data)

    if d_minus >= d_plus:
        return -a, +a
    return +a, -a


def set_freejoint_pose(model, data, joint_name: str, pos: np.ndarray, quat=None) -> None:
    if quat is None:
        quat = np.array([1.0, 0.0, 0.0, 0.0])
    jid = get_id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    qadr = int(model.jnt_qposadr[jid])
    vadr = int(model.jnt_dofadr[jid])
    data.qpos[qadr:qadr + 3] = pos
    data.qpos[qadr + 3:qadr + 7] = quat
    data.qvel[vadr:vadr + 6] = 0.0
    mujoco.mj_forward(model, data)


def quat_about_z(angle: float) -> np.ndarray:
    half = 0.5 * float(angle)
    return np.array([np.cos(half), 0.0, 0.0, np.sin(half)], dtype=float)


def grasp_site_pos(api: ArmPlatformAPI, peg_tip_site: str) -> np.ndarray:
    return get_site_pos(api.model, api.data, peg_tip_site)


def shaft_tip_pos(api: ArmPlatformAPI, peg_tip_site: str, grasp_to_tip: float) -> np.ndarray:
    return grasp_site_pos(api, peg_tip_site) - np.array([0.0, 0.0, grasp_to_tip], dtype=float)


def apply_grasp_lock(
    api: ArmPlatformAPI,
    peg_tip_site: str,
    object_center_offset: np.ndarray,
    screw_angle: float = 0.0,
) -> None:
    grasp_pos = grasp_site_pos(api, peg_tip_site)
    set_freejoint_pose(
        api.model,
        api.data,
        "grasp_object_freejoint",
        grasp_pos + object_center_offset,
        quat=quat_about_z(screw_angle),
    )


def pseudo_contact_force(peg_tip: np.ndarray, hole_entry: np.ndarray, stiffness: float, contact_tolerance: float) -> float:
    contact_plane_z = float(hole_entry[2] + contact_tolerance)
    penetration = max(0.0, contact_plane_z - float(peg_tip[2]))
    return float(stiffness * penetration)


def load_task_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)["peg_in_hole"]


def sync_viewer(viewer, no_sleep: bool, dt: float) -> bool:
    if viewer is None:
        if not no_sleep:
            time.sleep(dt)
        return True
    if not viewer.is_running():
        return False
    viewer.sync()
    if not no_sleep:
        time.sleep(dt)
    return True


def write_report(
    path: Path,
    result: DemoResult,
    target_force: float,
    alignment_tolerance: float,
    success_depth: float,
) -> None:
    lines = []
    lines.append("# Peg-in-Hole Archimedes Spiral Demo Report\n")
    lines.append(f"- Final state: `{result.final_state.name}`")
    lines.append(f"- Result: **{'PASSED' if result.passed else 'FAILED'}**")
    lines.append(f"- Failure reason: `{result.failure_reason or 'none'}`")
    lines.append(f"- Target contact force: `{target_force:.3f}` N")
    lines.append(f"- Final XY alignment error: `{result.final_alignment_error:.6f}` m")
    lines.append(f"- Alignment tolerance: `{alignment_tolerance:.6f}` m")
    lines.append(f"- Final insertion depth: `{result.final_insertion_depth:.6f}` m")
    lines.append(f"- Required insertion depth: `{success_depth:.6f}` m")
    lines.append(f"- Max search radius used: `{result.max_search_radius_used:.6f}` m")
    lines.append("\n## Notes\n")
    lines.append("The search trajectory is an Archimedes spiral in the world XY plane.")
    lines.append("The shaft is kept attached to the gripper with logical grasp lock after grasping.")
    lines.append("The contact force used by the joint4 admittance loop is computed from tip penetration into the hole-entry plane for deterministic task-level validation.")
    path.write_text("\n".join(lines), encoding="utf-8")


def run_demo(args, viewer=None, api: Optional[ArmPlatformAPI] = None) -> DemoResult:
    cfg = load_task_config(PROJECT_ROOT / "configs" / "peg_in_hole_task.yaml")
    geom_cfg = cfg["geometry"]
    search_cfg = cfg["search"]
    insertion_cfg = cfg["insertion"]
    force_cfg = cfg["force_control"]
    contact_cfg = cfg["contact_detection"]
    site_cfg = cfg["sites"]

    model_path = (PROJECT_ROOT / args.model).resolve()
    log_dir = PROJECT_ROOT / "logs"
    log_dir.mkdir(exist_ok=True)
    csv_path = log_dir / "peg_in_hole_log.csv"
    report_path = log_dir / "peg_in_hole_report.md"

    if api is None:
        api = ArmPlatformAPI(model_path)
    open_cmd, close_cmd = detect_open_close_commands(api)

    peg_tip_site = site_cfg["peg_tip"]
    hole_entry_site = site_cfg["hole_entry"]

    hole_entry = get_site_pos(api.model, api.data, hole_entry_site)
    initial_offset = np.array(args.initial_offset_xy, dtype=float)
    shaft_half_length = float(geom_cfg["shaft_half_length"])
    grasp_to_tip = float(geom_cfg["grasp_to_tip"])
    object_center_offset = np.array([0.0, 0.0, shaft_half_length - grasp_to_tip], dtype=float)
    pre_insert_clearance = float(geom_cfg["pre_insert_clearance"])
    alignment_tolerance = float(args.alignment_tolerance)
    contact_tolerance = float(args.contact_tolerance)
    success_depth = float(args.success_depth)
    pseudo_stiffness = float(force_cfg["pseudo_contact_stiffness"])
    dt = float(args.dt)

    spiral = ArchimedesSpiralSearch(
        radius_max=float(args.search_radius),
        pitch=float(args.spiral_pitch),
        angular_speed=float(args.spiral_angular_speed),
    )
    screw = ScrewMotion(
        push_depth=float(args.insert_depth),
        screw_amplitude=float(insertion_cfg["screw_amplitude_rad"]),
        duration=float(insertion_cfg["duration"]),
        wiggle_amplitude=float(insertion_cfg["wiggle_amplitude"]),
        screw_turns=float(insertion_cfg["screw_turns"]),
    )

    machine = PegInHoleStateMachine()
    ctx = PegInHoleContext(
        alignment_tolerance=alignment_tolerance,
        insert_depth_target=success_depth,
        insert_timeout=float(insertion_cfg["timeout"]),
        search_timeout=float(args.max_time),
        max_search_radius=float(args.search_radius),
    )

    sim_time = 0.0
    state_enter_time = 0.0
    current_q = api.get_state().q_arm.copy()
    max_search_radius_used = 0.0
    last_spiral_offset = np.zeros(2, dtype=float)
    last_wiggle_offset = np.zeros(2, dtype=float)
    last_screw_angle = 0.0

    def machine_step() -> PegInHoleState:
        nonlocal state_enter_time
        old = machine.state
        new = machine.step(ctx)
        if new != old:
            state_enter_time = sim_time
            ctx.state_elapsed = 0.0
        return new

    def update_context() -> None:
        peg = shaft_tip_pos(api, peg_tip_site, grasp_to_tip)
        ctx.time = sim_time
        ctx.state_elapsed = sim_time - state_enter_time
        ctx.alignment_error_xy = float(np.linalg.norm(peg[:2] - hole_entry[:2]))
        ctx.insertion_depth = max(0.0, float(hole_entry[2] - peg[2]))
        ctx.contact_force = pseudo_contact_force(peg, hole_entry, pseudo_stiffness, contact_tolerance)
        ctx.contact_detected = ctx.contact_detected or ctx.contact_force > 0.05 or peg[2] <= hole_entry[2] + contact_tolerance

    def write_log(writer, phase: str) -> None:
        peg = shaft_tip_pos(api, peg_tip_site, grasp_to_tip)
        state = api.get_state()
        writer.writerow([
            sim_time,
            machine.state.name,
            phase,
            state.q_arm[0],
            state.q_arm[1],
            state.q_arm[2],
            state.q_arm[3],
            peg[0],
            peg[1],
            peg[2],
            hole_entry[0],
            hole_entry[1],
            hole_entry[2],
            ctx.alignment_error_xy,
            ctx.insertion_depth,
            ctx.contact_force,
            last_spiral_offset[0],
            last_spiral_offset[1],
            ctx.search_radius,
            last_wiggle_offset[0],
            last_wiggle_offset[1],
            last_screw_angle,
            machine.failure_reason,
        ])

    def set_target_by_ik(target_pos: np.ndarray, q_init: np.ndarray) -> np.ndarray:
        ik = api.ik_position(target_pos, q_init=q_init, site_name=peg_tip_site)
        if not ik.success and ik.error_norm > args.ik_tolerance:
            ctx.ik_failed = True
            machine_step()
            raise RuntimeError(f"IK failed for target {array_str(target_pos)}; error={ik.error_norm:.6g}")
        return ik.q.copy()

    def animate_to(target_pos: np.ndarray, duration: float, phase: str, writer, lock_object: bool) -> np.ndarray:
        nonlocal sim_time, current_q
        q_goal = set_target_by_ik(target_pos, current_q)
        traj = JointTrajectory(current_q, q_goal, max(dt, duration), method="quintic")
        steps = max(1, int(duration / dt))
        for k in range(steps + 1):
            u_time = min(duration, k * dt)
            q = traj.sample(u_time)
            set_q_arm_kinematic(api, q)
            if lock_object:
                apply_grasp_lock(api, peg_tip_site, object_center_offset, screw_angle=last_screw_angle)
            update_context()
            write_log(writer, phase)
            if not sync_viewer(viewer, args.no_sleep, dt):
                break
            sim_time += dt
        current_q = q_goal
        set_q_arm_kinematic(api, current_q)
        if lock_object:
            apply_grasp_lock(api, peg_tip_site, object_center_offset, screw_angle=last_screw_angle)
        update_context()
        return current_q

    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "time",
            "state",
            "phase",
            "q1",
            "q2",
            "q3",
            "q4",
            "peg_x",
            "peg_y",
            "peg_z",
            "hole_x",
            "hole_y",
            "hole_z",
            "alignment_error_xy",
            "insertion_depth",
            "contact_force",
            "spiral_x",
            "spiral_y",
            "spiral_radius",
            "wiggle_x",
            "wiggle_y",
            "screw_angle",
            "failure_reason",
        ])

        machine_step()

        pre_insert_target = hole_entry + np.array([
            initial_offset[0],
            initial_offset[1],
            grasp_to_tip + pre_insert_clearance,
        ])

        set_slide_pair_direct(api.model, api.data, open_cmd)
        current_q = animate_to(pre_insert_target, duration=2.0, phase="approach_hole", writer=writer, lock_object=False)
        ctx.approach_done = True
        machine_step()

        set_slide_pair_direct(api.model, api.data, close_cmd)
        apply_grasp_lock(api, peg_tip_site, object_center_offset, screw_angle=last_screw_angle)
        for _ in range(max(1, int(0.4 / dt))):
            update_context()
            write_log(writer, "grasp_peg")
            if not sync_viewer(viewer, args.no_sleep, dt):
                break
            sim_time += dt
        ctx.grasp_done = True
        machine_step()

        current_q = animate_to(pre_insert_target, duration=0.4, phase="move_to_pre_insert", writer=writer, lock_object=True)
        ctx.pre_insert_done = True
        machine_step()

        contact_target = hole_entry + np.array([initial_offset[0], initial_offset[1], grasp_to_tip], dtype=float)
        current_q = animate_to(contact_target, duration=2.0, phase="reach_contact_plane", writer=writer, lock_object=True)

        down_test = current_q.copy()
        up_test = current_q.copy()
        low, high = api.arm_controller.limits()
        down_test[3] = min(high[3], current_q[3] + 0.01)
        up_test[3] = max(low[3], current_q[3] - 0.01)
        set_q_arm_kinematic(api, down_test)
        z_plus = shaft_tip_pos(api, peg_tip_site, grasp_to_tip)[2]
        set_q_arm_kinematic(api, up_test)
        z_minus = shaft_tip_pos(api, peg_tip_site, grasp_to_tip)[2]
        down_sign = 1.0 if z_plus < z_minus else -1.0
        set_q_arm_kinematic(api, current_q)
        apply_grasp_lock(api, peg_tip_site, object_center_offset, screw_angle=last_screw_angle)

        admittance = Joint4AdmittanceController(
            initial_command=float(current_q[3]),
            down_sign=down_sign,
            command_min=float(low[3]),
            command_max=float(high[3]),
            target_force=float(args.target_force),
            k_force=float(force_cfg["k_force"]),
            max_speed=float(force_cfg["max_speed"]),
            filter_alpha=float(force_cfg["filter_alpha"]),
            deadband=float(force_cfg["deadband"]),
        )
        admittance.reset(command=float(current_q[3]), measured_force=ctx.contact_force)

        for _ in range(max(1, int(1.2 / dt))):
            reading = ctx.contact_force
            ctrl = admittance.update(reading, dt)
            q_cmd = current_q.copy()
            q_cmd[3] = ctrl.command
            set_q_arm_kinematic(api, q_cmd)
            current_q = q_cmd
            apply_grasp_lock(api, peg_tip_site, object_center_offset, screw_angle=last_screw_angle)
            update_context()
            write_log(writer, "joint4_admittance_contact")
            if not sync_viewer(viewer, args.no_sleep, dt):
                break
            sim_time += dt
            if ctx.contact_force >= 0.25 * args.target_force:
                ctx.contact_detected = True
                break

        if not ctx.contact_detected:
            ctx.contact_detected = ctx.contact_force > 0.05
        machine_step()

        search_start = sim_time
        while machine.state == PegInHoleState.SEARCHING_SPIRAL and sim_time <= args.max_time:
            t_search = sim_time - search_start
            last_spiral_offset[:] = spiral.sample_xy(t_search)
            ctx.search_elapsed = t_search
            ctx.search_radius = spiral.radius(t_search)
            max_search_radius_used = max(max_search_radius_used, ctx.search_radius)

            target_xy = hole_entry[:2] + initial_offset + last_spiral_offset
            target_tip_z = hole_entry[2] - max(ctx.insertion_depth, args.target_force / pseudo_stiffness)
            target = np.array([target_xy[0], target_xy[1], target_tip_z + grasp_to_tip], dtype=float)
            current_q = set_target_by_ik(target, current_q)
            set_q_arm_kinematic(api, current_q)
            apply_grasp_lock(api, peg_tip_site, object_center_offset, screw_angle=last_screw_angle)
            update_context()
            write_log(writer, "archimedes_spiral_search")
            machine_step()
            if not sync_viewer(viewer, args.no_sleep, dt):
                break
            sim_time += dt

        insert_start = sim_time
        insert_start_offset = shaft_tip_pos(api, peg_tip_site, grasp_to_tip)[:2] - hole_entry[:2]
        insert_start_depth = ctx.insertion_depth
        handoff_duration = max(dt, float(insertion_cfg["handoff_duration"]))
        while machine.state == PegInHoleState.INSERTING_WIGGLE_SCREW and sim_time <= args.max_time:
            t_insert = sim_time - insert_start
            cmd = screw.sample_command(t_insert)
            last_wiggle_offset[:] = cmd.wiggle_offset
            last_screw_angle = cmd.screw_angle

            blend = min(1.0, t_insert / handoff_duration)
            smooth_offset = (1.0 - blend) * insert_start_offset + blend * cmd.wiggle_offset
            target_xy = hole_entry[:2] + smooth_offset
            target_tip_z = hole_entry[2] - (insert_start_depth + cmd.insertion_depth)
            target = np.array([target_xy[0], target_xy[1], target_tip_z + grasp_to_tip], dtype=float)
            current_q = set_target_by_ik(target, current_q)
            set_q_arm_kinematic(api, current_q)
            apply_grasp_lock(api, peg_tip_site, object_center_offset, screw_angle=last_screw_angle)
            update_context()
            write_log(writer, "wiggle_screw_insert")
            machine_step()
            if not sync_viewer(viewer, args.no_sleep, dt):
                break
            sim_time += dt

        update_context()
        if machine.state not in (PegInHoleState.COMPLETE, PegInHoleState.FAILED):
            machine.failure_reason = "max_time_reached"
            ctx.failure_reason = machine.failure_reason
            machine.state = PegInHoleState.FAILED

        write_log(writer, "final")

    passed = (
        machine.state == PegInHoleState.COMPLETE
        and ctx.alignment_error_xy <= alignment_tolerance
        and ctx.insertion_depth >= success_depth
    )

    result = DemoResult(
        passed=bool(passed),
        final_state=machine.state,
        final_alignment_error=float(ctx.alignment_error_xy),
        final_insertion_depth=float(ctx.insertion_depth),
        max_search_radius_used=float(max_search_radius_used),
        failure_reason=ctx.failure_reason or machine.failure_reason,
        report_path=report_path,
        csv_path=csv_path,
    )
    write_report(
        report_path,
        result,
        target_force=float(args.target_force),
        alignment_tolerance=alignment_tolerance,
        success_depth=success_depth,
    )
    return result


def parse_args() -> argparse.Namespace:
    cfg = load_task_config(PROJECT_ROOT / "configs" / "peg_in_hole_task.yaml")
    geom_cfg = cfg["geometry"]
    search_cfg = cfg["search"]
    insertion_cfg = cfg["insertion"]
    force_cfg = cfg["force_control"]
    contact_cfg = cfg["contact_detection"]

    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=str, default=cfg["model_xml"])
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-sleep", action="store_true")
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--max-time", type=float, default=float(search_cfg["timeout"]))
    parser.add_argument("--search-radius", type=float, default=float(search_cfg["radius_max"]))
    parser.add_argument("--spiral-pitch", type=float, default=float(search_cfg["pitch"]))
    parser.add_argument("--spiral-angular-speed", type=float, default=float(search_cfg["angular_speed"]))
    parser.add_argument("--insert-depth", type=float, default=float(insertion_cfg["push_depth"]))
    parser.add_argument("--success-depth", type=float, default=float(insertion_cfg["success_depth"]))
    parser.add_argument("--target-force", type=float, default=float(force_cfg["target_force"]))
    parser.add_argument("--alignment-tolerance", type=float, default=float(geom_cfg["alignment_tolerance"]))
    parser.add_argument("--contact-tolerance", type=float, default=float(contact_cfg["contact_tolerance"]))
    parser.add_argument("--ik-tolerance", type=float, default=0.002)
    parser.add_argument(
        "--initial-offset-xy",
        type=float,
        nargs=2,
        default=list(geom_cfg["initial_search_offset_xy"]),
        metavar=("DX", "DY"),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    print("=" * 80)
    print("Peg-in-Hole Archimedes Spiral Demo")
    print("=" * 80)
    print(f"Model: {PROJECT_ROOT / args.model}")
    print(f"Headless: {args.headless}")
    print(f"Spiral radius={args.search_radius:.4f}, pitch={args.spiral_pitch:.4f}, angular_speed={args.spiral_angular_speed:.4f}")
    print(f"Target force={args.target_force:.3f} N, insert_depth={args.insert_depth:.4f} m")

    if args.headless:
        result = run_demo(args, viewer=None)
    else:
        api = ArmPlatformAPI((PROJECT_ROOT / args.model).resolve())
        with mujoco.viewer.launch_passive(api.model, api.data) as viewer:
            viewer.cam.distance = 1.2
            result = run_demo(args, viewer=viewer, api=api)

    print("=" * 80)
    print(f"Final state: {result.final_state.name}")
    print(f"Final XY alignment error: {result.final_alignment_error:.6f} m")
    print(f"Final insertion depth: {result.final_insertion_depth:.6f} m")
    print(f"Max search radius used: {result.max_search_radius_used:.6f} m")
    print(f"Report: {result.report_path}")
    print(f"CSV log: {result.csv_path}")
    print(f"Result: {'PASSED' if result.passed else 'FAILED'}")
    print("=" * 80)

    if not result.passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
