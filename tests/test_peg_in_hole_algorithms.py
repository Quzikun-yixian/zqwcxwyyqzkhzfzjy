# -*- coding: utf-8 -*-
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from planning.screw_motion import ScrewMotion
from planning.spiral_search import ArchimedesSpiralSearch
from task.peg_in_hole.state_machine import PegInHoleContext, PegInHoleState, PegInHoleStateMachine


class ArchimedesSpiralSearchTest(unittest.TestCase):
    def test_radius_is_monotonic_and_capped(self):
        search = ArchimedesSpiralSearch(radius_max=0.03, pitch=0.003, angular_speed=2.0)
        radii = [search.radius(t) for t in np.linspace(0.0, 80.0, 200)]

        self.assertGreaterEqual(min(radii), 0.0)
        self.assertLessEqual(max(radii), 0.03)
        self.assertTrue(all(a <= b + 1e-12 for a, b in zip(radii, radii[1:])))

    def test_radius_increases_by_pitch_per_turn(self):
        search = ArchimedesSpiralSearch(radius_max=0.03, pitch=0.003, angular_speed=2.0)
        one_turn_time = 2.0 * np.pi / search.angular_speed

        self.assertAlmostEqual(search.radius(one_turn_time), 0.003, places=8)
        self.assertAlmostEqual(search.radius(2.0 * one_turn_time), 0.006, places=8)

    def test_offset_is_in_xy_plane(self):
        search = ArchimedesSpiralSearch(radius_max=0.03, pitch=0.003, angular_speed=2.0)
        offset = search.sample_offset(7.5)

        self.assertEqual(offset.shape, (3,))
        self.assertAlmostEqual(float(offset[2]), 0.0, places=12)


class ScrewMotionTest(unittest.TestCase):
    def test_depth_and_wiggle_are_bounded(self):
        motion = ScrewMotion(push_depth=0.03, wiggle_amplitude=0.002, screw_amplitude=0.2, duration=5.0, screw_turns=3.0)

        start = motion.sample_command(0.0)
        mid = motion.sample_command(2.5)
        end = motion.sample_command(5.0)

        self.assertAlmostEqual(start.insertion_depth, 0.0)
        self.assertAlmostEqual(mid.insertion_depth, 0.015)
        self.assertAlmostEqual(end.insertion_depth, 0.03)
        self.assertLessEqual(float(np.linalg.norm(start.wiggle_offset)), 0.002 + 1e-12)
        self.assertLessEqual(float(np.linalg.norm(mid.wiggle_offset)), 0.002 + 1e-12)
        self.assertLessEqual(float(np.linalg.norm(end.wiggle_offset)), 0.002 + 1e-12)
        self.assertAlmostEqual(end.screw_angle, 6.0 * np.pi)


class PegInHoleStateMachineTest(unittest.TestCase):
    def test_normal_path_reaches_complete(self):
        machine = PegInHoleStateMachine()
        ctx = PegInHoleContext(alignment_tolerance=0.004, insert_depth_target=0.025)

        self.assertEqual(machine.step(ctx), PegInHoleState.APPROACH_HOLE)

        ctx.approach_done = True
        self.assertEqual(machine.step(ctx), PegInHoleState.GRASP_PEG)

        ctx.grasp_done = True
        self.assertEqual(machine.step(ctx), PegInHoleState.MOVE_TO_PRE_INSERT)

        ctx.pre_insert_done = True
        self.assertEqual(machine.step(ctx), PegInHoleState.REACHING_PUSH)

        ctx.contact_detected = True
        self.assertEqual(machine.step(ctx), PegInHoleState.SEARCHING_SPIRAL)

        ctx.alignment_error_xy = 0.001
        self.assertEqual(machine.step(ctx), PegInHoleState.INSERTING_WIGGLE_SCREW)

        ctx.insertion_depth = 0.026
        self.assertEqual(machine.step(ctx), PegInHoleState.COMPLETE)

    def test_search_timeout_fails(self):
        machine = PegInHoleStateMachine()
        ctx = PegInHoleContext(
            approach_done=True,
            grasp_done=True,
            pre_insert_done=True,
            contact_detected=True,
            alignment_error_xy=0.02,
            search_elapsed=31.0,
            search_timeout=30.0,
        )

        machine.step(ctx)
        machine.step(ctx)
        machine.step(ctx)
        machine.step(ctx)
        self.assertEqual(machine.step(ctx), PegInHoleState.SEARCHING_SPIRAL)
        self.assertEqual(machine.step(ctx), PegInHoleState.FAILED)
        self.assertEqual(machine.failure_reason, "search_timeout")


if __name__ == "__main__":
    unittest.main()
