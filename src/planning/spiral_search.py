# -*- coding: utf-8 -*-
from __future__ import annotations

import numpy as np


class ArchimedesSpiralSearch:
    """2D Archimedes spiral in a local search plane.

    This module is independent of MuJoCo. The task layer decides how to map
    the local offset to the hole frame / world frame.
    """

    def __init__(self, radius_max: float = 0.03, pitch: float = 0.003, angular_speed: float = 2.0):
        self.radius_max = radius_max
        self.pitch = pitch
        self.angular_speed = angular_speed

    def sample_offset(self, t: float) -> np.ndarray:
        theta = self.angular_speed * t
        r = min(self.pitch * theta / (2 * np.pi), self.radius_max)
        return np.array([0.0, r * np.cos(theta), r * np.sin(theta)], dtype=float)
