# -*- coding: utf-8 -*-
from __future__ import annotations

import numpy as np


class ScrewMotion:
    """Simple insertion screw motion command generator placeholder."""

    def __init__(self, push_depth: float = 0.03, screw_amplitude: float = 0.2, duration: float = 5.0):
        self.push_depth = push_depth
        self.screw_amplitude = screw_amplitude
        self.duration = duration

    def sample(self, t: float):
        u = np.clip(t / self.duration, 0.0, 1.0)
        push = self.push_depth * u
        screw = self.screw_amplitude * np.sin(2 * np.pi * u)
        return push, screw
