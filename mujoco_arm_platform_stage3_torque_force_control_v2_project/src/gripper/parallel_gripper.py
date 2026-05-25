# -*- coding: utf-8 -*-
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ParallelGripper:
    """High-level gripper interface.

    The rest of the project should call open(), close(), or set_opening().
    Nobody outside this module should directly write joint5 / joint6.
    """

    controller: object
    open_command: float = -0.03
    close_command: float = 0.03

    def set_opening(self, value: float):
        return self.controller.set_target([value])

    def open(self):
        return self.set_opening(self.open_command)

    def close(self):
        return self.set_opening(self.close_command)

    def get_opening(self) -> float:
        return float(self.controller.get_q()[0])
