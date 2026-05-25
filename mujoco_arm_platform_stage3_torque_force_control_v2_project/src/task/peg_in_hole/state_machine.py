# -*- coding: utf-8 -*-
from __future__ import annotations

from enum import Enum, auto


class PegInHoleState(Enum):
    IDLE = auto()
    APPROACH_HOLE = auto()
    GRASP_PEG = auto()
    MOVE_TO_PRE_INSERT = auto()
    REACHING_PUSH = auto()
    SEARCHING_SPIRAL = auto()
    INSERTING_WIGGLE_SCREW = auto()
    COMPLETE = auto()
    FAILED = auto()


class PegInHoleStateMachine:
    """Task-level shell.

    This module should be the only place that encodes the task sequence.
    Reaching/searching/inserting motion details should call planning modules.
    """

    def __init__(self):
        self.state = PegInHoleState.IDLE

    def reset(self):
        self.state = PegInHoleState.IDLE

    def step(self, context):
        # Keep this intentionally thin. Add actual conditions later:
        # velocity threshold, contact monitor, site pose error, insertion depth.
        return self.state
