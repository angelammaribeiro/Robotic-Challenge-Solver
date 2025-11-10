from enum import Enum


class Checkpoint(Enum):
    """Classification for neighboring cells relative to the robot."""

    FREE = 0
    OBSTACLE = 1
    UNKNOWN = 2


__all__ = ["Checkpoint"]