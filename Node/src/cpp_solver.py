from typing import Set, Tuple
from point_cloud import PointCloud
import numpy as np


class CPPSolver:

    robot_width: float

    def __init__(self, robot_width: float):
        self.robot_width = robot_width

    def plan(self,
             traversable_pc: PointCloud,
             goal_pc: PointCloud,
             start_point: np.ndarray,
             covered_points: Set[Tuple[float]]) -> np.ndarray:

        raise NotImplementedError()
