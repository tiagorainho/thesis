from __future__ import annotations
import numpy as np
from .point_cloud import PointCloud, PointColor
from typing import Tuple, List


class State:
    traversable_pc: PointCloud
    inaccessible_pc: PointCloud

    goal_pc: PointCloud
    update_pc: PointCloud

    previous_positions: List[Tuple[float, float, float]]

    def __init__(self) -> None:
        self.traversable_pc = PointCloud(color=PointColor.TRAVERSABLE)
        self.inaccessible_pc = PointCloud(color=PointColor.BLOCKED)
        self.goal_pc = PointCloud(color=PointColor.GOAL)
        self.update_pc = PointCloud(color=PointColor.UPDATE)
        self.previous_positions = list()

    def set_position(self, new_position):
        self.previous_positions.append(new_position)

    @property
    def current_position(self):
        return self.previous_positions[-1]

    def update(self, state: State):

        # kalman filter with others maps

        # update point cloud tree to improve search performance

        pass

    def walkable_neighbors(self, point, radius: float) -> np.ndarray:
        return self.traversable_pc.neighbors(point=point, radius=radius)

    @property
    def entropy(self):
        return 0.1

    def __repr__(self):
        return str({
            "traversable_pc": self.traversable_pc,
            "goal_pc": self.goal_pc,
            "update_pc": self.update_pc
        })
