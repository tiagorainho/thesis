from time import perf_counter
import numpy as np
import logging
from typing import Mapping

from state import State
from planner import Planner
from point_cloud import PointCloud
from state_manager import StateManager

logger = logging.getLogger("Orchestrator")
MAX_ENTROPY_ALLOWED = 400


class Orchestrator:
    identifier: any
    planner: Planner
    state_manager: StateManager

    def __init__(self, identifier: any, planner: Planner, state_manager: StateManager) -> None:
        self.identifier = identifier
        self.planner = planner
        self.state_manager = state_manager

    @property
    def state(self) -> State:
        return self.state_manager.state

    def verify_current_plan(self) -> bool:

        # check if planning is possible
        start_time = perf_counter()
        plan_traversable = False  # self.planner.verify(self.state)

        logging_message = f"[VERIFICATION] {plan_traversable}: {round(perf_counter()-start_time, 3)} secs"
        if not plan_traversable:
            logger.error(logging_message)
            return False
        logger.debug(logging_message)

        # calculate entropy and check if its in reasonable bounds
        if self.state.entropy > MAX_ENTROPY_ALLOWED:
            return False

        return True

    def update(self, position: np.ndarray, other_positions: Mapping[any, np.ndarray]):

        # update state
        # self.state.update(state=None)

        # self.planner.plan.path = []  # to delete

        # verify if the state is not possible
        if not self.verify_current_plan():

            # divide areas
            positions: Mapping[any, np.ndarray] = other_positions
            positions[self.identifier] = position
            initial_positions = [tuple(v) for k, v in positions.items()]
            array_position = sorted(positions.keys()).index(self.identifier)
            different_areas = self.state.traversable_pc.divide_areas(
                starting_points=initial_positions,
                max_neighbor_distance=0.5/2
            )

            # get and assign goal points
            _, designated_points, designated_points_voxel = different_areas[array_position]
            self.state.goal_pc.pc = designated_points.pc
            self.state.goal_pc.build_tree()

            # get voxel traversable point cloud
            traversable_points = set()
            traversable_points.update([tuple(position)] + [tuple(pos) for pos in other_positions.values()])
            for _, _, designated_points_voxel_each in different_areas:
                pts = [tuple(v) for v in np.asarray(designated_points_voxel_each.pc.points)]
                traversable_points.update(pts)
            traversable_voxel_pc = PointCloud(points=traversable_points)

            # compute a new plan
            self.planner.update_plan(
                traversable_pc=traversable_voxel_pc,
                goal_pc=self.state.goal_pc,
                position=np.asarray(position)
            )

        # follow the plan
        self.planner.drive()
