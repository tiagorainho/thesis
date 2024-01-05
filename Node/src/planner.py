from typing import List, Tuple
import logging
import numpy as np
from state import State
from cpp_solver import CPPSolver
from segment import Segment
from point_cloud import PointCloud, PointColor
from time import perf_counter

logger = logging.getLogger("Planner")

class Planner:

    solver: CPPSolver
    plan: Segment
    
    def __init__(self, cpp_solver: CPPSolver) -> None:
        self.solver = cpp_solver
        self.plan = Segment([])

        logger.info(msg=f"{self.solver.__class__.__name__} initialized")

    def update_plan(self, traversable_pc: PointCloud, goal_pc: PointCloud, position: np.ndarray):

        # down sample graphs
        traversable_down_sampled = PointCloud(points=np.asarray(traversable_pc.pc.points)).pc  # traversable_pc.down_sample()
        pts = np.asarray(traversable_down_sampled.points)
        np.around(pts, decimals=3, out=pts)

        traversable_down_sampled_pc = PointCloud(point_cloud=traversable_down_sampled, color=PointColor.TRAVERSABLE)

        # check points close to edges/inaccessibles and add them to the downsample
        # this is useful to use low computational power in large spaces and increase precision in closed spaces
        # IMPLEMENTAÇÃO
        # percorrer as blocked, e fazer 2 neighbor searches para 2 raios diferentes e passar pelos pontos traversable
        # verdadeiros, caso o ponto traversable exista no raio maior e nao no menor, adicionar esse ponto aos pontos
        # downsampled

        start_time = perf_counter()

        # use CPP solver to plan a new path segment
        new_plan = self.solver.plan(
            traversable_pc=traversable_down_sampled_pc,
            goal_pc=goal_pc,
            start_point=position)

        logging_message = (f"[PLAN] {len(new_plan)} steps,"
            f" repeated {len(new_plan)-len(set([tuple(v) for v in new_plan]))}"
            f": {round(perf_counter()-start_time, 3)} secs")

        if len(new_plan) <= 1:
            logger.error(f"{logging_message}: TOO SHORT")
        else:
            logger.info(logging_message)

        self.plan.path = new_plan
        pass

    def drive(self):
        pass

    def verify(self, state):

        segment_pairs = self.plan.segment_pairs()
        for position1, position2 in segment_pairs:
            if not self.can_move(state=state, start=position1, goal=position2):
                return False

        return len(segment_pairs) != 0

    def can_move(self, state: State, start, goal):
        path = state.traversable_pc.get_path(
            start=start,
            goal=goal,
            max_distance_to_goal=self.solver.robot_width,
            max_variance=5
        )
        return path is not None
