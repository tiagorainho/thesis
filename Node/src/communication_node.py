from __future__ import annotations
import logging
import numpy as np
from typing import List, Set
from point_cloud import PointCloud


logger = logging.getLogger("Orchestrator")


class StatePayload:
    identifier: int
    point_cloud: PointCloud

    def __init__(self, identifier: int, point_cloud: PointCloud):
        self.identifier = identifier
        self.point_cloud = point_cloud

    def __repr__(self):
        return f"Payload {self.identifier}: {self.point_cloud}"


class CommunicationNode:
    identifier: int
    queue: List[StatePayload]

    def __init__(self, identifier: int):
        self.identifier = identifier
        self.queue = list()

    def recv(self, payload: StatePayload):
        self.queue.append(payload)

    def apply_new_state(self, other_robots: Set):

        logger.info(f"Applying {len(self.queue)} new states")

        robot_states = {robot.robot_id: robot.orchestrator.state for robot in other_robots}

        while self.queue:
            payload: StatePayload = self.queue.pop(0)
            robot_states[payload.identifier].traversable_pc.set(point_cloud=payload.point_cloud)
