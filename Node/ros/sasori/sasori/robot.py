from __future__ import annotations
import numpy as np
from .orchestrator import Orchestrator
from typing import Set
import logging

logger = logging.getLogger("Robot")


class Robot:
    orchestrator: Orchestrator
    robot_id: int
    initial_position: np.ndarray
    current_position: np.ndarray
    other_robots: Set[Robot]

    def __init__(self, position: np.ndarray, orchestrator: Orchestrator):
        self.orchestrator = orchestrator
        self.initial_position = position
        self.current_position = position
        self.other_robots = set()
        logger.debug(f"Robot {self.robot_id} was created ({self})")
        self.orchestrator.state.set_position(tuple(position))
    
    @property
    def robot_id(self):
        return self.orchestrator.state_manager.communication_node.identifier

    @property
    def position(self):
        return self.orchestrator.state.current_position

    def add_other_robot(self, robot: Robot):
        self.other_robots.add(robot)
        logger.debug(f"Robot {self.robot_id} knows {robot.robot_id}")

    def run(self):
        
        logger.info(f"Robot {self.robot_id} running with {len(self.orchestrator.state.traversable_pc.pc.points)} points")

        # update state
        self.orchestrator.state_manager.communication_node.apply_new_state(self.other_robots)
        self.orchestrator.state_manager.update([other_robot.orchestrator.state for other_robot in self.other_robots])

        # update path plan
        self.orchestrator.update(
            position=np.asarray(self.position),
            other_positions={robot.robot_id: np.asarray(robot.position) for robot in self.other_robots}
        )
