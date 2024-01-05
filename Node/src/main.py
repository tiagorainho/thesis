import numpy as np
import open3d as o3d
from cpp_inward_spiral import InwardSpiralSolver
from planner import Planner
from state import State
from state_manager import StateManager
from communication_node import CommunicationNode
from orchestrator import Orchestrator
from robot import Robot
import os

ROBOT_ID=1
ROBOT_WIDTH=0.5
NUMBER_OF_ANGLES=4
REMAINING_THRESHOLD=1
"""
ROBOT_ID: int = os.environ['ROBOT_ID']
ROBOT_WIDTH: float = os.environ['ROBOT_WIDTH']
NUMBER_OF_ANGLES: int = os.environ['NUMBER_OF_ANGLES']
REMAINING_THRESHOLD: float = os.environ['REMAINING_THRESHOLD']
"""

# for now
MAP_SIZE = (5, 5, 1)
MAP_POINT_PER_METER = 25
NUMBER_POINTS = 2000
starting_position = (0, 0, 0)

def random_map(delta_x: float, delta_y: float, delta_z: float, n: float = None) -> np.ndarray:

    if n is None:
        n = MAP_POINT_PER_METER * delta_x * delta_y

    return np.random.uniform([-delta_x/2, -delta_y/2, -delta_z/2], [delta_x/2, delta_y/2, delta_z/2], size=(n, 3))

def map_mountain(delta_x: float, delta_y: float, delta_z: float, n: float = None) -> np.ndarray:
    points = random_map(delta_x=delta_x, delta_y=delta_y, delta_z=delta_z, n=n)

    mountains = [
        (1, 1, 4, 2),
        (-5, -5, 2, 3),
        (-3, 2, 3, 2),
        (6, 4, 3, 3)
    ]

    for p in points:
        x, y, _ = p
        z = 0

        for mountain_x, mountain_y, mountain_z, mountain_delta_z in mountains:
            new_x = x + mountain_x
            new_y = y + mountain_y
            x = new_x
            y = new_y
            # z += mountain_z / (new_x*new_x + new_x*new_y + new_y*new_y + mountain_delta_z)

            z += -(0.01 * (x-1) * (x-1) + 0.01 * (y-1) * (y-1))
        p[2] = z

    return points

if __name__ == '__main__':
    
    print("começar")
    # instantiate main components
    robot_solver = InwardSpiralSolver(
        robot_width=ROBOT_WIDTH,
        n_angles=NUMBER_OF_ANGLES,
        remaining_threshold=REMAINING_THRESHOLD)
    robot_planner = Planner(cpp_solver=robot_solver)
    robot_state = State()

    traversable_points = random_map(delta_x=MAP_SIZE[0], delta_y=MAP_SIZE[1], delta_z=MAP_SIZE[2], n=NUMBER_POINTS)

    robot_state.traversable_pc.extend(traversable_points)

    robot_state.inaccessible_pc.extend(np.asarray([]))
    robot_orchestrator = Orchestrator(
        identifier=ROBOT_ID,
        planner=robot_planner,
        state_manager=StateManager(
                state=robot_state, 
                communication_node=CommunicationNode(identifier=ROBOT_ID)
        )
    )

    # create the robot
    new_robot = Robot(robot_id=ROBOT_ID, position=np.asarray(starting_position), orchestrator=robot_orchestrator)

    new_robot.run()

    print(new_robot.orchestrator.planner.plan.path)

    print("acabar")

