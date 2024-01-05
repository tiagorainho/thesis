from time import sleep
from typing import List
import cProfile
import pstats
import io
import logging
import os
import atexit
import open3d as o3d
import numpy as np
from random import seed
from threading import Thread

from point_cloud import PointCloud, PointColor
from orchestrator import Orchestrator
from planner import Planner
from state import State
from cpp_inward_spiral import InwardSpiralSolver
from robot import Robot
from state_manager import StateManager
from communication_node import CommunicationNode, StatePayload

# set logging
logger = logging.getLogger("Main")
logging.basicConfig(level=logging.INFO, format="[%(name)s] %(levelname)s - %(message)s")
logging.addLevelName(logging.DEBUG, "\x1b[38;20m%s\033[1;0m" % logging.getLevelName(logging.DEBUG))
logging.addLevelName(logging.INFO, "\x1b[94m%s\033[1;0m" % logging.getLevelName(logging.INFO))
logging.addLevelName(logging.WARNING, "\x1b[33;20m%s\033[1;0m" % logging.getLevelName(logging.WARNING))
logging.addLevelName(logging.ERROR, "\x1b[31;1m%s\033[1;0m" % logging.getLevelName(logging.ERROR))
logging.addLevelName(logging.CRITICAL, "\033[1;41m%s\033[1;0m" % logging.getLevelName(logging.CRITICAL))

# set default random seed
RANDOM_SEED = 100
np.random.seed(RANDOM_SEED)
seed(RANDOM_SEED)

# set visualization settings
FPS = 60
WINDOW_HEIGHT = 900
WINDOW_WIDTH = 1200

# set default map configuration
MAP_SIZE = (25, 25, 0.001)
# MAP_SIZE = (80, 20, 0.00005)
MAP_POINT_PER_METER = 45
NUMBER_POINTS = MAP_POINT_PER_METER * MAP_SIZE[0] * MAP_SIZE[1]

# set robot configuration
robot_width = 0.5
NUMBER_OF_ANGLES = 4
REMAINING_THRESHOLD = 1  # 0.98

# set simulation configurations
ADD_POINTS: bool = False


STARTING_ROBOT_COORDINATES = [
    # (0, 0, 0),
    # (0, 0, 0)
    (-MAP_SIZE[0]/2, -MAP_SIZE[1]/2, 0.),
    (-MAP_SIZE[0]/2 + 1.5, -MAP_SIZE[1]/2, 0.),
]
"""
STARTING_ROBOT_COORDINATES = [
    (-40, -10, 0),
    (-40, -10, 0)
]
"""

ALGORITHM_DELTA_T = 1
SHOW_DOWN_SAMPLE: bool = False
MULTI_THREADING: bool = True
POINT_SIZE = 2
SHOW_CENTROID: bool = True
ROBOT_INDEX_TO_VISUALIZE = 0

# set auxiliary variables
DEBUG: bool = False
DEBUG_CPROFILE_FILENAME: str = "performance.txt"
robot_working: bool = False
CAPTURE_SCREEN: bool = False
SCREEN_CAPTURE_DIR = "screen_captures"
ARROW_KEY_CODES = {  # https://www.glfw.org/docs/3.3/group__keys.html
    "RIGHT": 262,
    "LEFT": 263,
    "DOWN": 264,
    "UP": 265
}

i = 0


def run_robots(robots_to_run: List[Robot]):
    global robot_working
    global robot_to_follow

    while robot_running:

        if ADD_POINTS:
            add_points_incrementally(robots[0])

        for r in robots_to_run:

            if not robot_running:
                break

            robot_working = True
            r.run()
            robot_working = False

        # add random points
        """
        robot_to_follow.orchestrator.state.traversable_pc.extend(
            random_map(MAP_SIZE[0], MAP_SIZE[1], 3, 500)
        )
        print(len(np.asarray(robot_to_follow.orchestrator.state.traversable_pc.pc.points)))
        """

        sleep(ALGORITHM_DELTA_T)


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
            z += mountain_z / (new_x*new_x + new_x*new_y + new_y*new_y + mountain_delta_z)

            # z += -(0.01 * (x-1) * (x-1) + 0.01 * (y-1) * (y-1))
        p[2] = z

    return points


def exit_handler():
    global robot_running
    robot_running = False
    logger.warning("Switching Off")
    while robot_working:
        sleep(0.1)
    logger.warning("Turned Off")


def exit_cprofile():
    if not DEBUG:
        return

    pr.disable()

    s = io.StringIO()
    ps = pstats.Stats(pr, stream=s).sort_stats('tottime')
    ps.print_stats()

    pr.print_stats(sort=True)

    with open(DEBUG_CPROFILE_FILENAME, 'w+') as f:
        f.write(s.getvalue())


def update_robot(direction: int):
    global robot_to_follow
    global robots
    global ROBOT_INDEX_TO_VISUALIZE

    ROBOT_INDEX_TO_VISUALIZE = (ROBOT_INDEX_TO_VISUALIZE + direction) % len(robots)
    robot_to_follow = robots[ROBOT_INDEX_TO_VISUALIZE]
    logger.info(f"Following robot {ROBOT_INDEX_TO_VISUALIZE}: {robot_to_follow}")


def update_visualizer():
    global traversable_pc
    global inaccessible_pc
    global robot_work_area_pc
    global robot_to_follow

    # add plan
    if len(robot_to_follow.orchestrator.planner.plan.path) >= 2:
        PointCloud(points=robot_to_follow.orchestrator.planner.plan.path).set_line_set(
            line_set=line_set,
            color=PointColor.NEUTRAL)

    # show down sampled version
    if SHOW_DOWN_SAMPLE:

        # update down sampled traversable point cloud
        possibly_new_traversable_pc = robot_to_follow.orchestrator.state.traversable_pc.down_sample()
        if traversable_pc.pc != possibly_new_traversable_pc and len(possibly_new_traversable_pc.points) > 0:
            traversable_pc.set(PointCloud(point_cloud=possibly_new_traversable_pc, color=PointColor.TRAVERSABLE))

        # update down sampled inaccessible point cloud
        possibly_new_inaccessible_pc = robot_to_follow.orchestrator.state.inaccessible_pc.down_sample()
        if inaccessible_pc.pc != possibly_new_inaccessible_pc and len(possibly_new_inaccessible_pc.points) > 0:
            inaccessible_pc.set(PointCloud(point_cloud=possibly_new_inaccessible_pc, color=PointColor.BLOCKED))
    else:
        # update real point clouds
        if len(robot_to_follow.orchestrator.state.traversable_pc.pc.points) > 0:
            traversable_pc.set(robot_to_follow.orchestrator.state.traversable_pc)

        if len(robot_to_follow.orchestrator.state.inaccessible_pc.pc.points) > 0:
            inaccessible_pc.set(robot_to_follow.orchestrator.state.inaccessible_pc)

    # update working area
    if len(robot_to_follow.orchestrator.state.goal_pc.pc.points) > 0:
        robot_work_area_pc.set(robot_to_follow.orchestrator.state.goal_pc)

    # update geometries
    vis.update_geometry(line_set)
    vis.update_geometry(inaccessible_pc.pc)
    vis.update_geometry(traversable_pc.pc)
    vis.update_geometry(robot_work_area_pc.pc)

    # improve visualization fluency
    if MULTI_THREADING:
        vis.poll_events()
        vis.update_renderer()
    else:
        for _ in range(FPS):
            vis.poll_events()
            vis.update_renderer()
            sleep(1 / FPS)

    # capture screen
    if CAPTURE_SCREEN:
        if not os.path.exists(SCREEN_CAPTURE_DIR):
            os.makedirs(SCREEN_CAPTURE_DIR)
        vis.capture_screen_image(f"{SCREEN_CAPTURE_DIR}/capture.png", do_render=True)


def add_points_incrementally(r: Robot):
    global i
    i += 1

    new_points = map_mountain(
        delta_x=MAP_SIZE[0],  # * 1.5,  # * (1 + (i / 10)),
        delta_y=MAP_SIZE[1],  #  * 1.5,  # * (1 + (i / 10)),
        delta_z=MAP_SIZE[2],  # * (1 + i / 20),
        n=2000
    )

    new_state = PointCloud(point_cloud=r.orchestrator.state.traversable_pc.pc)
    new_state.extend(new_points)

    r.orchestrator.state.traversable_pc.extend(new_points)

    # simulate state updates
    for r2 in robots:
        if r is r2:
            continue

        r2.orchestrator.state_manager.communication_node.recv(
            StatePayload(identifier=r.robot_id, point_cloud=new_state)
        )


if __name__ == '__main__':
    robot_running = True
    atexit.register(exit_handler)
    if DEBUG:
        atexit.register(exit_cprofile)
        pr = cProfile.Profile()
        pr.enable()

    # generate a initial state
    initial_state = State()

    # create the map
    traversable_points = map_mountain(delta_x=MAP_SIZE[0], delta_y=MAP_SIZE[1], delta_z=MAP_SIZE[2], n=NUMBER_POINTS)
    logger.info(f"Added {len(traversable_points)} points to the map of dimensions {MAP_SIZE}")

    print(traversable_points)

    # add map to state
    initial_state.traversable_pc.extend(traversable_points)
    initial_state.inaccessible_pc.extend(np.asarray([]))

    # instantiate the robots
    robots: List[Robot] = list()

    for robot_id, starting_coordinate in enumerate(STARTING_ROBOT_COORDINATES):

        # estimate starting position
        starting_position = initial_state.traversable_pc.estimate_smooth_closest_point(
            point=starting_coordinate,
            radius=robot_width)

        # instantiate main components
        robot_solver = InwardSpiralSolver(
            robot_width=robot_width,
            n_angles=NUMBER_OF_ANGLES,
            remaining_threshold=REMAINING_THRESHOLD)
        robot_planner = Planner(cpp_solver=robot_solver)
        robot_state = State()

        if robot_id > 0:
            traversable_points = [(p[0]-3, p[1]+4, p[2]+1) for p in traversable_points]

        robot_state.traversable_pc.extend(traversable_points)

        robot_state.inaccessible_pc.extend(np.asarray([]))
        robot_orchestrator = Orchestrator(
            identifier=robot_id,
            planner=robot_planner,
            state_manager=StateManager(state=robot_state, communication_node=CommunicationNode(identifier=robot_id))
        )

        # create the robot
        new_robot = Robot(robot_id=robot_id, position=np.asarray(starting_position), orchestrator=robot_orchestrator)

        # add the robot to the collection
        robots.append(new_robot)

    # test
    # robots[0].orchestrator.state_manager.update(robots[1].orchestrator.state)
    # o3d.visualization.draw_geometries([robots[0].orchestrator.state.traversable_pc.pc])

    # share robot initial positions
    for robot1 in robots:
        for robot2 in robots:
            if robot1 is robot2:
                continue
            robot1.add_other_robot(robot2)

    # define robot to follow in the visualization
    robot_to_follow = robots[ROBOT_INDEX_TO_VISUALIZE]

    # create visualizer
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(height=WINDOW_HEIGHT, width=WINDOW_WIDTH)

    # instantiate traversable point cloud to visualize
    traversable_pc = PointCloud(color=PointColor.TRAVERSABLE)
    traversable_pc.set(robot_to_follow.orchestrator.state.traversable_pc)

    # instantiate traversable point cloud to visualize
    inaccessible_pc = PointCloud(color=PointColor.BLOCKED)
    inaccessible_pc.set(robot_to_follow.orchestrator.state.inaccessible_pc)

    # instantiate working area to visualize
    robot_work_area_pc = PointCloud(color=PointColor.GOAL)
    robot_work_area_pc.set(robot_to_follow.orchestrator.state.goal_pc)

    # instantiate path plan line set to visualize
    line_set: o3d.geometry.LineSet = o3d.geometry.LineSet()

    # create centroid to pinpoint the starting point
    if SHOW_CENTROID:
        for robot in robots:
            centroid: o3d.geometry.TriangleMesh = o3d.geometry.TriangleMesh.create_sphere(radius=robot_width)
            centroid.translate(robot.initial_position, relative=False)
            centroid.paint_uniform_color([1, 0.5, 0.5])

            # add centroid to visualizer
            vis.add_geometry(centroid)

    # add point clouds to visualizer
    vis.add_geometry(line_set)
    vis.add_geometry(traversable_pc.pc)
    vis.add_geometry(inaccessible_pc.pc)
    vis.add_geometry(robot_work_area_pc.pc)

    # set render options
    if POINT_SIZE is not None:
        render_options: o3d.visualization.RenderOption = vis.get_render_option()
        render_options.point_size = POINT_SIZE

    # add key callbacks
    vis.register_key_callback(ARROW_KEY_CODES["RIGHT"], lambda _: update_robot(1))
    vis.register_key_callback(ARROW_KEY_CODES["LEFT"], lambda _: update_robot(-1))

    print("Use arrow left (<-) and arrow right (->) to update robot to follow")

    if MULTI_THREADING:
        Thread(target=run_robots, args=(robots,)).start()

    while True:
        if not MULTI_THREADING:

            if ADD_POINTS:
                add_points_incrementally(robots[0])

            for robot in robots:
                robot.run()

        """
        # expand the map incrementally
        
        new_traversable_points = map_mountain(delta_x=MAP_SIZE[0]*(1+(i/10)), delta_y=MAP_SIZE[1]*(1+(i/10)),
                                              delta_z=MAP_SIZE[2]*(1+i/20), n=400)
        i += 1
        for robot in robots:
            robot.orchestrator.state.traversable_pc.extend(new_traversable_points)

        """

        update_visualizer()
