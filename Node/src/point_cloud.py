from __future__ import annotations
import logging
from typing import Set, Tuple, List, Union
from enum import Enum
import open3d as o3d
import numpy as np
from graph import Graph
from queue import PriorityQueue
from time import perf_counter
from math import dist
from random import random

logger = logging.getLogger("Point Cloud")

MIN_COVERAGE_PERCENTAGE = 0.95
VOXEL_SIZE = 0.25
NUMBER_DIRECTIONS = 6
DEFAULT_MAX_PERCENTAGE_COVERED_NEIGHBORS = 0.6


class PointColor(Enum):
    TRAVERSABLE = [0, 1, 1]
    BLOCKED = [1, 0, 0]
    NEUTRAL = [0, 0, 0]
    GOAL = [0, 0, 1]
    UPDATE = [1, 0.5, 0]


class PointCloud:

    pc: o3d.geometry.PointCloud
    pc_tree: o3d.geometry.PointCloud
    color: PointColor

    def __init__(
            self,
            points: Union[List, Set, np.ndarray, None] = None,
            point_cloud: Union[o3d.geometry.PointCloud, None] = None,
            file: str = None,
            color: PointColor = PointColor.NEUTRAL
    ):

        self.pc = o3d.geometry.PointCloud()
        self.color = color

        if points is not None:
            self.pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        elif point_cloud is not None:
            self.pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(point_cloud.points)))
        elif file is not None:
            # open file
            pass

        self.pc.paint_uniform_color(color=self.color.value)

        if len(self.pc.points) > 0:
            self.pc_tree = o3d.geometry.KDTreeFlann(self.pc)

    def estimate_smooth_closest_point(
            self,
            point: Tuple[float, float, float],
            radius: float = None
    ) -> Union[Tuple[float, float, float], None]:

        closest_point = self.closest_neighbor(point=point)
        if closest_point is None:
            return None

        if radius is None:
            return self.closest_neighbor(point=point)

        neighbors = self.neighbors(point=closest_point, radius=radius)
        return self.estimate_smooth_point(point=tuple(closest_point), radius=radius, neighbors=neighbors)

    def estimate_smooth_point(
            self,
            point: Tuple[float, float, float],
            radius=None,
            neighbors=None
    ) -> Union[Tuple[float, float, float], None]:

        if neighbors is None:
            neighbors = self.neighbors(point=point, radius=radius)

        if len(neighbors) == 0:
            return None

        return point[0], point[1], np.average(neighbors, axis=0)[2]

    def down_sample(self, voxel_size: float = VOXEL_SIZE) -> o3d.geometry.PointCloud:
        # return self.pc.uniform_down_sample(4)
        # return self.pc.random_down_sample(sampling_ratio=0.4)
        return self.pc.voxel_down_sample(voxel_size)

    def build_tree(self):
        self.pc_tree = o3d.geometry.KDTreeFlann(self.pc)

    def extend(self, points: np.ndarray):
        self.pc.points.extend(points)
        self.refresh()

    def refresh(self):
        self.pc.remove_duplicated_points()
        self.pc.paint_uniform_color(color=self.color.value)
        self.build_tree()

    def set(self, point_cloud: PointCloud = None, points: np.ndarray = None):
        if point_cloud is not None:
            self.pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(point_cloud.pc.points)))
        elif points is not None:
            self.pc.points = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        else:
            return
        self.refresh()

    def neighbors(self, point, radius: float, include: bool = True) -> np.ndarray:
        """
        :param point:
        :param radius:
        :param include:
        :return: N3x of points by order of distance within a maximum radius from the specified focal point
        """
        _, idx, _ = self.pc_tree.search_radius_vector_3d(query=point, radius=radius)

        neighbors_array = np.asarray(self.pc.points)[idx, :]
        if not include:
            if len(neighbors_array) > 0:
                return neighbors_array[1:]

        return neighbors_array

    def closest_neighbor(self, point, max_radius=None) -> Union[np.ndarray, None]:
        if max_radius is None:
            _, idx, _ = self.pc_tree.search_knn_vector_3d(query=point, knn=1)
        else:
            _, idx, _ = self.pc_tree.search_hybrid_vector_3d(query=point, radius=max_radius, max_nn=1)

        closest_neighbor_point = np.asarray(self.pc.points)[idx, :]

        # it is required to check the length because the closest neighbor is returned as an array
        if len(closest_neighbor_point) == 0:
            return None

        return closest_neighbor_point[0]

    def set_line_set(self, line_set: o3d.geometry.LineSet, color: PointColor = PointColor.NEUTRAL) -> None:
        # set points
        point_array = np.asarray(self.pc.points)
        line_set.points = o3d.utility.Vector3dVector(point_array)

        # set line
        range_points = range(len(point_array))
        segmented_lines = [v for v in zip(range_points, range_points[1:])]
        line_set.lines = o3d.utility.Vector2iVector(segmented_lines)

        # set color
        colors = [color.value for _ in range(len(segmented_lines))]
        line_set.colors = o3d.utility.Vector3dVector(colors)

    def get_path(self, start, goal, max_distance_to_goal: float = 1, max_variance=None):
        if max_variance is not None:
            max_distance = max_variance * max_distance_to_goal

        return Graph.generic_a_star(
            start=start,
            goal=goal,
            check_done=lambda node: node.distance_to_goal <= max_distance_to_goal,
            ignore_condition=(lambda node: node.distance_to_goal > max_distance) if max_variance is not None else
            (lambda node: False),
            get_neighbors=lambda node: self.neighbors(point=node.value, radius=max_distance_to_goal)
        )

    def find_path_to_closest_goal(
            self,
            start: Tuple[float, float, float],
            goal: Set[Tuple[float, float, float]],
            ignore: Set[Tuple[float, float, float]],
            max_neighbor_distance: float
    ):
        start_tuple = tuple(start)
        return Graph.generic_bfs(
            start=start,
            check_done=lambda node: node.value in goal and node.value not in ignore and node.value != start_tuple,
            get_neighbors=lambda node: self.neighbors(point=node.value, radius=max_neighbor_distance)
        )

    def next_pos(self, position: Tuple[float, float, float], radius: float):
        directions = PointCloud.circular_positions(NUMBER_DIRECTIONS, radius, 0)
        next_approximated_positions = position + directions

        estimated_values = []
        for pos in next_approximated_positions:

            # get smooth point
            new_position = self.estimate_smooth_point(
                point=tuple(pos),
                radius=radius,
                neighbors=self.neighbors(point=pos, radius=radius)
            )

            if new_position is None:
                continue

            # normalize vector
            # NOT NECESSARY SINCE THE NORMALIZATION IS ONLY FROM 1 POINT TO ANOTHER,
            # THEREFORE ITS BETTER TO USE A THRESHOLD TO CHECK FOR LARGER DISTANCES
            """
            vector = np.asarray([
                position[0] - new_position[0],
                position[1] - new_position[1],
                position[2] - new_position[2]
            ])
            normalized_vector = (vector / np.sqrt(np.sum(vector**2))) * radius * 0.5
            new_point = np.asarray(position) + normalized_vector
            new_point = np.round(new_point, decimals=2)
            estimated_values.append(tuple(new_point))
            """

            estimated_values.append(new_position)

        return estimated_values

    @staticmethod
    def circular_positions(n_angles: int, step_size: float, angle_offset: float = 0):
        points = []

        for direction_idx in range(n_angles):
            angle_variation = direction_idx / n_angles * np.pi * 2 + angle_offset

            x = np.cos(angle_variation) * step_size
            y = np.sin(angle_variation) * step_size
            points.append(np.array([x, y, 0]))

        return np.asarray(points)

    def divide_areas(
            self,
            starting_points: List[Tuple[float, float, float]],
            max_neighbor_distance: float,
            limit: int = 100000,
            min_coverage_percentage: float = MIN_COVERAGE_PERCENTAGE
            # fetch_next_points: lambda point: List[np.ndarray],
    ) -> List[Tuple[Tuple[float, float, float], PointCloud, PointCloud]]:
        """
        :param starting_points:
        :param max_neighbor_distance:
        :param limit:
        :param min_coverage_percentage:
        :return: starting point, covered point cloud, area point cloud
        """

        start_time = perf_counter()

        fetch_next_points = self.next_pos

        areas: List[RobotArea] = list()

        # instantiate starting points in the robot area
        starting_points.sort(key=lambda p: dist((0, 0, 0), p))
        for point in starting_points:
            new_area = RobotArea(starting_point=point)
            areas.append(new_area)

        logger.info(f"[{PointCloud.divide_areas.__name__} -> {self.pc}] starting points: {np.asarray(starting_points)}")

        remaining_areas: List[RobotArea] = [area for area in areas]

        i = 0
        while remaining_areas and i < limit:
            i += 1

            # if sum([len(area.path) for area in areas]) >= 39:
            #     break

            current_area = min(remaining_areas, key=lambda area: len(area.path))

            # if there are no more points to search for in that area
            if current_area.queue.empty():

                logger.info(f"Area is Empty: {current_area} ... checking new sub-regions")

                # use bfs to search for the next closest point to search

                # for that, get points not covered yet
                untouched_points = set([tuple(p) for p in np.asarray(self.pc.points)])
                for robot_area in areas:
                    untouched_points.difference_update(robot_area.points)

                # check if it is worth to reach other available points
                current_coverage_percentage = 1-len(untouched_points)/len(self.pc.points)

                if current_coverage_percentage >= min_coverage_percentage:
                    logger.warning(f"Area was not be further expanded since "
                                   f"{current_coverage_percentage*100}% of work was already done: {current_area}")

                    # remove the current area
                    remaining_areas.remove(current_area)

                    continue

                def check_new_uncovered(value):

                    if value not in untouched_points:
                        return False

                    if value == current_area.starting_point:
                        return False

                    if not current_area.check_if_can_add(
                            new_point=value,
                            neighbor_points=[value],
                            other_areas=[]
                    ):
                        untouched_points.difference_update([value])
                        return False

                    return True

                # get path to the closest point which others have not worked on
                path_to_closest_not_covered = Graph.generic_bfs(
                    start=current_area.starting_point,
                    check_done=lambda node: check_new_uncovered(node.value) and node.value not in current_area.blocked,
                    get_neighbors=lambda node: self.neighbors(point=node.value, radius=max_neighbor_distance),
                    # ignore_condition=lambda node, closed_nodes: node in closed_nodes
                )
                # VER AQUI EM CIMA IGNORE CONDITION

                # if no more areas are available
                if path_to_closest_not_covered is None:
                    logger.warning(f"Area could not be further expanded since "
                                   f"it cant reach an uncovered position: {current_area}")

                    # remove the current area
                    remaining_areas.remove(current_area)

                    continue

                logger.info(f"Area expanded to {path_to_closest_not_covered[-1]}: {current_area}")

                current_area.queue.put((0, tuple(path_to_closest_not_covered[-1])))

            # get the next position of the area
            weight, current_position = current_area.queue.get()

            neighbors: List[Tuple[float, float, float]] = [tuple(v) for v in self.neighbors(
                point=current_position,
                radius=max_neighbor_distance*1.2)]  # meter isto melhor

            # ignore new point if it does NOT meet the requirements
            other_areas = [other_area for other_area in areas if other_area != self]
            if not current_area.check_if_can_add(
                    new_point=current_position,
                    neighbor_points=neighbors,
                    other_areas=other_areas
            ):
                current_area.blocked.add(current_position)
                continue

            current_area.register_point(current_position, neighbors)

            # calculate the next possible positions
            new_points = fetch_next_points(position=current_position, radius=max_neighbor_distance*2)

            for point in new_points:

                weight = len(current_area.path)
                # n_points = len(self.neighbors(point=point, radius=max_neighbor_distance))
                # if n_points > 0:
                #    weight = 1/n_points

                if point not in current_area.points:
                    current_area.queue.put((weight, point))

        logger.info(f"[{PointCloud.divide_areas.__name__}] computation time: {perf_counter() - start_time} s")

        """
        pcs = [PointCloud(points=area.path).pc for area in areas]
        pcs[0].paint_uniform_color([81 / 255, 196 / 255, 219 / 255])
        pcs[1].paint_uniform_color([235 / 255, 195 / 255, 94 / 255])
        print(pcs)
        o3d.visualization.draw_geometries(pcs)
        """

        logger.info(f"[{PointCloud.divide_areas.__name__}] Areas information: {areas}")
        return [(area.starting_point, PointCloud(points=area.points), PointCloud(points=area.path)) for area in areas]

    def __repr__(self):
        return str(self.pc)


class RobotArea:
    starting_point: Tuple[float, float, float]
    points: Set[Tuple[float, float, float]]
    blocked: Set[Tuple[float, float, float]]
    path: Set[Tuple[float, float, float]]
    queue: PriorityQueue[Tuple[float, Tuple[float, float, float]]]

    def register_point(self, point: Tuple[float, float, float], neighbors: List[Tuple[float, float, float]]):

        # add neighbors converted to tuples
        self.points.update(neighbors)

        # add point to path
        self.path.add(point)

    def __init__(self, starting_point: Tuple[float, float, float]):
        self.starting_point = starting_point
        self.points = set()
        self.path = set()
        self.blocked = set()
        self.queue = PriorityQueue()
        self.queue.put((0, starting_point))

    def __lt__(self, other: RobotArea):
        return len(self.path) < len(other.path)

    def __repr__(self):
        priority = self.queue.queue[0][0] if not self.queue.empty() else None

        return str({
            "starting point": self.starting_point,
            "path": len(self.path),
            "points": len(self.points),
            "blocked": len(self.blocked),
            "queue": self.queue.qsize(),
            "priority": priority
        })

    @staticmethod
    def check_completion(
            neighbor_points: List[Tuple[float, float, float]],
            other_areas: List[RobotArea],
            max_percentage_of_completion: float = DEFAULT_MAX_PERCENTAGE_COVERED_NEIGHBORS
    ):
        if len(other_areas) == 0:
            return True

        counter = 0
        for neighbor in neighbor_points:
            for other_area in other_areas:
                if neighbor in other_area.points:
                    counter += 1
                    break

        occupied_percentage = float(counter) / len(neighbor_points)

        # ignore points in which more than 50% of the neighbors are already covered by other areas
        return occupied_percentage < max_percentage_of_completion

    def check_if_can_add(
            self,
            new_point: Tuple[float, float, float],
            neighbor_points: List[Tuple[float, float, float]],
            other_areas: List[RobotArea]
    ):

        if len(neighbor_points) == 0:
            return False

        # ignore point if it is already added
        if new_point in self.path:
            return False

        return self.check_completion(
            neighbor_points=neighbor_points,
            other_areas=other_areas
        )


def test():

    visualize_test = True
    colors: Union[None, List[Tuple[float, float, float]]] = [
        (81/255, 196/255, 219/255),
        (235/255, 195/255, 94/255),
        (10 / 255, 100 / 255, 150 / 255),
        (150 / 255, 50 / 255, 200 / 255),
        (100 / 255, 30 / 255, 255 / 255),
        (75 / 255, 250 / 255, 90 / 255),
    ]

    tests = [
        ((80, 20), [(-40, -10, 0), (-40, -9, 0)]),
        ((100, 50), [(-10, -10, 0), (10, 10, 0)]),
        ((40, 40), [(-10, -10, 0), (10, 10, 0)]),
        ((14, 10), [(-7, -5, 0), (7, 5, 0)]),
        ((10, 60), [(5, -30, 0), (1, 1, 0)]),
        ((20, 10), [(0, 0, 0), (1, 1, 0)]),
        ((20, 20), [(0, 0, 0), (1, 1, 0)]),
        ((20, 30), [(0, 0, 0), (1, 1, 0)]),
        ((20, 20), [(0, 0, 0), (1, 1, 0), (20, 20, 20)]),
        ((20, 20), [(0, 0, 0), (1, 1, 0), (10, 10, 0), (15, 15, 0)]),
        ((20, 20), [(0, 0, 0), (1, 1, 0), (10, 10, 0), (15, 15, 0), (5, 5, 0)]),
    ]

    def random_map(delta_x: float, delta_y: float, delta_z: float, n: float = None) -> np.ndarray:
        if n is None:
            n = 20 * delta_x * delta_y

        return np.random.uniform([-delta_x / 2, -delta_y / 2, -delta_z / 2], [delta_x / 2, delta_y / 2, delta_z / 2],
            size=(n, 3))

    for dimensions, starting_coordinates in tests[:1]:
        np.random.seed(100)
        x, y = dimensions
        points = random_map(delta_x=x, delta_y=y, delta_z=0.00005)
        start_time = perf_counter()
        p = PointCloud(points=points)
        # pr.enable()
        pcs = p.divide_areas(starting_points=starting_coordinates, max_neighbor_distance=0.5)
        # pr.disable()
        logger.info(f"dimensions: {dimensions} -> "
              f"{len(points)} points, with {len(starting_coordinates)} robots: {perf_counter()-start_time}")

        if visualize_test:
            lst = [pc.pc for _, _, pc in pcs]

            for i, pc in enumerate(lst):
                if colors is None:
                    color = (random(), random(), random())
                else:
                    color = colors[i]
                pc.paint_uniform_color(color=color)

            o3d.visualization.draw_geometries(lst)


def exit_cprofile():

    pr.disable()

    s = io.StringIO()
    ps = pstats.Stats(pr, stream=s).sort_stats('tottime')
    ps.print_stats()

    pr.print_stats(sort=True)


if __name__ == '__main__':
    DEBUG_PERFORMANCE = False

    if DEBUG_PERFORMANCE:
        import cProfile
        import pstats
        import io
        import atexit

        atexit.register(exit_cprofile)
        pr = cProfile.Profile()
        pr.enable()

    test()