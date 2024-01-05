from typing import Set, Tuple, List, Generator
import numpy as np
import logging

from cpp_solver import CPPSolver
from point_cloud import PointCloud

logger = logging.getLogger("CPP Inward Spiral")

PRECISION = 3


class InwardSpiralSolver(CPPSolver):

    n_angles: int
    remaining_threshold: float

    def __init__(self, robot_width: float, n_angles: int = 6, remaining_threshold: float = 1):
        """
        :param robot_width: width of the circular area the robot is capable of operating
        :param n_angles: number of angles equally distributed in 360 degrees. Will be used to get the new neighbors
        using inward spiral
        :param remaining_threshold: percentage of which if new direct neighbors are not found, the path is
        finalized instead of finding new uncovered points
        """
        super().__init__(robot_width=robot_width)
        self.n_angles = n_angles
        self.remaining_threshold = remaining_threshold

    def plan(self,
             traversable_pc: PointCloud,
             goal_pc: PointCloud,
             start_point: np.ndarray,
             covered_points: Set[Tuple[float]] = None) -> Generator[np.array, any, np.ndarray]:

        pts_ = np.asarray(goal_pc.pc.points)
        np.around(pts_, decimals=PRECISION, out=pts_)

        if covered_points is None:
            covered_points = set()

        # for p_ in np.asarray(traversable_pc.pc.points):
        #     covered_points.add(tuple(p_))

        """
        goal_pnts = set([tuple(v) for v in list(np.asarray(goal_pc.pc.points))])
        print("goal_pnts", goal_pnts)
        for c in covered_points:
            for c2 in goal_pc.neighbors(point=c, radius=0.5):
                # print("eliminado", c2)
                if tuple(c2) in goal_pnts:
                    goal_pnts.remove(tuple(c2))

        goal_pc = PointCloud(points=list(goal_pnts))
        goal_pc.build_tree()
        """

        # print(f"total: {len(traversable_pc.pc.points)}, covered: {len(covered_points)}")

        # print(f"{sorted([tuple(v) for v in list(np.asarray(traversable_pc.pc.points))], key=lambda k: str(k))} \n\n ----\n\n{sorted(list(covered_points), key=lambda k: str(k))}")

        traversable_pc.extend(points=np.asarray(goal_pc.pc.points))
        set_goal_points = set([tuple(p) for p in np.asarray(goal_pc.pc.points)])

        # print("traversable", [tuple(v) for v in list(np.asarray(traversable_pc.pc.points))])

        # initialize path plan
        path: List[Tuple[float, float, float]] = []
        # path_set: Set[Tuple[float, float, float]] = set()

        # define auxiliary functions
        def add_point_to_path(point: Tuple[float, float, float]):
            path.append(point)
            # path_set.add(point)
            extend_covered_points(points=[point])

        def extend_covered_points(points):
            covered_points.update(map(tuple, np.round(points, decimals=3)))

        def calculate_angle(from_pos, to_pos):
            vec = np.asarray(to_pos)[0:2] - np.asarray(from_pos)[0:2]
            return np.angle(vec[0] + vec[1] * 1j)

        def circular_positions(n_angles: int, step_size: float, angle_offset: float = 0):
            points = []

            for direction_idx in range(n_angles):
                angle_variation = direction_idx / n_angles * np.pi * 2 + angle_offset

                x = np.cos(angle_variation) * step_size
                y = np.sin(angle_variation) * step_size
                points.append(np.array([x, y, 0]))

            return np.asarray(points)

        def get_neighbor_inward(current_position, step_size, closed_points, angle_offset=0):

            # order the neighbors points based on the direction
            relative_circular_points = circular_positions(
                n_angles=self.n_angles,
                step_size=step_size,
                angle_offset=angle_offset)
            points = current_position + relative_circular_points

            # get first available point
            pc_aux = PointCloud(points=path)

            for point in points:

                # ignore point if it is too close to other points
                if len(pc_aux.neighbors(point=point, radius=step_size, include=False)) > 0:
                    continue

                # get multiple neighbors
                real_neighbors = goal_pc.neighbors(point=point, radius=step_size, include=False)

                # print(f"------ > {point}: {real_neighbors}")
                # print(covered_points)

                """
                import open3d as o3d
                centroid: o3d.geometry.TriangleMesh = o3d.geometry.TriangleMesh.create_sphere(radius=0.5)
                centroid.translate(current_position, relative=False)
                centroid.paint_uniform_color([1, 0.5, 0.5])

                o3d.visualization.draw_geometries([goal_pc.pc, centroid])
                """

                # ignore if there is no neighbor
                if len(real_neighbors) == 0:
                    continue

                for real_neighbor in real_neighbors:

                    # filter points that have been already been covered
                    if tuple(real_neighbor) in closed_points:
                        continue

                    # print(point)
                    # point[2] = np.average(real_neighbors, axis=0)[2]
                    # print(point)

                    # rounded_point = tuple(np.round(point, decimals=PRECISION))

                    # if rounded_point in closed_points:
                    #     continue

                    return tuple(np.round(point, decimals=3))

            return None

        # add starting point
        add_point_to_path(tuple(start_point))

        # eigenvalue global offset
        # calculate angle using svd
        ps = np.asarray(goal_pc.pc.points)
        U, S, Vh = np.linalg.svd(ps, full_matrices=True)
        most_relevant_vector = Vh[:, 0]
        angle = np.arctan2(most_relevant_vector[1], most_relevant_vector[0])

        print(angle)

        # global angle offset
        angle = 0


        # while there are points that have not been covered
        while True:

            # get last position of the path
            last_position: Tuple[float, float, float] = path[-1]

            yield last_position

            # calculate robot direction

            """
            # relative angle offset
            angle = calculate_angle(
                from_pos=path[-2],
                to_pos=last_position) if len(path) >= 2 else -1 / self.n_angles * np.pi * 2
            """

            # get neighbor points
            # TENHO Q AUMENTAR O STEP SIZE, NO ENTANTO HA PROBLEMAS PQ N VAI ENCONTRAR PONTOS, PARA TAL TENHO Q FAZER
            # UM METODO DE PESQUISA EM Q VOU DO GOAL ATE AO START POINT
            neighbor: Tuple[float, float, float] = get_neighbor_inward(
                current_position=last_position,
                step_size=self.robot_width,
                angle_offset=angle,
                closed_points=covered_points
            )

            # print(f"neighbor: {neighbor}")

            # if there is a reachable close neighbor that was not covered yet
            if neighbor is not None:

                # if neighbor in path:
                #    break

                add_point_to_path(neighbor)

                extend_covered_points(points=goal_pc.neighbors(point=neighbor, radius=self.robot_width))
                continue

            # break if the remaining number of not covered points is not significant
            # logger.error(f"VER ----> {len(goal_pc.pc.points),  len(covered_points), self.remaining_threshold}")
            # if len(goal_pc.pc.points) * self.remaining_threshold < len(covered_points):
            #     break

            # if there are no neighbors, search for the closest point
            # NOTE: because it reached at the current point from a chain of points distanced by a radius,
            # it can also move up the chain

            # import open3d as o3d
            # o3d.visualization.draw_geometries([traversable_pc.pc])

            # pts = set()
            # pts.update(tuple(p) for p in np.asarray(goal_pc.pc.points))
            # pts.update(tuple(p) for p in np.asarray(traversable_pc.pc.points))
            # new_pc = PointCloud(points=pts)

            """
            start_tuple = tuple(last_position)
            path_to_closest_non_covered_point = Graph.generic_bfs(
                start=start_tuple,
                check_done=lambda node: len(goal_pc.neighbors(
                    point=node.value,
                    radius=self.robot_width,
                    include=False)
                ) > 0 and tuple(np.round(node.value, decimals=PRECISION)) not in path_set and node.value != start_tuple,
                get_neighbors=lambda node: traversable_pc.neighbors(point=node.value, radius=self.robot_width)
            )
            """

            path_to_closest_non_covered_point = traversable_pc.find_path_to_closest_goal(
                start=last_position,
                goal=set_goal_points,
                ignore=covered_points,
                max_neighbor_distance=self.robot_width * 1.25  # extra distance to access different heights
            )

            # if no available points are found, it means that the coverage was completed
            if path_to_closest_non_covered_point is None:
                break

            # add uncovered point
            """
            path_to_closest_non_covered_point.append(
                goal_pc.neighbors(
                    point=path_to_closest_non_covered_point[-1],
                    radius=self.robot_width,
                    include=False
                )[0]
            )
            """

            # otherwise, add the path (to the closest available point) to the plan path
            path.extend([tuple(np.round(p, decimals=PRECISION)) for p in path_to_closest_non_covered_point[1:]])
            # path_set.update([tuple(np.round(p, decimals=PRECISION)) for p in path_to_closest_non_covered_point[1:]])

            extend_covered_points(
                points=goal_pc.neighbors(
                    point=path_to_closest_non_covered_point[-1],
                    radius=self.robot_width
                )
            )

        # log statistics
        len_covered = len(covered_points) - len(covered_points.intersection(path))

        len_pc = len(goal_pc.pc.points)
        coverage_percentage = round((len_covered/len_pc)*100, 1)
        if len_covered/len_pc > self.remaining_threshold:
            logger.info(f"Completed plan [{coverage_percentage}%]")
        else:
            logger.error(f"Uncompleted plan [{coverage_percentage}%]")
        logger.debug(f"pc points: {len_pc}, covered points: {len_covered}")

        return np.asarray(path)
