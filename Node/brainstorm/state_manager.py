from typing import List, Tuple, Mapping
import open3d as o3d
import numpy as np
from time import sleep
from point_cloud import PointCloud
from state import State
from communication_node import CommunicationNode

voxel_size = 0.25
print("Full registration ...")
max_correspondence_distance_coarse = voxel_size * 15
max_correspondence_distance_fine = voxel_size * 1.5

VISUALIZE_STATE: bool = False
FPS = 30
TIME_WINDOW = 5


class StateManager:
    state: State
    communication_node: CommunicationNode

    def __init__(self, state: State, communication_node: CommunicationNode):
        self.state = state
        self.state.traversable_pc.pc = self.state.traversable_pc.down_sample()
        self.state.traversable_pc.build_tree()
        self.communication_node = communication_node

    @staticmethod
    def pairwise_registration(source, target):
        print("Apply point-to-plane ICP")
        icp_coarse = o3d.pipelines.registration.registration_icp(
            source, target, max_correspondence_distance_coarse, np.identity(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane())
        icp_fine = o3d.pipelines.registration.registration_icp(
            source, target, max_correspondence_distance_fine,
            icp_coarse.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane())
        transformation_icp = icp_fine.transformation
        information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
            source, target, max_correspondence_distance_fine,
            icp_fine.transformation)
        return transformation_icp, information_icp

    @staticmethod
    def full_registration(pcds):
        pose_graph = o3d.pipelines.registration.PoseGraph()
        odometry = np.identity(4)
        pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
        n_pcds = len(pcds)
        for source_id in range(n_pcds):
            for target_id in range(source_id + 1, n_pcds):
                transformation_icp, information_icp = StateManager.pairwise_registration(
                    pcds[source_id], pcds[target_id])
                print("Build o3d.pipelines.registration.PoseGraph")
                if target_id == source_id + 1:  # odometry case
                    odometry = np.dot(transformation_icp, odometry)
                    pose_graph.nodes.append(
                        o3d.pipelines.registration.PoseGraphNode(
                            np.linalg.inv(odometry)))
                    pose_graph.edges.append(
                        o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                                 target_id,
                                                                 transformation_icp,
                                                                 information_icp,
                                                                 uncertain=False))
                else:  # loop closure case
                    pose_graph.edges.append(
                        o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                                 target_id,
                                                                 transformation_icp,
                                                                 information_icp,
                                                                 uncertain=True))
        return pose_graph

    @staticmethod
    def merge(pcs: List[PointCloud]) -> Tuple[PointCloud, o3d.pipelines.registration.PoseGraph]:
        # http://www.open3d.org/docs/latest/tutorial/Advanced/multiway_registration.html

        pcds_down = [pc.pc for pc in pcs]

        voxel_size = 0.25
        max_correspondence_distance_coarse = voxel_size * 15
        max_correspondence_distance_fine = voxel_size * 1.5

        def pairwise_registration(source, target):
            icp_coarse = o3d.pipelines.registration.registration_icp(
                source, target, max_correspondence_distance_coarse, np.identity(4),
                o3d.pipelines.registration.TransformationEstimationPointToPlane())
            icp_fine = o3d.pipelines.registration.registration_icp(
                source, target, max_correspondence_distance_fine,
                icp_coarse.transformation,
                o3d.pipelines.registration.TransformationEstimationPointToPlane())
            transformation_icp = icp_fine.transformation
            information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
                source, target, max_correspondence_distance_fine,
                icp_fine.transformation)
            return transformation_icp, information_icp

        def full_registration(pcds):
            pose_graph = o3d.pipelines.registration.PoseGraph()
            odometry = np.identity(4)
            pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
            n_pcds = len(pcds)
            for source_id in range(n_pcds):
                for target_id in range(source_id + 1, n_pcds):
                    transformation_icp, information_icp = pairwise_registration(
                        pcds[source_id], pcds[target_id])
                    if target_id == source_id + 1:  # odometry case
                        odometry = np.dot(transformation_icp, odometry)
                        pose_graph.nodes.append(
                            o3d.pipelines.registration.PoseGraphNode(
                                np.linalg.inv(odometry)))
                        pose_graph.edges.append(
                            o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                                     target_id,
                                                                     transformation_icp,
                                                                     information_icp,
                                                                     uncertain=False))
                    else:  # loop closure case
                        pose_graph.edges.append(
                            o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                                     target_id,
                                                                     transformation_icp,
                                                                     information_icp,
                                                                     uncertain=True))
            return pose_graph

        for pcd in pcds_down:
            pcd.estimate_normals()

        pose_graph = full_registration(pcds_down)

        option = o3d.pipelines.registration.GlobalOptimizationOption(
            max_correspondence_distance=max_correspondence_distance_fine,
            edge_prune_threshold=0.25,
            reference_node=0)
        o3d.pipelines.registration.global_optimization(
            pose_graph,
            o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
            o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
            option)

        # use transformation matrix
        pcd_combined = o3d.geometry.PointCloud()
        for point_id in range(len(pcds_down)):
            pcds_down[point_id].transform(pose_graph.nodes[point_id].pose)
            pcd_combined += pcds_down[point_id]

        pcd_combined.voxel_down_sample(voxel_size=voxel_size)
        return PointCloud(point_cloud=pcd_combined), pose_graph.nodes

    def update(self, other_states: List[State]):

        pcs_list = [self.state.traversable_pc] + [state.traversable_pc for state in other_states]

        all_pcs = [PointCloud(points=np.asarray(pc.pc.points)) for pc in pcs_list]

        traversed_pc_merged, pose_graph = StateManager.merge(
            all_pcs
        )

        for i, other_state in enumerate(other_states, start=1):

            other_state.traversable_pc.pc.transform(pose_graph[i].pose)

            pc_with_position = PointCloud(points=[other_state.current_position])
            pc_with_position.pc.transform(pose_graph[i].pose)
            position = other_state.traversable_pc.estimate_smooth_closest_point(
                point=pc_with_position.pc.points[0])
            other_state.set_position(position)

        self.state.traversable_pc.set(PointCloud(point_cloud=traversed_pc_merged.down_sample()))

        if VISUALIZE_STATE:
            vis = o3d.visualization.VisualizerWithKeyCallback()
            vis.create_window(height=1000, width=1400)

            for other_state in [self.state] + other_states:

                geo = other_state.traversable_pc.pc
                geo.paint_uniform_color([1, 0, 0])
                vis.add_geometry(geo)

                centroid2: o3d.geometry.TriangleMesh = o3d.geometry.TriangleMesh.create_sphere(radius=0.5)
                centroid2.translate(other_state.current_position, relative=False)
                centroid2.paint_uniform_color([0, 1, 0])
                vis.add_geometry(centroid2)

            for _ in range(TIME_WINDOW):
                for _ in range(FPS):
                    vis.poll_events()
                    vis.update_renderer()
                    sleep(1 / FPS)

if __name__ == '__main__':
    MAP_POINT_PER_METER = 45
    MAP_SIZE = (15, 15, 0.0005)
    ALLOW_ROTATION = True
    from scipy.ndimage import rotate

    from typing import Union
    colors: Union[None, List[Tuple[float, float, float]]] = [
        (255 / 255, 0 / 255, 0 / 255),
        (0 / 255, 0 / 255, 255 / 255),
        (0 / 255, 255 / 255, 0 / 255),
    ]

    def random_map(delta_x: float, delta_y: float, delta_z: float, n: float = None) -> np.ndarray:

        if n is None:
            n = MAP_POINT_PER_METER * delta_x * delta_y

        return np.random.uniform([-delta_x / 2, -delta_y / 2, -delta_z / 2], [delta_x / 2, delta_y / 2, delta_z / 2],
                                 size=(n, 3))


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
                z += mountain_z / (new_x * new_x + new_x * new_y + new_y * new_y + mountain_delta_z)

                # z += -(0.01 * (x-1) * (x-1) + 0.01 * (y-1) * (y-1))
            p[2] = z

        return points

    # ROBOT 1
    robot1_state = State()

    # create the map
    robot1_traversable_points = map_mountain(delta_x=MAP_SIZE[0]+3, delta_y=MAP_SIZE[1]-7, delta_z=MAP_SIZE[2])
    print(f"Added {len(robot1_traversable_points)} points to the map of dimensions {MAP_SIZE}")

    # add map to state
    robot1_state.traversable_pc.extend(robot1_traversable_points)
    robot1_state.inaccessible_pc.extend(np.asarray([]))
    robot1_state.set_position((0,0,0))

    robot1_state_manager = StateManager(state=robot1_state, communication_node=CommunicationNode(identifier=1))

    prev_robot1_pc = PointCloud(points=np.asarray(robot1_state.traversable_pc.pc.points))

    # ROBOT 2

    robot2_state = State()

    # create the map
    robot2_traversable_points = map_mountain(delta_x=MAP_SIZE[0]-5, delta_y=MAP_SIZE[1]+6, delta_z=MAP_SIZE[2])
    print(f"Added {len(robot2_traversable_points)} points to the map of dimensions {MAP_SIZE}")


    # add map to state
    robot2_state.traversable_pc.extend(robot2_traversable_points)
    robot2_state.inaccessible_pc.extend(np.asarray([]))
    robot2_state.set_position((0, 0, 0))

    if ALLOW_ROTATION:
        rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle((np.radians(30), np.radians(-25), 0))
        robot2_state.traversable_pc.pc.rotate(rotation_matrix, center=(0, 0, 0))

    StateManager(state=robot2_state, communication_node=CommunicationNode(identifier=2))

    prev_robot2_pc = PointCloud(points=np.asarray(robot2_state.traversable_pc.pc.points))

    # update robot1
    robot1_state_manager.update([robot2_state])

    # color pcs
    prev_robot1_pc.pc.paint_uniform_color(colors[0])
    prev_robot1_pc.pc.estimate_normals()
    prev_robot2_pc.pc.paint_uniform_color(colors[1])
    prev_robot2_pc.pc.estimate_normals()

    new_robot1_pc = robot1_state_manager.state.traversable_pc
    new_robot1_pc.pc.paint_uniform_color(colors[2])
    new_robot1_pc.pc.estimate_normals()

    # o3d.visualization.draw_geometries([prev_robot1_pc.pc, prev_robot2_pc.pc])
    o3d.visualization.draw_geometries([prev_robot1_pc.pc, prev_robot2_pc.pc, new_robot1_pc.pc])
    o3d.visualization.draw_geometries([new_robot1_pc.pc])