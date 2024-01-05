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
