import open3d as o3d
import numpy as np
from time import sleep
import time

# create visualizer and window.
vis = o3d.visualization.Visualizer()
vis.create_window(height=480, width=640)

# initialize pointcloud instance.
pcd = o3d.geometry.PointCloud()
vis.add_geometry(pcd)


for _ in range(10):
    # remove, create and add new geometry.
    # vis.remove_geometry(pcd)
    pcd.points.extend(np.random.rand(1, 3))
    vis.update_geometry(pcd)

    vis.poll_events()
    vis.update_renderer()

    sleep(0.5)
vis.destroy_window()