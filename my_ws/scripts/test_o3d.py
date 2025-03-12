import open3d as o3d
import numpy as np

# Create random points to represent voxel centers
num_points = 1000
points = np.random.rand(num_points, 3) * 10  # Scale points to fit in a 10x10x10 grid

# Create a point cloud from the points
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# Create the voxel grid from the point cloud
voxel_size = 0.5  # Size of each voxel
voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size)

# Visualize the voxel grid (headless mode option)
o3d.visualization.draw_geometries([voxel_grid], window_name="Voxel Grid", width=800, height=600, left=50, top=50, point_show_normal=False, mesh_show_back_face=False, window_background_color=[0, 0, 0])
