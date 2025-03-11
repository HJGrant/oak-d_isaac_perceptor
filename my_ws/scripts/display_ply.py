import open3d as o3d

# Load the .ply file
ply_file = '/home/itr/workspaces/isaac_ros-dev/my_ws/scripts/tmp/voxel_grid.ply'  # Replace with the path to your .ply file
pcd = o3d.io.read_point_cloud(ply_file)

# Visualize the point cloud
o3d.visualization.draw_geometries([pcd])
