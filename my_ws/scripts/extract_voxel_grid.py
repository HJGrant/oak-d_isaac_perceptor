import rclpy
from rclpy.node import Node
from nvblox_msgs.msg import VoxelBlockLayer, VoxelBlock
from geometry_msgs.msg import Point32
import numpy as np
import open3d as o3d

class VoxelGridExtractor(Node):
    def __init__(self):
        super().__init__('nvblox_voxel_grid_extractor')
        
        # Subscriber to the VoxelBlockLayer topic
        self.subscription = self.create_subscription(
            VoxelBlockLayer,
            '/nvblox_node/color_layer',
            self.voxel_block_layer_callback,
            10  # Queue size
        )
        
    def voxel_block_layer_callback(self, msg: VoxelBlockLayer):
        # Extract voxel block data from the VoxelBlockLayer message
        voxel_centers = []
        voxel_colors = []

        for block in msg.blocks:
            print("Entering the first loop!")
            # The centers and colors are arrays of geometry_msgs/Point32 and std_msgs/ColorRGBA
            for center, color in zip(block.centers, block.colors):
                voxel_centers.append([center.x, center.y, center.z])
                voxel_colors.append([color.r, color.g, color.b, color.a])  # RGBA values

        # Convert to numpy arrays for processing
        voxel_centers = np.array(voxel_centers)
        voxel_colors = np.array(voxel_colors)

        # If we have voxel data, save it as a .ply file
        if voxel_centers.shape[0] > 0:
            print("Saving!")
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(voxel_centers)
            pcd.colors = o3d.utility.Vector3dVector(voxel_colors[:, :3])  # Only use RGB for visualization
            
            # Save the point cloud to a .ply file
            ply_file = '/workspaces/isaac_ros-dev/my_ws/scripts/tmp/voxel_grid.ply'  # Change the path as needed
            o3d.io.write_point_cloud(ply_file, pcd)
            
            self.get_logger().info(f"Voxel grid saved to {ply_file} with {len(voxel_centers)} points.")

def main(args=None):
    rclpy.init(args=args)

    # Create the node and start spinning
    voxel_grid_extractor = VoxelGridExtractor()
    print("Spinning the Node!")
    rclpy.spin(voxel_grid_extractor)

    # Destroy the node when done
    voxel_grid_extractor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
