import pandas as pd
import open3d as o3d
import numpy as np
import os

def load_voxel_data_from_csv(csv_filename):
    df = pd.read_csv(csv_filename)
    
    # Extract voxel positions and sizes
    voxel_centers = df[['CenterX', 'CenterY', 'CenterZ']].values
    voxel_sizes = df[['VoxelSize_m']].values
    
    voxel_colors = df[['ColorR', 'ColorG', 'ColorB']].values

    return voxel_centers, voxel_sizes, voxel_colors

def create_voxel_grid(voxel_centers, voxel_sizes, voxel_colors):
    voxel_grid = o3d.geometry.PointCloud()
    voxel_grid.points = o3d.utility.Vector3dVector(voxel_centers)
    voxel_grid.colors = o3d.utility.Vector3dVector(voxel_colors)

    return voxel_grid

def visualize_voxel_grid(voxel_grid):
    o3d.visualization.draw_geometries([voxel_grid])

def process_folder(folder_path):
    csv_files = [f for f in os.listdir(folder_path) if f.endswith(".csv")]
    if not csv_files:
        print("No CSV files found in folder.")
        return

    for csv_file in csv_files:
        csv_path = os.path.join(folder_path, csv_file)
        print(f"Processing: {csv_file}")

        voxel_centers, voxel_sizes, voxel_colors = load_voxel_data_from_csv(csv_path)
        
        
        if len(voxel_centers) > 100000:
            voxel_centers = voxel_centers[::10] 
            voxel_colors = voxel_colors[::10]
        
        voxel_grid = create_voxel_grid(voxel_centers, voxel_sizes, voxel_colors)
        visualize_voxel_grid(voxel_grid)

if __name__ == "__main__":
    folder_path = "bottle_0.025"  # Change this path
    process_folder(folder_path)
