import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def loader(file_path):
    return o3d.io.read_point_cloud(file_path)

def pass_through_filter(pcd, min_value=-1.5, max_value=-0.5):
    # remove thr ground
    points = np.asarray(pcd.points)
    
    filter = np.logical_and(points[:, 2] >= min_value, points[:, 2] <= max_value)

    filtered_points = points[filter]
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
    
    return filtered_pcd

def projection_to_xy_plane(pcd):
    # discard z coordinates for projection
    points = np.asarray(pcd.points)
    return points[:, :2]  

def save_file(map_2d, file_path):
    # saving file
    np.savetxt(file_path, map_2d, delimiter=',')

if __name__ == '__main__':
    # Load pcd
    pcl_cloud = loader('/home/risheek/Documents/ME5413_Final_Project_Group_12-main/map_extraction/maps/GlobalMap.pcd')

    # filter the pointclod - to remove the ground
    obstacles_cloud = pass_through_filter(pcl_cloud, min_value=-0.1, max_value=20)

    # Project the obstacles to 2D
    map_2d = projection_to_xy_plane(obstacles_cloud)
    
    # Plot the data
    plt.scatter(map_2d[:, 0], map_2d[:, 1], s=0.1)  # s is the marker size
    plt.xlabel('X coordinate')
    plt.ylabel('Y coordinate')
    plt.title('2D Map Visualization')
    plt.axis('equal')  # Ensure equal scaling for both axes
    plt.grid(True)
    plt.show()

    # Save the 2D map
    save_file(map_2d, './2d_map.csv')

