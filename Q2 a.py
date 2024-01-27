import open3d as o3d
import numpy as np
import copy
import sys  # Add this import for sys.maxsize

# Load the source and target point clouds
demo_icp_pcds = o3d.data.DemoICPPointClouds()
source = o3d.io.read_point_cloud(demo_icp_pcds.paths[0])
target = o3d.io.read_point_cloud(demo_icp_pcds.paths[1])

o3d.visualization.draw_geometries([source, target], zoom=0.4459,
                                front=[0.9288, -0.2951, -0.2242],
                                lookat=[1.6784, 2.0612, 1.4451],
                                up=[-0.3402, -0.9189, -0.1996])

def icp(source, target, max_iterations=1000, tolerance=1e-5):
    transformation = np.identity(4)
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)

    transformation_array = []

    target_points = np.asarray(target_temp.points)
    target_center = np.mean(target_points, axis=0)

    for iteration in range(max_iterations):
        source_points = []
        
        # Represent source data as KD Tree
        source_tree = o3d.geometry.KDTreeFlann(source_temp)
        N = len(target_points)
        for i in range(N):                              # Iterating over target points
            # Map point from target to source and build the source points
            [k, idx, _] = source_tree.search_knn_vector_3d(target.points[i], 1) 
            source_points.append(source_temp.points[idx[0]])
        
        source_points = np.asarray(source_points)

        H = (target_points - target_center).T @ (source_points - np.mean(source_points, axis=0))
        U, S, Vt = np.linalg.svd(H)
        R = U @ Vt
        t = target_center - R @ np.mean(source_points, axis=0)

        # Update the transformation
        transformation_new = np.identity(4)
        transformation_new[:3, :3] = R
        transformation_new[:3,  3] = t
        transformation = transformation_new @ transformation

        # Apply the transformation to the original target point cloud
        source_temp.transform(transformation_new)

        transformation_array.append(np.linalg.norm(transformation_new - np.identity(4)))
        print(np.linalg.norm(transformation_new - np.identity(4)), iteration)

        # Check for convergence
        if np.linalg.norm(transformation_new - np.identity(4)) < tolerance:
            print("Wow breaking due to tolerance")
            break
            
    o3d.visualization.draw_geometries([source_temp, target_temp], zoom=0.4459,
                                    front=[0.9288, -0.2951, -0.2242],
                                    lookat=[1.6784, 2.0612, 1.4451],
                                    up=[-0.3402, -0.9189, -0.1996])
    
    import matplotlib.pyplot as plt
    plt.plot(np.arange(len(transformation_array)), transformation_array)
    plt.show()
    return transformation

# Find the transformation using ICP
transformation = icp(source, target)

