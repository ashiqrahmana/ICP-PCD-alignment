# Iterative Closest Point (ICP) Alignment - README

## Introduction
This repository contains the code for aligning two point clouds using the Iterative Closest Point (ICP) algorithm. The project consists of two main tasks.

### Task 1 (2pt)
Aligning demo point clouds provided by Open3D using a custom implementation of the ICP algorithm.

### Task 2 (2pt)
Aligning two point clouds from the KITTI dataset using the same ICP implementation as in Task 1. Comparing the results between Task 1 and Task 2.

## Code Structure

### Task 1: Aligning Open3D Demo Point Clouds
```python
import open3d as o3d
import copy

demo_icp_pcds = o3d.data.DemoICPPointClouds()
source = o3d.io.read_point_cloud(demo_icp_pcds.paths[0])
target = o3d.io.read_point_cloud(demo_icp_pcds.paths[1])

# Write your ICP implementation here

def draw_registration_result(source, target, transformation):
    """
    param: source - source point cloud
    param: target - target point cloud
    param: transformation - 4 X 4 homogeneous transformation matrix
    """
    # Visualization code
```

### Task 2: Aligning KITTI Point Clouds
```python
import open3d as o3d
import copy

# Load point clouds from the KITTI dataset (data/Task2 folder)
source_kitti = o3d.io.read_point_cloud("data/Task2/source_kitti.pcd")
target_kitti = o3d.io.read_point_cloud("data/Task2/target_kitti.pcd")

# Write your ICP implementation here

def draw_registration_result_kitti(source, target, transformation):
    """
    param: source - source point cloud
    param: target - target point cloud
    param: transformation - 4 X 4 homogeneous transformation matrix
    """
    # Visualization code
```

## Implementation Details
### ICP Algorithm
- Loads two point clouds (source and target).
- Initializes the ICP function with default parameters and copies of the source and target point clouds.
- Initializes the cumulative transformation matrix as the identity matrix.
- Computes the mean of target points and extracts the target points as a NumPy array.
- Iterates over a specified number of iterations.
- Builds a KD-tree from the current source point cloud.
- For each point in the target cloud, finds the nearest neighbor in the source cloud and collects the corresponding source points.
- Computes the transformation using Singular Value Decomposition (SVD).
- Updates the cumulative transformation matrix.
- Applies the new transformation to the source point cloud.
- Calculates the Euclidean norm of the difference between the new transformation and the identity matrix.
- Checks for convergence by comparing the change in transformation with a specified tolerance.

### Visualization
- Visualizes the transformed source and original target point clouds using Open3D.
- Plots and displays the convergence over iterations.

## Results and Visualizations
### Task 1
- [ICP End Result View 1](path/to/figure3.png)
- [ICP End Result View 2](path/to/figure4.png)

### Task 2
- [Pre-ICP](path/to/figure5.png)
- [Post-ICP](path/to/figure6.png)

## Conclusion
The implemented ICP algorithm successfully aligns point clouds in both Task 1 and Task 2. The visualizations demonstrate the effectiveness of the alignment process.

## Usage
To run the code, follow the instructions in the provided Python scripts for Task 1 and Task 2.

Note: Using an ICP API in existing libraries instead of the provided implementation will result in a reduced score.

## Author
- Name: Ashiq Rahman Anwar Batcha
- NetID: aa10277
