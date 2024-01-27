# ICP (Iterative Closest Point) Alignment

## Task Overview

### a) Part 1: Demo Point Clouds
In this part, the task involves aligning two point clouds (source and target) using the Iterative Closest Point (ICP) algorithm. Demo point clouds provided by Open3D are loaded and aligned using a custom ICP implementation. The code includes a function, `draw_registration_result`, for visualizing the registration results. The final 4x4 homogeneous transformation matrix obtained after the ICP refinement is used to transform and visualize the source point cloud aligned with the target.

#### Implementation Process:
1. The `extract_sift_features` function reads the images in grayscale and extracts SIFT features (keypoints and descriptors).
2. The `match_features` function matches query descriptors with database descriptors using the Brute-Force matcher, applying a ratio test to filter good matches.
3. Directory paths for query and database images are provided.
4. Results are stored in a list called `results` containing dictionaries with information about the query image, database image, and similarity score.
5. The script displays the top 5 matching results for each query image.

### b) Part 2: KITTI Dataset Point Clouds
In this part, two point clouds from the KITTI dataset are given, and the alignment is repeated using the custom ICP implementation. The results from Part 1 are compared with the results from Part 2.

#### Comparison and Analysis:
- The alignment results from Part 2 are evaluated and compared with those from Part 1.
- The visualizations for both parts are provided.

## Code Implementation

```python
# Place the provided code here
```

## Results and Visualizations

### Part 1: Demo Point Clouds
- Results display the top 5 matching database images for each query.
- Visualizations show the aligned source point cloud with the original target point cloud.

### Part 2: KITTI Dataset Point Clouds
- Comparison and analysis of the alignment results with visualizations.
- Evaluation of the success of point cloud alignment.

## Conclusion
The ICP algorithm is implemented and applied to align point clouds from both demo datasets and the KITTI dataset. The custom implementation ensures accurate alignment, and the visualizations provide insights into the effectiveness of the algorithm. Comparison between different datasets helps evaluate the robustness of the ICP approach in aligning diverse point cloud scenes.

**Author:** Ashiq Rahman Anwar Batcha, NetID: aa10277
