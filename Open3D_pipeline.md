### Open3D pipeline

The reconstruction system pipeline of Open3D

#### 1. Make fragments

This step creates fragments from n_frames_per_fragment images. Before all, Open3D will first create pose graph for aligning point cloud from (or RGBD) frames. The pose graph contains nodes and edges. The node is a piece of geometry $P_{i}$ associated with a pose matrix $T_{i}$ which transforms $P_{i}$ into the global space. The set $T_{i}$ are the unknown variables to be optimized. 

When create the pose graph, the register_one_rgbd_pair function is used to compute the transformation matrix between two frames. For the non-adjacent RGBD frames, the function `pose_estimation()` which computes OpenCV ORB feature and performs 5-point RANSAC to estimate a rough alignment (transformation), which is used as the initialization of `compute_rgbd_odometry()`. For the adjacent RGBD frames, an identity matrix is used as initialization. Once a pose graph is created, multi-way registration is performed by calling function `optimize_posegraph_for_fragment()`. The function calls `global_optimization()` to estimate poses of the RGBD frames. For efficiency, only key frames are used.

**More details about pose graph of multi-way registration:**

[[Choi2015\]](http://www.open3d.org/docs/release/tutorial/reference.html#choi2015) has observed that pairwise registration is error-prone. False pairwise alignments can outnumber correctly aligned pairs. Thus, they partition pose graph edges into two classes. **Odometry edges** connect temporally close, neighboring nodes. A local registration algorithm such as ICP can reliably align them. **Loop closure edges** connect any non-neighboring nodes. The alignment is found by [global registration](http://www.open3d.org/docs/release/tutorial/Advanced/global_registration.html#global-registration) and is less reliable. 

**The code pipeline of - Make fragments:**

run_system.py -> make_fragments.py -> run() -> process_single_fragment() -> make_pose_graph_for_fragment() -> create two classes graph and write into files -> optimize_posegraph_for_fragment -> make_pointcloud_for_fragment().

注：**这里需要改成树状图的形式**

#### 2. Register fragments


#### 3. Refine registration

#### 4. Integrate scene

