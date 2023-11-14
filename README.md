# Central Voting PPF

Reproduced code for:

> [Efficient Center Voting for Object Detection and 6D Pose Estimation in 3D Point Cloud](https://jianweiguo.net/publications/papers/2021_TIP_6DPose.pdf)

:broken_heart: No official repo for this great job.

## Dependence 

+ Cmake 3.5
+ PCL 1.8
+ Boost
+ OpenMP 
+ Eigen3

## Usage

### Compile the code

```bash
    mkdir build && cd build && cmake .. && make
```

### Run the code

```bash
    ./central_voting /path_to_src_point_cloud /path_to_tgt_point_cloud
```

+ The **.pcd point cloud file** is needed
+ arg1 is the model and arg2 is scene point cloud

### Warning

+ **Most Important**, **I didn’t reach the same result as the paper mentioned**, So maybe the code miss some implementation details and met this bad result. :disappointed:

+ This is not a complete project and I just finished the key code and check the correctness.
+ I did not write the Config file , so some params need to be adjusted from the code and don't forget recompile.

