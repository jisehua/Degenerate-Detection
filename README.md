# Degenerate-Detection

### About
This project presents a point-to-distribution based approach for detecting LiDAR SLAM degeneracy. A novel degeneracy factor is introduced, leveraging local geometric model information to accurately characterize the algorithm's degeneracy state. This method not only effectively reduces noise interference and minimizes false detections but also enhances overall robustness.

### Video
https://github.com/jisehua/Degenerate-Detection/assets/89381045/50744cab-6248-4e52-8b5d-8fa28f6f5884

### Related Publications:
Sehua Ji, Weinan Chen, Zerong Su, Yisheng Guan, Jiehao Li, Hong Zhang, Haifei Zhu, **A Point-to-distribution Degeneracy Detection Factor for LiDAR SLAM using Local Geometric Models**, *IEEE ICRA 2024*.

If you use our method in an academic work, please cite:
```
@article{Ji2024APD,
  title={A Point-to-distribution Degeneracy Detection Factor for LiDAR SLAM using Local Geometric Models},
  author={Sehua Ji, Weinan Chen, Zerong Su, Yisheng Guan, Jiehao Li, Hong Zhang, Haifei Zhu},
  journal={2024 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2024},
  pages={12283-12289},
}
```

## How to use

1. Build the project in the ROS workspace
  ```
  mkdir -p catkin_ws/src && cd catkin_ws/src
  git clone
  cd catkin_ws
  catkin_make
  ```
2. Related parameter
  ```
  * star_launch:
      /resolution_value  # Initialize voxel size.
      /velodyne_points   # LiDAR frame ID.
      /save_path         # The path where the test results are saved.
  * voxel_grid_covariance_omp_impl.hpp:
      bool pub_mark;     # Switch for voxel visualization on rviz.(If set to true, the playback speed of the dataset can be reduced for better presentation, e.g. x0.5.)
  ```
3. Run
  ```
  roslaunch degeneracy_detection start.launch
  ```
## Notic
The voxel segmentation method mentioned in the paper has not yet been fully organized; therefore, part of the code is being open-sourced first.
