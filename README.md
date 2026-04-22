# RealSense -> Voxblox ESDF Bridge (C++)

这个示例程序会：

1. 启动 Intel RealSense 深度流（`Z16`）
2. 将深度图转换为相机坐标系点云
3. 把点云积分到 `voxblox::EsdfPipeline`（TSDF）
4. 周期性更新 ESDF
5. 导出：
   - `tsdf.voxblox`
   - `esdf.voxblox`
   - `esdf_slice.csv`

## 依赖

- `librealsense2`（已验证你本机可用）
- `Eigen3`
- `glog/gflags/protobuf`（由 voxblox 依赖）
- 本机 voxblox 源码目录：`~/Projects/voxblox/voxblox`

## 编译

```bash
cd ~/Projects/camera_driver_lib
source /opt/ros/humble/setup.bash
mkdir -p build
cd build
cmake ..
make -j
```

## 运行

程序现在是 YAML-only 模式，参数全部来自：

- `config/bridge.yaml`
- `config/camera.yaml`

运行方式：

```bash
cd ~/Projects/camera_driver_lib
./build/realsense_voxblox_bridge
```

说明：`pub_min_ray/pub_max_ray` 只影响 ROS2 点云发布，不影响 TSDF/ESDF 积分过滤；`pub_max_ray <= 0` 表示不做上限截断。

运行中 `Ctrl+C` 可以提前停止并导出结果。

## RViz2 显示点云

1. 新开终端：

```bash
source /opt/ros/humble/setup.bash
rviz2
```

2. 在 RViz2 中：

- `Global Options -> Fixed Frame` 设为 `world`
- `Add -> By topic -> /realsense/points -> PointCloud2`

3. 如果看不到点云，检查：

- `config/bridge.yaml` 的 `ros2_pub` 是否为 `1`
- `config/bridge.yaml` 的 `ros2_frame` 是否与 RViz 的 `Fixed Frame` 一致

## 说明

- 当前默认把相机位姿固定为世界系原点（`T_G_C = Identity`），适合先验证链路。
- 若要构建全局地图，需要把你自己的位姿估计（VIO/里程计/SLAM）接入 `T_G_C`。
- 现在支持从 `config/camera.yaml` 读取固定外参（`tx,ty,tz,qx,qy,qz,qw`），用于固定相机的眼在手外场景。
