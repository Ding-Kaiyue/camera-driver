# camera_driver

这个程序会：

1. 启动 Intel RealSense 深度流（`Z16`）
2. 将深度图转换为相机坐标系点云
3. 根据 TF / 固定外参把点云转换到世界坐标系
4. 按照机械臂椭球体做自身点云过滤
5. 用处理后的实时点云更新内部 REMANI 风格 ESDF
6. 发布避障输入点云、相机世界位姿和 ESDF 可视化

## 依赖

- `librealsense2`
- `Eigen3`
- ROS 2 Humble

## 配置

现在按职责拆成 3 份配置：

- `config/camera.yaml`
- `config/filter.yaml`
- `config/esdf_param.yaml`

其中：

- `/camera_driver/obstacle_pointcloud` 是裁地面和自体过滤后的实时避障输入点云
- `/camera_driver/camera_world_pose` 是相机在世界坐标系下的位姿
- `/camera_driver/esdf_slice` 是实时 ESDF 切片可视化

## RViz2 显示

1. `Fixed Frame` 设为 `world`
2. 添加 `/camera_driver/obstacle_pointcloud`
3. 如需检查相机位姿，再添加 `/camera_driver/camera_world_pose`
4. 如需检查 ESDF，再添加 `/camera_driver/esdf_slice`

## 说明

- 现在使用 camera_driver 内部的 REMANI 风格实时 ESDF
- 点云发布固定为经过裁地面和自体过滤后的避障输入
- 若要保证点云和机械臂对齐，`camera.yaml` 中的 `parent_frame`、TF 树和外参必须一致
