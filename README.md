
# DeepStream-Yolo-ROS

[English Version](README_en.md)

本项目已适配 Jetson 平台 DeepStream 6.2 + CUDA 11.4。

> **说明**：使用 `catkin_make` 构建本包时，仅生成消息相关文件，不编译二进制程序。二进制程序需进入 `deepstream-app-roscpp` 文件夹，使用 Makefile 单独编译。

核心代码位于 `deepstream-app-roscpp`，基于 NVIDIA 官方 deepstream-app 改写，增加了 ROSCPP 节点初始化与消息发布功能。

---

## 编译流程

1. **构建 ROS 消息头文件**

   ```bash
   cd catkin_ws
   catkin_make -DCATKIN_WHITELIST_PACKAGES=deepstream_yolo_ros
   ```

2. **编译 deepstream-app-roscpp 二进制程序**

   ```bash
   cd catkin_ws/src/deepstream_yolo_ros/deepstream-app-roscpp
   export CUDA_VER=11.4
   export CATKIN_WORKSPACE=~/catkin_ws
   make
   make install
   ```

3. **编译 DeepStream-Yolo 工程**

   参考 [DeepStream-Yolo](https://github.com/marcoslucianops/DeepStream-Yolo) 官方文档。

---

## 配置方法

1. **launch 文件配置**

   修改 `<arg name="config" default="/home/ubuntu/deepstream_app_config.txt" />`，将 DeepStream-Yolo 的配置文件路径填入此处。

2. **primary-gie 的 interval 参数**

   如果在配置文件中为 `primary-gie` 设置了 `interval` 且不为零，请确保该参数已写入 launch 文件对应的 txt 配置文件的`[primary-gie]`区域 **（而不是类似`config_infer_primary_***.txt`的文件中的`[property]`）** ，否则 `deepstream-app-roscpp` 无法正确获取。

---

## 启动方法

```bash
source catkin_ws/devel/setup.bash
roslaunch deepstream_yolo_ros deepstream_yolo_ros.launch
```

---

## 许可证

MIT License