# DeepStream-Yolo-ROS

This project is adapted for Jetson platform DeepStream 6.2 + CUDA 11.4.

> **Note**: When building this package with `catkin_make`, only message-related files are generated. The binary program must be compiled separately in the `deepstream-app-roscpp` folder using the provided Makefile.

The core code is located in `deepstream-app-roscpp`, which is based on NVIDIA's official deepstream-app example, with added ROSCPP node initialization and message publishing functionality.

---

## Build Process

1. **Build ROS message headers**

   ```bash
   cd catkin_ws
   catkin_make -DCATKIN_WHITELIST_PACKAGES=deepstream_yolo_ros
   ```

2. **Compile the deepstream-app-roscpp binary**

   ```bash
   cd catkin_ws/src/deepstream_yolo_ros/deepstream-app-roscpp
   export CUDA_VER=11.4
   export CATKIN_WORKSPACE=~/catkin_ws
   make
   make install
   ```

3. **Build the DeepStream-Yolo project**

   Refer to the official [DeepStream-Yolo](https://github.com/marcoslucianops/DeepStream-Yolo) documentation.

---

## Configuration

1. **Launch file configuration**

   Modify `<arg name="config" default="/home/ubuntu/deepstream_app_config.txt" />` in the launch file, and set the path to your DeepStream-Yolo configuration file.

2. **primary-gie interval parameter**

   If you set the `interval` parameter for `primary-gie` in the configuration file and its value is not zero, make sure this parameter is written in the `[primary-gie]` section of the txt config file specified in the launch file (**not in the `[property]` section of files like `config_infer_primary_***.txt`**), otherwise `deepstream-app-roscpp` will not read it correctly.

---

## How to Start

```bash
source catkin_ws/devel/setup.bash
roslaunch deepstream_yolo_ros deepstream_yolo_ros.launch
```

---

## License

MIT License
