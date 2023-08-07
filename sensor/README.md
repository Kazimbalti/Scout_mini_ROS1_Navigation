# 0. Sensor List

- 2D LiDAR
- 3D LiDAR
    - VLP-16
    - HDL-32E
- IMU
- Camera
    - RGB camera
    - Openni-kinect
    - Realsense-d435
- GPS

## 1. Sensor Configuration

- Detailed setting for each sensor relative to the platform is written in scout/scout_description/urdf/mini.xacro
- Detailed information for each sensor is written in scout/scout_description/(sensor_name).xacro
  - ex) [imu.xacro](https://github.com/hjinnkim/ugv_gazebo_sim/blob/noetic-devel/scout/scout_description/urdf/imu.xacro)
    - If you want to change range option for lidar, you need to change the argument in the xacro file
- You can set the sensor on/off, change the position by setting the argument in the launch file.
    - Please refer scout_bringup/scout_test/scout_mini_sensor_test.launch

## 2. 2D LiDAR

### Reference

* https://www.youtube.com/watch?v=jJzzw2jk-lY

* https://classic.gazebosim.org/tutorials?tut=ros_gzplugins#GPULaser

* https://level-asphalt-6c6.notion.site/Gazebo-Scout-mini-add-lidar-086c23578e904467864f284ad02c8564

* hokuyo.dae : https://github.com/osrf/gazebo_models/tree/master/hokuyo/meshes

## 3. 3D LiDAR

### Reference

- https://github.com/lmark1/velodyne_simulator

## 4. realsense camera

#### If compressed image topics are not published

* try to install following ros packages where < distribution> is your ros-distro

    ```
    sudo apt install ros-<distribution>-image-transport-plugins ros-<distribution>-compressed-image-transport ros-<distribution>-theora-image-transport ros-<distribution>-compressed-depth-image-transport
    ```

### Reference

* https://github.com/pal-robotics/realsense_gazebo_plugin

* https://github.com/pal-robotics/realsense_gazebo_plugin/issues/7

* https://github.com/IntelRealSense/realsense-ros

* urdf : https://github.com/IntelRealSense/realsense-ros/tree/development/realsense2_description/urdf

* d435.dae : https://github.com/IntelRealSense/realsense-ros/tree/development/realsense2_description/meshes



### 4. IMU

#### Reference

- https://classic.gazebosim.org/tutorials?tut=ros_gzplugins#IMU(GazeboRosImu)

- https://answers.ros.org/question/12430/modelling-sensorsimu-in-gazebo/

### 5. GPS

#### Reference

- https://github.com/tu-darmstadt-ros-pkg/hector_gazebo/tree/melodic-devel/hector_gazebo_plugins/include/hector_gazebo_plugins