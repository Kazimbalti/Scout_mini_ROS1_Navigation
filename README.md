# Scout_mini_ROS1_Navigation
Implementation of Autonomous driving of scout_mini in gazebo simulator using SLAM+EKF+Point_Cloud+GMAPPING

### **1. Installation**
1. mkdir -p ~/scout_ws/src
2. cd ~/scout_ws/src
3. Clone this repository in your src as
   git clone
4. chmod +x install_packages.sh && ./setup.sh
   mapping, Navigation, Robot_localization packages will be downloaded
5. rosdep install --from-paths src --ignore-src -r -y
6. catkin_make
7. source devel/setup.bash
   **rosdep install** command will automatically install the required dependencies for the packages in the workspace. The dependencies are listed in CMakeLists.txt file in the packages.

### **2. Usage**

1. **Display platform description in RVIZ**
    ```
    cd ~/scout_ws
    source devel/setup.bash
    roslaunch scout_description display_scout_mini.launch 
    ```
    This will show you default vehicle platform without additional sensors.

2. **Launch gazebo simulator and teleop control**

    a. Launch gazebo simulator
    ```
    cd ~/scout_ws
    source devel/setup.bash
    roslaunch scout_gazebo_sim scout_mini_empty_world.launch
    ```

    b. Run teleop controller (move: w, a, x, d / stop: s)

    ```
    //Open another terminal

    cd ~/scout_ws
    source devel/setup.bash
    roslaunch scout_teleop scout_teleop_key.launch 
    ```

### **3. 2D SLAM**

Before running any command below, source devel/setup.bash

0. **Run Simulator**

    ```
    roslaunch scout_gazebo_sim scout_mini_playpen.launch
    ```

1. **Odometry & Kalman Filter Localization**
    ```
    roslaunch scout_base scout_mini_base.launch
    ```
    ```
    roslaunch scout_filter ekf_filter_cmd.launch
    ```

2. **SLAM mapping**

    a. Gmapping

    ```
    // Run gmapping slam
    roslaunch scout_slam scout_slam.launch
    ```

    b. Drive & mapping

    ```
    // Drive via teleop
    roslaunch scout_teleop scout_teleop_key.launch
    ```

    c. Save map

    ```
    // Save map
    roslaunch scout_slam gmapping_save.launch
    ```
    - default map file name: map1
    - map file will be saved in "scout_bringup/scout_slam/maps"
    - you can change saved map file name in the launch file
    - or you can set file name
        ```
        roslaunch scout_slam gmapping_save.launch map_file:=(file name)
        ```

### 3. **2D Navigation**

Before running any command below, source devel/setup.bash

0. **Run Simulator**

    ```
    roslaunch scout_gazebo_sim scout_mini_playpen.launch
    ```

1. **Odometry & Kalman Filter Localization**

    ```
    roslaunch scout_base scout_mini_base.launch
    ```

    ```
    roslaunch scout_filter ekf_filter_cmd.launch
    ```

2. Navigation

    ```
    // Run navigation
    roslaunch scout_navigation scout_navigation.launch
    ```
    you can set the destination via "2D Nav Goal" button
