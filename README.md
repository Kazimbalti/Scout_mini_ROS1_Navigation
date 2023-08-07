# Scout_mini_ROS1_Navigation
Implementation of Autonomous driving of scout_mini in gazebo simulator using SLAM+EKF+Point_Cloud+GMAPPING

### **1. Installation**
```
1. mkdir -p ~/scout_ws/src
2. cd ~/scout_ws/src
3. Clone this repository in your src as
   git clone
4. chmod +x install_packages.sh && ./setup.sh
```
   mapping, Navigation, Robot_localization packages will be downloaded
```
5. rosdep install --from-paths src --ignore-src -r -y
6. catkin_make
7. source devel/setup.bash
```
   **rosdep install** command will automatically install the required dependencies for the packages in the workspace. The dependencies are listed in CMakeLists.txt file in the packages.

### **2. Implementation**

1. **Display platform description in RVIZ**
    ```
    In terminal 1: 
    cd ~/scout_ws
    source devel/setup.bash
    roslaunch scout_description display_scout_mini.launch 
    ```
    This will show you default vehicle platform without additional sensors.
![rviz](https://github.com/Kazimbalti/Scout_mini_ROS1_Navigation/assets/32608321/af6bc104-013a-4f52-b224-ee08d63f186c)

2. **Launch gazebo simulator and teleop control**

    a. Launch gazebo simulator
    ```
    In terminal 1:
    cd ~/scout_ws
    source devel/setup.bash
    roslaunch scout_gazebo_sim scout_mini_empty_world.launch
    ```
![gazebo](https://github.com/Kazimbalti/Scout_mini_ROS1_Navigation/assets/32608321/0420164b-19c9-41ae-ae81-878f9d0f2172)

    b. Run teleop controller (move: w, a, x, d / stop: s)

    ```
    //Open another terminal
    In Terminal 2:
    
    cd ~/scout_ws
    source devel/setup.bash
    roslaunch scout_teleop scout_teleop_key.launch 
    ```

### **3. 2D SLAM**

Before running any command below, source devel/setup.bash

0. **Run Simulator**

    ```
    In Terminal 1:
    roslaunch scout_gazebo_sim scout_mini_playpen.launch
    ```
![gazebo_slam_scout_mini](https://github.com/Kazimbalti/Scout_mini_ROS1_Navigation/assets/32608321/02a92a1f-ff7e-4cd8-9950-e92fe1fe0fd5)

1. **Odometry & Kalman Filter Localization**
    ```
    In Terminal 2:
    roslaunch scout_base scout_mini_base.launch
    ```
    ```
    In Terminal 3:
    roslaunch scout_filter ekf_filter_cmd.launch
    ```

2. **SLAM mapping**

    a. Gmapping

    ```
    In Terminal 4:
    // Run gmapping slam
    roslaunch scout_slam scout_slam.launch
    ```
![rviz_slam_scout_mini](https://github.com/Kazimbalti/Scout_mini_ROS1_Navigation/assets/32608321/c5a88abc-d917-483a-8324-e82d4580c1b4)

    b. Drive & mapping

    ```
    // Drive via teleop
    In terminal 5:
    roslaunch scout_teleop scout_teleop_key.launch
    ```

    c. Save map

    ```
    // Save map
    In Terminal 6:
    roslaunch scout_slam gmapping_save.launch
    ```
    - default map file name: map1
    - map file will be saved in "scout_bringup/scout_slam/maps"
    - you can change saved map file name in the launch file
    - or you can set file name
        ```
        roslaunch scout_slam gmapping_save.launch map_file:=(file name)
        ```
![slam_mymap](https://github.com/Kazimbalti/Scout_mini_ROS1_Navigation/assets/32608321/af47398a-2c7d-4257-824d-7703aca189a9)


Now close everything.

### 3. **2D Navigation**

Before running any command below, source devel/setup.bash

0. **Run Simulator**

    ```
    In terminal 1: 
    roslaunch scout_gazebo_sim scout_mini_playpen.launch
    ```
![gazebo_scout_mini](https://github.com/Kazimbalti/Scout_mini_ROS1_Navigation/assets/32608321/250f8a87-3ecb-4974-a600-f804028d9036)

1. **Odometry & Kalman Filter Localization**

    ```
    In Terminal 2:
    roslaunch scout_base scout_mini_base.launch
    ```

    ```
    In Terminal 3:
    roslaunch scout_filter ekf_filter_cmd.launch
    ```

2. Navigation

    ```
    // Run navigation
    In Terminal 4:
    roslaunch scout_navigation scout_navigation.launch
    ```
    you can set the destination via "2D Nav Goal" button
   
![rviz_nav_scout_mini](https://github.com/Kazimbalti/Scout_mini_ROS1_Navigation/assets/32608321/4c783656-85c2-4d21-b080-800404d18f69)
