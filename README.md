# zm_robot
- The zm_robot is a AGV using four Mecanum wheel.

- Sensor: sick s30B x 2, RGB-D camera and IMU sensor.

- Software: Robot Operating System 2.

- Version: foxy.

------

-  Step1. Install package.

- Your PC need to install ros package.

    ``` $ sudo apt-get install ros-foxy-gazebo-ros-pkgs ros-foxy-gazebo-ros-control ros-foxy-ros-controllers ```

    ``` $ sudo apt-get install -y libgazebo11-dev ```
    
    ``` $ sudo apt-get install -y gazebo11 ```

  - Install ros navigation package.

    ``` $ sudo apt-get install -y ros-foxy-cartographer-ros ```
  
    ``` $ sudo apt-get install -y ros-foxy-navigation2 ```

- Step2. Install gazebo_mecanum_plugins package.

``` bash
$ cd <catkin_workspace>/src
```

``` bash
$ git clone -b ros2-foxy https://github.com/qaz9517532846/gazebo_mecanum_plugins.git
```

``` bash
$ cd ..
```

``` bash
$ colcon build
```

- Step3. Install AWS RoboMaker Small Warehouse World

``` bash
$ cd <catkin_workspace>/src
```

``` bash
$ git clone -b ros2 https://github.com/aws-robotics/aws-robomaker-small-warehouse-world
```

``` bash
$ cd ..
```

``` bash
$ colcon build
```

- Step4. Open zm_robot package.

The zm_robot 3d model xaro file into Rviz.

``` bash
$ ros2 launch zm_robot_description zm_robot_demo.launch.py
```

- The zm_robot 3d model xaro file into Gazebo.

``` bash
$ ros2 launch zm_robot_gazebo zm_robot_empty_world.launch.py
```

- This is a zm_robot control using a keyboard.

``` bash
$ ros2 run zm_robot_teleop_key zm_robot_teleope_key.py
```

- The zm_robot warehouse environment under Gazebo.

``` bash
$ ros2 launch zm_robot_gazebo zm_robot_aws_warehouse.launch.py
```

- The zm_robot create a map at warehouse.

``` bash
$ ros2 launch zm_robot_navigation zm_robot_cartographer.launch.py
```

- The zm_robot can do navigation and aviod obstacles at warehouse.

``` bash
$ ros2 launch zm_robot_navigation zm_robot_navigation2.launch.py
```

------

## zm_robot topic

| Topic                              | Description                                                       |
| ---                                | ---                                                               | 
| cmd_vel                            | zm_robot input to move velocity.                                  |
| joint_states                       | zm_robot joint status topic.                                      |
| odom                               | zm_robot odomentry topic.                                         |
| sick_s30b/laser/scan0              | sick laser1 Laserscan.                                            |
| sick_s30b/laser/scan1              | sick laser2 Laserscan.                                            |
| kinect_v2/color/image_raw          | Kinect V2 RGB image.                                              |
| kinect_v2/depth/image_raw          | Kinect V2 Depth image.                                            |
| zm_robot_imu                       | zm_robot IMU sensor.                                              ||

------


## History:

| Version        | Description                                                                 | File Creation |
| ---            | ---                                                                         | ---           |
| v1.0           | zm_robot original version.                                                  | July, 2020.   |
| v2.0           | zm_robot v2.0 version for zm_robot programing simply.                       | July, 2021.   |
| v2.1           | zm_robot v2.1 version add safety function           .                       | August, 2021. |
| v3.0           | zm_robot v3.0 version using PID controller for zm_robot programing simply.  | July, 2021.   |
| v4.0           | zm_robot v4.0 under ROS 2 Foxy                      .                       | August, 2021. ||

------

## Reference:

[1]. turtlebot3_teleop, https://github.com/ROBOTIS-GIT/turtlebot3/tree/master/turtlebot3_teleop

[2]. mecanum_sim, https://github.com/joelillo/mecanum_sim

[3]. Gazebo tutorial - Control plugin. http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5

[4]. Gazebo tutorial - Sensor plugin. http://gazebosim.org/tutorials?tut=ros_gzplugins&cat=connect_ros

[5]. aws-robomaker-small-warehouse-world. https://github.com/aws-robotics/aws-robomaker-small-warehouse-world

[6]. gazebo_mecanum_plugins. https://github.com/qaz9517532846/gazebo_mecanum_plugins

------

This repository is for your reference only. copying, patent application, academic journals are strictly prohibited.

Copyright © 2021 ZM Robotics Software Laboratory.
