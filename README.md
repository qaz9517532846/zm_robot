# zm_robot
- The zm_robot is a AGV using four Mecanum wheel.

- Sensor: sick s30B x 2, RGB-D camera and IMU sensor.

- Software: Robot Operating System.

- Version: kinetic, melodic, noetic.

------

-  Step1. Install package.

- Your PC need to install ros package.
- 
  - if your pc used noetic version.

    ``` $ sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-ros-controllers ```

    ``` $ sudo apt-get install -y libgazebo11-dev ```
    
    ``` $ sudo apt-get install -y gazebo11 ```

  - Install ros navigation package.

    ``` $ sudo apt-get install -y ros-noetic-openslam-gmapping ros-noetic-navigation ```
  
    ``` $ sudo apt-get install -y ros-noetic-amcl ros-noetic-move-base ```

  - if your pc used melodic version.

    ``` $ sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control ros-melodic-ros-controllers ```

    ``` $ sudo apt-get install -y libgazebo9-dev ```
    
    ``` $ sudo apt-get install -y gazebo9 ```

  - Install ros navigation package.

    ``` $ sudo apt-get install -y ros-melodic-openslam-gmapping ros-melodic-navigation ```
  
    ``` $ sudo apt-get install -y ros-melodic-amcl ros-melodic-move-base ```

  - if your pc used kinetic version.

    ``` $ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control ros-kinetic-ros-controllers ```
   
    ``` $ sudo apt-get install -y libgazebo7-dev ```
   
    ``` $ sudo apt-get install -y gazebo7 ```

  - Install ros navigation package.
  
    ``` $ sudo apt-get install -y ros-kinetic-openslam-gmapping ros-kinetic-navigation ```
    
    ``` $ sudo apt-get install -y ros-kinetic-amcl ros-kinetic-move-base ```

- Step2. Install gazebo_mecanum_plugins package.

``` bash
$ cd <catkin_workspace>/src
```

``` bash
$ git clone https://github.com/qaz9517532846/gazebo_mecanum_plugins.git
```

``` bash
$ cd ..
```

``` bash
$ catkin_make
```


- Step3. Install ira_laser_tools package.

``` bash
$ cd <catkin_workspace>/src
```

``` bash
$ git clone https://github.com/iralabdisco/ira_laser_tools.git
```

``` bash
$ cd ..
```

``` bash
$ catkin_make
```

- Step4. Open zm_robot package.

The zm_robot 3d model xaro file into Rviz.

``` bash
$ roslaunch zm_robot_description zm_robot_demo.launch
```

- The zm_robot 3d model xaro file into Gazebo.

``` bash
$ roslaunch zm_robot_gazebo zm_robot_world.launch
```

![image](https://github.com/qaz9517532846/zm_robot/blob/v2.1/image/zm_robot.png)


- This is a zm_robot control using a keyboard.

``` bash
$ rosrun zm_robot_teleop_key zm_robot_teleope_key.py
```

![image](https://github.com/qaz9517532846/zm_robot/blob/v2.1/image/zm_robot_control.png)


- The zm_robot warehouse environment under Gazebo.

``` bash
$ roslaunch zm_robot_gazebo zm_robot_warehouse.launch
```

![image](https://github.com/qaz9517532846/zm_robot/blob/v2.1/image/zm_robot_warehouse.png)


- The zm_robot create a map at warehouse.

``` bash
$ roslaunch zm_robot_navigation zm_robot_gmapping.launch
```

- The zm_robot can do navigation and aviod obstacles at warehouse.

``` bash
$ roslaunch zm_robot_navigation zm_robot_navigation.launch
```

![image](https://github.com/qaz9517532846/zm_robot/blob/v2.1/image/zm_robot_navigation.png)


- The zm_robot can do navigation and aviod obstacles at warehouse using programing.

``` bash
$ roslaunch zm_robot_navigation zm_robot_navigation.launch
```

``` bash
$ rosrun zm_robot_programing zm_robot_move
```

The zm_robot_move.cpp example.

``` bash
#include <zm_robot_programing/zm_robot_move_function.h>

void spinThread(){
  ros::spin();
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "zm_robot_move"); 
  ros::NodeHandle n;

  boost::thread spin_thread = boost::thread(boost::bind(&spinThread));

  zm_robot my_zm_robot;

  ////// zm_robot programing control start //////

  my_zm_robot.move_map(1.0, 3.0, 1.5708);
  
  my_zm_robot.move_base(1.0, 3.0, 1.5708);

  ////// END //////

  return 0;
}
```

## illustration:

| Function                           | Description                                                |
| ---                                | ---                                                        | 
| my_zm_robot.move_map(x, y, theta)  | zm_robot can move to designated location relative to map.  |
| my_zm_robot.move_base(x, y, theta) | zm_robot can move to designated location relative to base. ||

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

[5]. warehouse_simulation_toolkit. https://github.com/wh200720041/warehouse_simulation_toolkit

[6]. ira_laser_tools. http://wiki.ros.org/ira_laser_tools

[7]. dynamic_reconfigure. http://wiki.ros.org/dynamic_reconfigure/Tutorials

[8]. gazebo_mecanum_plugins. https://github.com/qaz9517532846/gazebo_mecanum_plugins

[9]. rto_core. https://github.com/dietriro/rto_core

------

This repository is for your reference only. copying, patent application, academic journals are strictly prohibited.

Copyright © 2021 ZM Robotics Software Laboratory.
