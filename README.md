# zm_robot
The zm_robot is a AGV using four Mecanum wheel.

Sensor: sick s30B x 2, RGB-D camera and IMU sensor.

Software: Robot Operating System.

Version: kinetic and melodic.

# Step1. Install package.

Your PC need to install ros package.

if your pc used melodic version.

``` bash
$ sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control ros-melodic-ros-controllers
```

``` bash
$ sudo apt-get install -y libgazebo9-dev
```

``` bash
$ sudo apt-get install -y gazebo9
```

Install ros navigation package.

``` bash
$ sudo apt-get install -y ros-melodic-openslam-gmapping ros-melodic-navigation
```

``` bash
$ sudo apt-get install -y ros-melodic-amcl ros-melodic-move-base
```
------

if your pc used kinetic version.

``` bash
$ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control ros-kinetic-ros-controllers
```

``` bash
$ sudo apt-get install -y libgazebo7-dev
```

``` bash
$ sudo apt-get install -y gazebo7
```

Install ros navigation package.

``` bash
$ sudo apt-get install -y ros-kinetic-openslam-gmapping ros-kinetic-navigation
```

``` bash
$ sudo apt-get install -y ros-kinetic-amcl ros-kinetic-move-base
```

------

# Step2. Install ira_laser_tools package.

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

------

# Step3. Open zm_robot package.

The zm_robot 3d model xaro file into Rviz.

``` bash
$ roslaunch zm_robot_description zm_robot_demo.launch
```

![image](https://github.com/qaz9517532846/zm_robot/blob/master/image/zm_robot_rviz.png)

------

# The zm_robot 3d model xaro file into Gazebo.

``` bash
$ roslaunch zm_robot_gazebo zm_robot_world.launch
```
![image](https://github.com/qaz9517532846/zm_robot/blob/master/image/zm_robot_gazebo.png)

------

# The zm_robot wheel controllers in rviz and gazebo.

``` bash
$ roslaunch zm_robot_control zm_robot_control_rviz.launch
```

![image](https://github.com/qaz9517532846/zm_robot/blob/master/image/zm_robot_control.png)

------

# This is a zm_robot control using a keyboard.

``` bash
$ cd <catkin_workspace>/src/zm_robot/zm_robot_teleop_key/scripts
```

``` bash
$ chmod +x zm_robot_teleope_key.py
```

``` bash
$ rosrun zm_robot_teleop_key zm_robot_teleope_key.py
```

------

# The zm_robot using PID controller.

``` bash
$ roslaunch zm_robot_gazebo zm_robot_pid.launch
```

``` bash
$ roslaunch zm_robot_pid_control zm_robot_pid_rviz.launch
```

![image](https://github.com/qaz9517532846/zm_robot/blob/master/image/zm_robot_pid_control.png)

------

# The zm_robot warehouse environment under Gazebo.

``` bash
$ roslaunch zm_robot_gazebo zm_robot_warehouse.launch
```

![image](https://github.com/qaz9517532846/zm_robot/blob/master/image/zm_robot_in_warehouse.png)

------

# The zm_robot create a map at warehouse.

``` bash
$ roslaunch zm_robot_navigation zm_robot_gmapping.launch
```

![image](https://github.com/qaz9517532846/zm_robot/blob/master/image/zm_robot_gmapping.png)

------

# The zm_robot can do navigation and aviod obstacles at warehouse.

``` bash
$ roslaunch zm_robot_navigation zm_robot_navigation.launch
```

![image](https://github.com/qaz9517532846/zm_robot/blob/master/image/zm_robot_navigation.png)

------

# The zm_robot can do navigation and aviod obstacles at warehouse using programing.

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

illustration:

| Function                           | Description                                                |
| ---                                | ---                                                        | 
| my_zm_robot.move_map(x, y, theta)  | zm_robot can move to designated location relative to map.  |
| my_zm_robot.move_base(x, y, theta) | zm_robot can move to designated location relative to base. ||

------

# Reference:

[1]. turtlebot3_teleop, https://github.com/ROBOTIS-GIT/turtlebot3/tree/master/turtlebot3_teleop

[2]. mecanum_sim, https://github.com/joelillo/mecanum_sim

[3]. Gazebo tutorial - Control plugin. http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5

[4]. warehouse_simulation_toolkit. https://github.com/wh200720041/warehouse_simulation_toolkit
