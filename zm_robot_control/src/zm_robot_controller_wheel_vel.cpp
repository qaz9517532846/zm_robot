#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <string>

float linear_x, linear_y, angular_z;

void call_twist(const geometry_msgs::Twist::ConstPtr& msg)
{
    linear_x = msg->linear.x;
    linear_y = msg->linear.x;
    angular_z = msg->angular.z;
}

int main(int argc, char** argv)
{
    // Node initialization
    ros::init(argc, argv, "zm_robot_controller_odom");
    ros::NodeHandle n;

    ros::Subscriber odom_pub = n.subscribe("cmd_vel", 50, &call_twist);

    ros::Publisher wheel1_command = n.advertise<std_msgs::Float64>("/zm_robot/joint1_velocity_controller/command", 50);
    ros::Publisher wheel2_command = n.advertise<std_msgs::Float64>("/zm_robot/joint2_velocity_controller/command", 50);
    ros::Publisher wheel3_command = n.advertise<std_msgs::Float64>("/zm_robot/joint3_velocity_controller/command", 50);
    ros::Publisher wheel4_command = n.advertise<std_msgs::Float64>("/zm_robot/joint4_velocity_controller/command", 50);

    ros::Rate loop_rate(50);

    while(n.ok())
    {
       std_msgs::Float64 wheel_1_control;
       std_msgs::Float64 wheel_2_control;
       std_msgs::Float64 wheel_3_control;
       std_msgs::Float64 wheel_4_control;

       if(linear_x != 0 && linear_y == 0 && angular_z == 0)
       {
           wheel_1_control = linear_x;
           wheel_2_control = linear_x;
           wheel_3_control = linear_x;
           wheel_4_control = linear_x;
       }
       else if(linear_x == 0 && linear_y != 0 && angular_z == 0)
       {
           wheel_1_control = linear_y;
           wheel_2_control = -linear_y;
           wheel_3_control = linear_y;
           wheel_4_control = -linear_y;
       }
       else if(linear_x == 0 && linear_y == 0 && angular_z < 0)
       {
          wheel_1_control = angular_z * 2.2 / 0.065;
          wheel_2_control = 0;
          wheel_3_control = angular_z * 2.2 / 0.065;
          wheel_4_control = 0;
       }
       else if(linear_x == 0 && linear_y == 0 && angular_z > 0)
       {
          wheel_1_control = 0;
          wheel_2_control = angular_z * 2.2 / 0.065;
          wheel_3_control = 0;
          wheel_4_control = angular_z * 2.2 / 0.065;
       }
       else
       {
          wheel_1_control = 0;
          wheel_2_control = 0;
          wheel_3_control = 0;
          wheel_4_control = 0;
       }
       
       //publish the message
       wheel1_command.publish(wheel_1_control);
       wheel2_command.publish(wheel_2_control);
       wheel3_command.publish(wheel_3_control);
       wheel4_command.publish(wheel_4_control);
       loop_rate.sleep();
       ros::spinOnce();
    }
    return 0;
}
