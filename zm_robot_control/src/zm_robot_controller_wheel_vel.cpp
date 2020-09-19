#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <string>

float linear_x = 0;
float linear_y = 0;
float angular_z = 0;

void call_twist(const geometry_msgs::Twist::ConstPtr& msg)
{
    linear_x = msg->linear.x;
    linear_y = msg->linear.y;
    angular_z = msg->angular.z;
}

int main(int argc, char** argv)
{
    // Node initialization
    ros::init(argc, argv, "zm_robot_controller_wheel_vel");
    ros::NodeHandle n;

    ros::Subscriber odom_pub = n.subscribe("cmd_vel", 50, &call_twist);

    ros::Publisher wheel1_command = n.advertise<std_msgs::Float64>("/wheel1_velocity", 50);
    ros::Publisher wheel2_command = n.advertise<std_msgs::Float64>("/wheel2_velocity", 50);
    ros::Publisher wheel3_command = n.advertise<std_msgs::Float64>("/wheel3_velocity", 50);
    ros::Publisher wheel4_command = n.advertise<std_msgs::Float64>("/wheel4_velocity", 50);

    ros::Rate loop_rate(50);

    while(n.ok())
    {
       std_msgs::Float64 wheel_1_control;
       std_msgs::Float64 wheel_2_control;
       std_msgs::Float64 wheel_3_control;
       std_msgs::Float64 wheel_4_control;

       wheel_1_control.data = (linear_x + linear_y - 2.2 * angular_z) / 0.065;
       wheel_2_control.data = (linear_x - linear_y - 2.2 * angular_z) / 0.065;
       wheel_3_control.data = (linear_x + linear_y + 2.2 * angular_z) / 0.065;
       wheel_4_control.data = (linear_x - linear_y + 2.2 * angular_z) / 0.065;

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
