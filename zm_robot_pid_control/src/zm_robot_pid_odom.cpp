#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include <string>
#include <geometry_msgs/Pose2D.h>

float robot_x = 0;
float robot_y = 0;
float robot_th = 0;

float wheel_1_value = 0;
float wheel_2_value = 0;
float wheel_3_value = 0;
float wheel_4_value = 0;

void zm_robot_position(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    robot_x = msg->x;
    robot_y = msg->y;
    robot_th = msg->theta;
}

void wheel1(const std_msgs::Float64::ConstPtr& msg)
{
    wheel_1_value = msg->data;
}

void wheel2(const std_msgs::Float64::ConstPtr& msg)
{
    wheel_2_value = msg->data;
}

void wheel3(const std_msgs::Float64::ConstPtr& msg)
{
    wheel_3_value = msg->data;
}

void wheel4(const std_msgs::Float64::ConstPtr& msg)
{
    wheel_4_value = msg->data;
}

int main(int argc, char** argv) {
    // Node initialization
    ros::init(argc, argv, "zm_robot_pid_odom");
    ros::NodeHandle n;
    tf::TransformBroadcaster odom_broadcaster;
  
    float vx = 0;
    float vy = 0;
    float vth = 0;

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

    ros::Subscriber wheel1_encoder = n.subscribe("/wheel1_velocity", 1000, &wheel1);
    ros::Subscriber wheel2_encoder = n.subscribe("/wheel2_velocity", 1000, &wheel2);
    ros::Subscriber wheel3_encoder = n.subscribe("/wheel3_velocity", 1000, &wheel3);
    ros::Subscriber wheel4_encoder = n.subscribe("/wheel4_velocity", 1000, &wheel4);

    ros::Subscriber zm_robot_pos = n.subscribe("/zm_robot_position", 1000, &zm_robot_position);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate loop_rate(50);

    while(n.ok())
    {
       current_time = ros::Time::now(); 
       double dt = (current_time - last_time).toSec();

       vx = 0.065 * (wheel_1_value + wheel_2_value + wheel_3_value + wheel_4_value) / 4 * cos(robot_th) * dt - 0.065 * (wheel_1_value - wheel_2_value + wheel_3_value - wheel_4_value) / 4 * sin(robot_th) * dt;
       vy = 0.065 * (wheel_1_value - wheel_2_value + wheel_3_value - wheel_4_value) / 4 * cos(robot_th) * dt + 0.065 * (wheel_1_value + wheel_2_value + wheel_3_value + wheel_4_value) / 4 * sin(robot_th) * dt;
       vth = 0.065 * (-wheel_1_value - wheel_2_value + wheel_3_value + wheel_4_value) / 0.55 / 4 * dt;

       geometry_msgs::TransformStamped odom_trans;
       geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_th);
		
       odom_trans.header.stamp = current_time;
       odom_trans.header.frame_id = "odom";
       odom_trans.child_frame_id = "base_footprint";
       odom_trans.transform.translation.x = robot_x;
       odom_trans.transform.translation.y = robot_y;
       odom_trans.transform.translation.z = 0.0;
       odom_trans.transform.rotation = odom_quat;
		
       //send the transform over /tf
       odom_broadcaster.sendTransform(odom_trans);  
        
       //next, we'll publish the odometry message over ROS
       nav_msgs::Odometry odom;
       odom.header.stamp = current_time;
       odom.header.frame_id = "odom";
		
       //set the position
       // Position
       odom.pose.pose.position.x = robot_x;
       odom.pose.pose.position.y = robot_y;
       odom.pose.pose.position.z = 0.0;
       // Orientation
       odom.pose.pose.orientation = odom_quat;
		
       //set the velocity
       odom.child_frame_id = "base_footprint";
       // Linear velocities
       odom.twist.twist.linear.x = vx;
       odom.twist.twist.linear.y = vy;
       odom.twist.twist.linear.z = 0.0;
       // Angular velocities
       odom.twist.twist.angular.x = 0;
       odom.twist.twist.angular.y = 0;
       odom.twist.twist.angular.z = vth;
       
       //publish the message
       odom_pub.publish(odom);
       loop_rate.sleep();
       last_time = current_time;
       ros::spinOnce();
    }
    return 0;
}
