#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include <string>

float wheel_1_value = 0;
float wheel_2_value = 0;
float wheel_3_value = 0;
float wheel_4_value = 0;

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
    ros::init(argc, argv, "zm_robot_controller_odom");
    ros::NodeHandle n;
    tf::TransformBroadcaster odom_broadcaster;
  
    float x = 0;
    float y = 0;
    float th = 0;
    float vx = 0;
    float vy = 0;
    float vth = 0;

    float joint_1_position = 0;
    float joint_2_position = 0;
    float joint_3_position = 0;
    float joint_4_position = 0;

    int wheel_1_num = 0;
    int wheel_2_num = 0;
    int wheel_3_num = 0;
    int wheel_4_num = 0;

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

    ros::Subscriber wheel1_encoder = n.subscribe("/wheel1_velocity", 1000, &wheel1);
    ros::Subscriber wheel2_encoder = n.subscribe("/wheel2_velocity", 1000, &wheel2);
    ros::Subscriber wheel3_encoder = n.subscribe("/wheel3_velocity", 1000, &wheel3);
    ros::Subscriber wheel4_encoder = n.subscribe("/wheel4_velocity", 1000, &wheel4);

    ros::Publisher joint_states_pub_ = n.advertise<sensor_msgs::JointState>("/joint_states", 100);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    sensor_msgs::JointState joint_states_;

    joint_states_.name.push_back("wheel_joint1");
    joint_states_.name.push_back("wheel_joint2");
    joint_states_.name.push_back("wheel_joint3");
    joint_states_.name.push_back("wheel_joint4");
    joint_states_.position.resize(4, 0.0);
    joint_states_.velocity.resize(4, 0.0);
    joint_states_.effort.resize(4, 0.0);


    ros::Rate loop_rate(50);

    while(n.ok())
    {
       current_time = ros::Time::now(); 
       double dt = (current_time - last_time).toSec();

       vx = 0.065 * (wheel_1_value + wheel_2_value + wheel_3_value + wheel_4_value) / 4 * cos(th) * dt - 0.065 * (wheel_1_value - wheel_2_value + wheel_3_value - wheel_4_value) / 4 * sin(th) * dt;
       vy = 0.065 * (wheel_1_value - wheel_2_value + wheel_3_value - wheel_4_value) / 4 * cos(th) * dt + 0.065 * (wheel_1_value + wheel_2_value + wheel_3_value + wheel_4_value) / 4 * sin(th) * dt;
       vth = 0.065 * (-wheel_1_value - wheel_2_value + wheel_3_value + wheel_4_value) / 0.55 / 4 * dt;

       x = x + vx;
       y = y + vy;
       th = th + vth;

       joint_1_position = joint_1_position + wheel_1_value * dt ;
       joint_2_position = joint_2_position + wheel_2_value * dt ;
       joint_3_position = joint_3_position + wheel_3_value * dt ;
       joint_4_position = joint_4_position + wheel_4_value * dt ;

       wheel_1_num = joint_1_position / 3.141592654 / 2;
       wheel_2_num = joint_2_position / 3.141592654 / 2;
       wheel_3_num = joint_3_position / 3.141592654 / 2;
       wheel_4_num = joint_4_position / 3.141592654 / 2;

       joint_1_position = joint_1_position - wheel_1_num * 2 * 3.141592654;
       joint_2_position = joint_2_position - wheel_2_num * 2 * 3.141592654;
       joint_3_position = joint_3_position - wheel_3_num * 2 * 3.141592654;
       joint_4_position = joint_4_position - wheel_4_num * 2 * 3.141592654;

       joint_states_.position[0] = joint_1_position;
       joint_states_.position[1] = joint_2_position;
       joint_states_.position[2] = joint_3_position;
       joint_states_.position[3] = joint_4_position;
       joint_states_.velocity[0] = wheel_1_value;
       joint_states_.velocity[1] = wheel_2_value;
       joint_states_.velocity[2] = wheel_3_value;
       joint_states_.velocity[3] = wheel_2_value;

       geometry_msgs::TransformStamped odom_trans;
       geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
		
       odom_trans.header.stamp = current_time;
       odom_trans.header.frame_id = "odom";
       odom_trans.child_frame_id = "base_footprint";
       odom_trans.transform.translation.x = x;
       odom_trans.transform.translation.y = y;
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
       odom.pose.pose.position.x = x;
       odom.pose.pose.position.y = y;
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
       joint_states_.header.stamp = current_time;
       joint_states_pub_.publish(joint_states_);
       last_time = current_time;
       loop_rate.sleep();
       ros::spinOnce();
    }
    return 0;
}
