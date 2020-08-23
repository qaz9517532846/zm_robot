#include <ros/ros.h>
#include <control_msgs/JointControllerState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include <string>

float wheel_1_value, wheel_2_value, wheel_3_value, wheel_4_value;

void wheel1(const control_msgs::JointControllerState::ConstPtr& msg)
{
    wheel_1_value = msg->process_value;
}

void wheel2(const control_msgs::JointControllerState::ConstPtr& msg)
{
    wheel_2_value = msg->process_value;
}

void wheel3(const control_msgs::JointControllerState::ConstPtr& msg)
{
    wheel_3_value = msg->process_value;
}

void wheel4(const control_msgs::JointControllerState::ConstPtr& msg)
{
    wheel_4_value = msg->process_value;
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

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

    ros::Subscriber wheel1_encoder = n.subscribe("/zm_robot/joint1_velocity_controller/state", 1000, &wheel1);
    ros::Subscriber wheel2_encoder = n.subscribe("/zm_robot/joint2_velocity_controller/state", 1000, &wheel2);
    ros::Subscriber wheel3_encoder = n.subscribe("/zm_robot/joint3_velocity_controller/state", 1000, &wheel3);
    ros::Subscriber wheel4_encoder = n.subscribe("/zm_robot/joint4_velocity_controller/state", 1000, &wheel4);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate loop_rate(50);

    while(n.ok())
    {
       current_time = ros::Time::now(); 
       double dt = (current_time - last_time).toSec();

       vx = 0.065 * (wheel_1_value + wheel_2_value + wheel_3_value + wheel_4_value) / 4 * cos(th) * dt;
       vy = 0.065 * (-wheel_1_value + wheel_2_value - wheel_3_value + wheel_4_value) / 4 * cos(th) * dt;
       vth = 0.065 * (wheel_1_value + wheel_2_value - wheel_3_value - wheel_4_value) / 0.55 / 4 * dt;

       x = x + vx;
       y = y + vy;
       th = th + vth;

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
       last_time = current_time;
       loop_rate.sleep();
       ros::spinOnce();
    }
    return 0;
}
