#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "client");
  //ROS_INFO(ros::this_node::getName() + " start");
 
  ros::NodeHandle n;

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("zm_robot_move_relative", true);
  ac.waitForServer(); //will wait for infinite time

  //std::string Node_name = ros::this_node::getName();

  //ROS_INFO("get server : " + Node_name);
  
  ros::Rate loop_rate(10);
  while(n.ok())
  {
   
    std::string frame_id;
    std::string x, y, theta;

    std::cout << "Please enter frame:";
    std::getline(std::cin, frame_id);
    std::cout << "Please enter x position:";
    std::getline(std::cin, x);
    std::cout << "Please enter y position:";
    std::getline(std::cin, y);
    std::cout << "Please enter theta position:";
    std::getline(std::cin, theta);

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = frame_id;
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = stod(x);
    goal.target_pose.pose.position.y = stod(y);
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(stod(theta));

    //ROS_INFO(goal.c_str());

    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base move.");
    else
      ROS_INFO("The base failed to move.");
  }

  return 0;
}
