#include <zm_robot_programing/zm_robot_move_function.h>

zm_robot::zm_robot(std::string name)
{
  boost::thread spin_thread = boost::thread(boost::bind(&zm_robot::spinThread, this));
}

zm_robot::~zm_robot()
{
  
}

void zm_robot::spinThread()
{
  ros::spin();
}

void zm_robot::move_map(double x, double y, double theta)
{
  MoveBaseClient ac("move_base");

  //give some time for connections to register
  sleep(2.0);

  //we'll send a goal to the robot to move 2 meters forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base move.");
  else
    ROS_INFO("The base failed to move.");
}

void zm_robot::move_base(double x, double y, double theta)
{
  MoveBaseClient ac("move_base");

  //give some time for connections to register
  sleep(2.0);

  //we'll send a goal to the robot to move 2 meters forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base move.");
  else
    ROS_INFO("The base failed to move.");
}
