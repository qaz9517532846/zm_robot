#include <zm_robot_move_relative/zm_move_relative.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "zm_robot_move_relative");

  zm_robot_MoveRelative move_relative(ros::this_node::getName());

  ros::spin();
  return 0;
}
