#include <zm_robot_programing/zm_robot_move_function.h>

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "zm_robot_move"); 
  ros::NodeHandle n;

  zm_robot my_zm_robot(ros::this_node::getName());

  ////// zm_robot programing control start //////

  my_zm_robot.move_map(1.0, 3.0, 1.5708);
  
  my_zm_robot.move_base(1.0, 3.0, 1.5708);

  ////// END //////

  return 0;
}
