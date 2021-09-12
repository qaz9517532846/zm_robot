#include "zm_safety/zm_safety.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "zm_safety_node");

	ZMSafety rs;
	rs.spin();

	return 0;
}