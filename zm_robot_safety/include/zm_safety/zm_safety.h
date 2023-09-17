#ifndef ZMROBOTSAFETY_H_
#define ZMROBOTSAFETY_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include <zm_robot_safety/zm_RobotSafetyConfig.h>

class ZMSafety
{
public:
	ZMSafety();
	~ZMSafety();

	void spin();

private:
	ros::NodeHandle nh_;

	ros::Publisher cmd_vel_pub_;
	ros::Publisher stop_viz_pub_;
	ros::Publisher slow_viz_pub_;

	ros::Subscriber cmd_vel_sub_;
	ros::Subscriber bumper_sub_;
	std::vector<ros::Subscriber> scan_sub_;

	geometry_msgs::Twist cmd_vel_msg_;

	visualization_msgs::Marker slow_viz_msg_, stop_viz_msg_;

	laser_geometry::LaserProjection projector_;
	tf::TransformListener tfListener_;

	bool stop_bumper_, total_stop, total_slow;
	std::vector<bool> stop_laser_;
	std::vector<bool> slow_laser_;
	std::vector<double> slow_dist_;


	double init_vel_x;
	double init_vel_y;
	double init_vel_th;
	double dist_;

	// params
	double slow_range_l, slow_range_w;
	double stop_range_l, stop_range_w;
	int node_loop_rate_;
	int scan_num_;

	bool unlock_safe_;

	void buildRectangleVizMsgs();

	void CmdVelCallback(const geometry_msgs::TwistConstPtr& msg);
	void bumperCallback(const std_msgs::BoolConstPtr& msg);
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg, int idx);

	void check(sensor_msgs::PointCloud cloud, int idx);

	void inE2(geometry_msgs::Point32 point, int idx);
	double solveE1(geometry_msgs::Point32 point, int idx);

	void visualizeRectangles(bool show = true);
	std::vector<geometry_msgs::Point> draw_range(double rang_w, double rang_l);
	void reconfig_callback(zm_robot_safety::zm_RobotSafetyConfig &config, uint32_t level);
	dynamic_reconfigure::Server<zm_robot_safety::zm_RobotSafetyConfig> reconfigure_server_;
};

#endif 