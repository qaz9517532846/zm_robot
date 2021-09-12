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
	ros::Subscriber scan_sub_;

	geometry_msgs::Twist cmd_vel_msg_;

	visualization_msgs::Marker slow_viz_msg_, stop_viz_msg_;

	laser_geometry::LaserProjection projector_;
	tf::TransformListener tfListener_;

	bool stop_bumper_, stop_laser_, slow_laser_;

	double dist_;

	// params
	double slow_range_l, slow_range_w;
	double stop_range_l, stop_range_w;
	int node_loop_rate_;

	void buildEllipseVizMsgs();

	void CmdVelCallback(const geometry_msgs::TwistConstPtr& msg);
	void bumperCallback(const std_msgs::BoolConstPtr& msg);
	void scanCallback(const sensor_msgs::LaserScanConstPtr& msg);

	void check(sensor_msgs::PointCloud cloud);

	void inE2(geometry_msgs::Point32 point);
	double solveE1(geometry_msgs::Point32 point);

	void visualizeEllipses(bool show = true);
	std::vector<geometry_msgs::Point> draw_range(double rang_w, double rang_l);
};

#endif 