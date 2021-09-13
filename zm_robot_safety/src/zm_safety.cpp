#include "zm_safety/zm_safety.h"

#define PI 3.141592653


ZMSafety::ZMSafety() : nh_("~"), 
                       stop_bumper_(false), 
					   slow_laser_(false),
                       stop_laser_(false),
					   slow_range_l(0.70),
                       slow_range_w(0.30),
					   stop_range_l(0.40),
                       stop_range_w(0.25),
                       node_loop_rate_ (20)
{
    nh_.param("slow_range_l", slow_range_l, 0.70);
	nh_.param("slow_range_w", slow_range_w, 0.30);
	nh_.param("stop_range_l", stop_range_l, 0.40);
	nh_.param("stop_range_w", stop_range_w, 0.25);
	nh_.param("node_loop_rate", node_loop_rate_, 20);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	slow_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("slow_range_marker", 10);
	stop_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("stop_range_marker", 10);

	cmd_vel_sub_ = nh_.subscribe("/zm_cmd_vel", 1, &ZMSafety::CmdVelCallback, this);
	bumper_sub_ = nh_.subscribe("/bumper", 1, &ZMSafety::bumperCallback, this);
	scan_sub_ = nh_.subscribe("/zm_robot_scan", 1, &ZMSafety::scanCallback, this);

	buildEllipseVizMsgs();
}

ZMSafety::~ZMSafety()
{
    cmd_vel_pub_.shutdown();
	cmd_vel_sub_.shutdown();
	bumper_sub_.shutdown();
	scan_sub_.shutdown();
}

void ZMSafety::spin()
{
	ros::Rate lr(node_loop_rate_);
	while(nh_.ok())
	{
		ros::spinOnce();
		lr.sleep();
	}
}

void ZMSafety::buildEllipseVizMsgs()
{
	stop_viz_msg_.header.frame_id = slow_viz_msg_.header.frame_id = "base_link";
	stop_viz_msg_.header.stamp = slow_viz_msg_.header.stamp = ros::Time::now();
	slow_viz_msg_.id = 1;
	stop_viz_msg_.id = 2;
	stop_viz_msg_.ns = "stop_range";
	slow_viz_msg_.ns = "slow_range";
	stop_viz_msg_.action = slow_viz_msg_.action = visualization_msgs::Marker::ADD;
	stop_viz_msg_.type = slow_viz_msg_.type = visualization_msgs::Marker::LINE_STRIP;

	// Color the ellipses green
	stop_viz_msg_.color.g = 1.0;
	stop_viz_msg_.color.a = 1.0;
	slow_viz_msg_.color.g = 1.0;
	slow_viz_msg_.color.a = 1.0;

	// Scale the points
	stop_viz_msg_.scale.x = 0.01;
	slow_viz_msg_.scale.x = 0.01;

	// Now we populate the msgs
	stop_viz_msg_.points = draw_range(stop_range_w, stop_range_l);
	slow_viz_msg_.points = draw_range(slow_range_w, slow_range_l);
}

std::vector<geometry_msgs::Point> ZMSafety::draw_range(double rang_w, double rang_l)
{
	std::vector<geometry_msgs::Point> range_result;

	geometry_msgs::Point draw_point;
	draw_point.x = rang_l / 2;
	draw_point.y = rang_w / 2;

	range_result.push_back(draw_point);

	draw_point.x = rang_l / 2;
	draw_point.y = -rang_w / 2;

	range_result.push_back(draw_point);

	draw_point.x = -rang_l / 2;
	draw_point.y = -rang_w / 2;

	range_result.push_back(draw_point);

	draw_point.x = -rang_l / 2;
	draw_point.y = rang_w / 2;

	range_result.push_back(draw_point);

	draw_point.x = rang_l / 2;
	draw_point.y = rang_w / 2;

	range_result.push_back(draw_point);

	return range_result;
}

void ZMSafety::CmdVelCallback(const geometry_msgs::TwistConstPtr& msg)
{
	init_vel_x = msg->linear.x;
	init_vel_y = msg->linear.y;
	init_vel_th = msg->angular.z;

	cmd_vel_msg_.linear.x = dist_* init_vel_x;
	cmd_vel_msg_.linear.y = dist_ * init_vel_y;
	cmd_vel_msg_.angular.z = dist_ * init_vel_th;

	cmd_vel_pub_.publish(cmd_vel_msg_);
}

void ZMSafety::bumperCallback(const std_msgs::BoolConstPtr& msg)
{
	if(msg->data)
	{
		ROS_ERROR("Bumper hit! Shutting down node!");
		ros::shutdown();
		return;
	}
}

void ZMSafety::scanCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
	sensor_msgs::PointCloud cloud;

	try
	{
		tfListener_.waitForTransform("/base_link", msg->header.frame_id,
                                                   msg->header.stamp + ros::Duration().fromSec(msg->ranges.size()*msg->time_increment),
                                                   ros::Duration(1.0));
		projector_.transformLaserScanToPointCloud("/base_link", *msg, cloud, tfListener_);
	}
	catch(tf::LookupException& ex)
	{
		ROS_WARN("Lookup exception: %s\n", ex.what());
		return;
	}
	catch(tf::ConnectivityException& ex)
	{
		ROS_WARN("Connectivity exception: %s\n", ex.what());
		return;
	}
	catch(tf::ExtrapolationException& ex)
	{
		ROS_WARN("Extrapolation exception: %s\n", ex.what());
		return;
	}
	check(cloud);
	visualizeEllipses();
}

void ZMSafety::check(sensor_msgs::PointCloud cloud)
{
	stop_laser_ = false;
	slow_laser_ = false;
	dist_ = 1.0;

	for(unsigned int i=0; i < cloud.points.size(); ++i)
	{
		inE2(cloud.points[i]);
	}
}

void ZMSafety::inE2(geometry_msgs::Point32 point)
{
	bool check_slow = (2 * abs(point.x) < slow_range_l) && (2 * abs(point.y) < slow_range_w);

	if(check_slow) // Check if the point is in Ellipse 2
	{
		slow_laser_ = true;
		dist_ = solveE1(point);
		bool check_stop = (2 * abs(point.x) < stop_range_l) && (2 * abs(point.y) < stop_range_w);
		if(check_stop)
		{
			dist_ = 0.0;
			stop_laser_ = true;
			return ;
		}
	}
}

double ZMSafety::solveE1(geometry_msgs::Point32 point)
{
	double dis_vel;
	if((2 * abs(point.x) / slow_range_l) >= (2 * abs(point.y) / slow_range_w))
	{
		dis_vel = 2 * abs(point.y) / slow_range_w;
	}
	else
	{
		dis_vel = 2 * abs(point.x) / slow_range_l;
	}

	return dis_vel;
}

void ZMSafety::visualizeEllipses(bool show)
{
	stop_viz_msg_.header.stamp = slow_viz_msg_.header.stamp = ros::Time::now();

	// Color the ellipses green
	stop_viz_msg_.color.r = 0.0;
	stop_viz_msg_.color.g = 1.0;
	stop_viz_msg_.color.b = 0.0;

	slow_viz_msg_.color.r = 0.0;
	slow_viz_msg_.color.g = 1.0;
	slow_viz_msg_.color.b = 0.0;

	if(stop_laser_)
	{
		// Color the ellipse e1 red
		stop_viz_msg_.color.r = 1.0;
		stop_viz_msg_.color.g = 0.0;
		stop_viz_msg_.color.b = 0.0;
	}
	if (slow_laser_)
	{
		// Color the ellipse e2 red
		slow_viz_msg_.color.r = 1.0;
		slow_viz_msg_.color.g = 0.0;
		slow_viz_msg_.color.b = 0.0;
	}

	stop_viz_pub_.publish(stop_viz_msg_);
	slow_viz_pub_.publish(slow_viz_msg_);
}